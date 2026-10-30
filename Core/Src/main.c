/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vl53l0x.h"
#include "ssd1306.h"
#include "warning.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_SAMPLE_INTERVAL_MS   200U
#define APP_REPORT_INTERVAL_MS   500U
#define APP_OLED_INTERVAL_MS    1000U
#define APP_SENSOR_RETRY_MS     2000U
#define MAX_DEMO_DISTANCE_MM    2000U   /* 2 m, outside -> NO TARGET */

/* Pin assignment based on the report's draft schematic. Active-high. */
#define LED_GREEN_PIN  GPIO_PIN_1   /* PA1 */
#define LED_YELLOW_PIN GPIO_PIN_2   /* PA2 */
#define LED_RED_PIN    GPIO_PIN_3   /* PA3 */
#define BUZZER_PIN     GPIO_PIN_11  /* PB11, through NPN driver */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static VL53L0X_Device vl53;
static Warning_Tracker warning_tracker;
static uint8_t sensor_ready = 0U;
static uint8_t oled_ready = 0U;
static uint8_t vl53_raw_status = 0U;
static uint16_t last_distance_mm = 0U;
static VL53L0X_Result last_range_result = VL53L0X_ERR_NOT_INITIALIZED;
static uint32_t last_sample_tick;
static uint32_t last_report_tick;
static uint32_t last_oled_tick;
static uint32_t last_retry_tick;
static char message[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void App_IO_Init(void);
static void App_SetWarnings(uint32_t now_ms);
static void App_ReadSensor(void);
static void App_SendText(const char *str);
static void App_Report(void);
static void App_DrawOLED(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void App_SendText(const char *str)
{
  if (str == NULL) return;
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)str,
                          (uint16_t)strlen(str), 200U);
}

/* Configure GPIO only within USER CODE, keeping CubeMX functions untouched.
   Later, you may configure these pins in the .ioc instead. */
static void App_IO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, LED_GREEN_PIN | LED_YELLOW_PIN | LED_RED_PIN,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, BUZZER_PIN, GPIO_PIN_RESET);

  gpio.Pin = LED_GREEN_PIN | LED_YELLOW_PIN | LED_RED_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = BUZZER_PIN;
  HAL_GPIO_Init(GPIOB, &gpio);
}

static void App_SetWarnings(uint32_t now_ms)
{
  GPIO_PinState green = GPIO_PIN_RESET;
  GPIO_PinState yellow = GPIO_PIN_RESET;
  GPIO_PinState red = GPIO_PIN_RESET;
  GPIO_PinState buzzer = GPIO_PIN_RESET;

  switch (warning_tracker.state) {
  case WARNING_SAFE:
  case WARNING_NO_TARGET:
    /* Green = no detected collision risk; NO TARGET is not a valid distance. */
    green = GPIO_PIN_SET;
    break;
  case WARNING_CAUTION:
    /* Yellow 1 Hz flash, short beep once per second. */
    yellow = ((now_ms % 1000U) < 500U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    buzzer = ((now_ms % 1000U) < 120U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    break;
  case WARNING_CRITICAL:
    red = ((now_ms % 250U) < 125U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    buzzer = GPIO_PIN_SET;
    break;
  default:
    /* Missing/invalid measurement must not be displayed as SAFE. */
    red = ((now_ms % 1000U) < 250U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    buzzer = ((now_ms % 1000U) < 100U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    break;
  }
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_PIN, green);
  HAL_GPIO_WritePin(GPIOA, LED_YELLOW_PIN, yellow);
  HAL_GPIO_WritePin(GPIOA, LED_RED_PIN, red);
  HAL_GPIO_WritePin(GPIOB, BUZZER_PIN, buzzer);
}

static void App_Report(void)
{
  uint32_t abs_speed;
  char direction;
    char ttc_text[20];

    if (warning_tracker.state == WARNING_NO_TARGET) {
      (void)snprintf(message, sizeof(message),
                     "[NO TARGET] beyond demo range; raw=%umm, status=0x%02X\r\n",
                     (unsigned int)last_distance_mm,
                     (unsigned int)vl53_raw_status);
      App_SendText(message);
      return;
    }

  if (warning_tracker.state == WARNING_FAULT) {
    (void)snprintf(message, sizeof(message),
                   "[FAULT] raw=%umm status=0x%02X; VL53=%s; step=%s; I2C=0x%08lX\r\n",
                   (unsigned int)last_distance_mm,
                   (unsigned int)vl53_raw_status,
                   VL53L0X_ResultText(last_range_result),
                   (vl53.last_step != NULL) ? vl53.last_step : "unknown",
                   (unsigned long)vl53.hal_i2c_error);
    App_SendText(message);
    return;
  }

  direction = (warning_tracker.closing_mmps < 0) ? '-' : '+';
  abs_speed = (warning_tracker.closing_mmps < 0) ?
      (uint32_t)(-warning_tracker.closing_mmps) :
      (uint32_t)warning_tracker.closing_mmps;

  if (warning_tracker.ttc_ms == WARNING_TTC_UNKNOWN)
    (void)snprintf(ttc_text, sizeof(ttc_text), "--");
  else
    (void)snprintf(ttc_text, sizeof(ttc_text), "%lu.%lus",
                   (unsigned long)(warning_tracker.ttc_ms / 1000UL),
                   (unsigned long)((warning_tracker.ttc_ms % 1000UL) / 100UL));

  (void)snprintf(message, sizeof(message),
                 "D=%ld.%ldcm | V=%c%lu.%lucm/s | TTC=%s | %s\r\n",
                 (long)(warning_tracker.filtered_mm / 10),
                 (long)(warning_tracker.filtered_mm % 10),
                 direction, (unsigned long)(abs_speed / 10U),
                 (unsigned long)(abs_speed % 10U), ttc_text,
                 Warning_StateText(warning_tracker.state));
  App_SendText(message);
}

static void App_DrawOLED(void)
{
  char line[30];
  uint32_t abs_speed;
  char direction;

  if (oled_ready == 0U) return;
  SSD1306_Clear();

  if (warning_tracker.state == WARNING_FAULT) {
    SSD1306_Print(0U, 0U, "VL53L0X ERROR");
    SSD1306_Print(0U, 2U, "NO VALID RANGE");
    SSD1306_Print(0U, 6U, "STATE: FAULT");
  } else if (warning_tracker.state == WARNING_NO_TARGET) {
    SSD1306_Print(0U, 0U, "NO TARGET");
    SSD1306_Print(0U, 2U, "OUT OF RANGE");
    SSD1306_Print(0U, 4U, "TTC: --");
    SSD1306_Print(0U, 6U, "STATE: NO TARGET");
  } else {
    (void)snprintf(line, sizeof(line), "DIST: %ld.%ld CM",
                   (long)(warning_tracker.filtered_mm / 10),
                   (long)(warning_tracker.filtered_mm % 10));
    SSD1306_Print(0U, 0U, line);

    direction = (warning_tracker.closing_mmps < 0) ? '-' : '+';
    abs_speed = (warning_tracker.closing_mmps < 0) ?
        (uint32_t)(-warning_tracker.closing_mmps) :
        (uint32_t)warning_tracker.closing_mmps;
    (void)snprintf(line, sizeof(line), "SPEED: %c%lu.%lu CM/S",
                   direction, (unsigned long)(abs_speed / 10U),
                   (unsigned long)(abs_speed % 10U));
    SSD1306_Print(0U, 2U, line);

    if (warning_tracker.ttc_ms == WARNING_TTC_UNKNOWN) {
      SSD1306_Print(0U, 4U, "TTC: --");
    } else {
      (void)snprintf(line, sizeof(line), "TTC: %lu.%lu S",
                     (unsigned long)(warning_tracker.ttc_ms / 1000UL),
                     (unsigned long)((warning_tracker.ttc_ms % 1000UL) / 100UL));
      SSD1306_Print(0U, 4U, line);
    }
    (void)snprintf(line, sizeof(line), "STATE: %s",
                   Warning_StateText(warning_tracker.state));
    SSD1306_Print(0U, 6U, line);
  }

  if (SSD1306_Update() != HAL_OK) {
    oled_ready = 0U;
    App_SendText("[WARN] OLED I2C transfer failed; UART continues.\r\n");
  }
}

/* 1. Read VL53L0X; 2. Identify sensor error / no target / normal range.
 * 3. If a range exists, calculate the distance, closing speed and TTC. */
static void App_ReadSensor(void)
{
  uint16_t mm = 0U;

  last_range_result = VL53L0X_ReadSingleMM(&vl53, &mm, &vl53_raw_status);
  last_distance_mm = mm;

  if (last_range_result != VL53L0X_OK) {
    /* True communication/timeout problem: red fault flash and retry init. */
    Warning_Fault(&warning_tracker);
    sensor_ready = 0U;
    last_retry_tick = HAL_GetTick();
  } else if (mm > MAX_DEMO_DISTANCE_MM) {
    /* I2C transfer OK, but range outside demo window (often 8190/8191).
     * This is a DEMO heuristic, not a fully decoded ST RangeStatus. */
    Warning_NoTarget(&warning_tracker);
  } else if (mm < 20U) {
    /* 0 or implausibly tiny range: do not mistakenly declare SAFE. */
    Warning_Fault(&warning_tracker);
  } else {
    Warning_Process(&warning_tracker, mm, HAL_GetTick());
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  App_IO_Init();
  Warning_Reset(&warning_tracker);
  App_SendText("\r\nSTM32F103 COLLISION WARNING DEMO\r\n");
  App_SendText("PA1=GREEN PA2=YELLOW PA3=RED PB11=BUZZER\r\n");
  App_SendText("I2C1 100kHz; UART1 115200 8N1; not safety certified\r\n");

  HAL_Delay(100U);
  oled_ready = (SSD1306_Init(&hi2c1) == HAL_OK) ? 1U : 0U;
  App_SendText(oled_ready ? "[OK] OLED SSD1306\r\n" :
                            "[WARN] OLED not found (try 0x3D)\r\n");

  last_range_result = VL53L0X_Init(&vl53, &hi2c1);
  sensor_ready = (last_range_result == VL53L0X_OK) ? 1U : 0U;
  App_SendText(sensor_ready ? "[OK] VL53L0X initialized\r\n" :
                             "[FAIL] VL53L0X init; will retry\r\n");

  last_sample_tick = HAL_GetTick() - APP_SAMPLE_INTERVAL_MS;
  last_report_tick = HAL_GetTick();
  last_oled_tick = HAL_GetTick() - APP_OLED_INTERVAL_MS;
  last_retry_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms = HAL_GetTick();

    /* Re-try a failed sensor without hanging the whole application. */
    if (sensor_ready == 0U &&
        (uint32_t)(now_ms - last_retry_tick) >= APP_SENSOR_RETRY_MS) {
      last_retry_tick = now_ms;
      last_range_result = VL53L0X_Init(&vl53, &hi2c1);
      sensor_ready = (last_range_result == VL53L0X_OK) ? 1U : 0U;
      if (sensor_ready != 0U) {
        App_SendText("[OK] VL53L0X recovered\r\n");
        last_sample_tick = HAL_GetTick() - APP_SAMPLE_INTERVAL_MS;
      }
    }

    if (sensor_ready != 0U &&
        (uint32_t)(now_ms - last_sample_tick) >= APP_SAMPLE_INTERVAL_MS) {
      last_sample_tick = now_ms;
      App_ReadSensor();
    }

    now_ms = HAL_GetTick();
    App_SetWarnings(now_ms);

    if ((uint32_t)(now_ms - last_report_tick) >= APP_REPORT_INTERVAL_MS) {
      last_report_tick = now_ms;
      App_Report();
    }
    if ((uint32_t)(now_ms - last_oled_tick) >= APP_OLED_INTERVAL_MS) {
      last_oled_tick = now_ms;
      App_DrawOLED();
    }
    HAL_Delay(10U);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
