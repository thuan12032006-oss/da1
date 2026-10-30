#ifndef SIMPLE_SSD1306_H
#define SIMPLE_SSD1306_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* Most 128x64 OLED modules use 7-bit address 0x3C. Some use 0x3D. */
#define SSD1306_I2C_ADDR_HAL (0x3CU << 1)

HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *hi2c);
void SSD1306_Clear(void);
void SSD1306_Print(uint8_t x_pixel, uint8_t page, const char *text);
HAL_StatusTypeDef SSD1306_Update(void);

#endif
