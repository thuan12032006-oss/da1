#ifndef VL53L0X_DRIVER_H
#define VL53L0X_DRIVER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* The VL53L0X uses 7-bit address 0x29. STM32 HAL takes address << 1. */
#define VL53L0X_ADDR_HAL  (0x29U << 1)

typedef enum {
    VL53L0X_OK = 0,
    VL53L0X_ERR_PARAM,
    VL53L0X_ERR_I2C,
    VL53L0X_ERR_ID,
    VL53L0X_ERR_TIMEOUT,
    VL53L0X_ERR_SPAD,
    VL53L0X_ERR_BUDGET,
    VL53L0X_ERR_NOT_INITIALIZED
} VL53L0X_Result;

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint8_t stop_variable;
    uint8_t initialized;
    uint32_t hal_i2c_error;
    const char *last_step;
} VL53L0X_Device;

/* Driver interface for main.c:
 * Init: called once after CubeMX MX_I2C1_Init().
 * ReadSingleMM: returns I2C/result code and writes the measured mm.
 * raw_range_status is a raw register byte, not ST API RangeStatus.
 * No GPIO1 wiring is needed for this blocking single-shot demo. */
VL53L0X_Result VL53L0X_Init(VL53L0X_Device *dev, I2C_HandleTypeDef *i2c);
VL53L0X_Result VL53L0X_ReadSingleMM(VL53L0X_Device *dev,
                                     uint16_t *distance_mm,
                                     uint8_t *raw_range_status);
const char *VL53L0X_ResultText(VL53L0X_Result result);

#endif
