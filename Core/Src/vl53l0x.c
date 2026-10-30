/*
 * Minimal VL53L0X single-shot ranging driver for STM32F1 HAL.
 * Ranging initialization sequences follow the ST VL53L0X API conventions;
 * default tuning values are also used in Pololu's MIT-licensed VL53L0X driver.
 * Not a replacement for ST's complete API (sigma/offset/crosstalk checks).
 *
 * References:
 *   ST VL53L0X datasheet, DocID029104 Rev 1, sections 2-3.
 *   https://www.st.com/en/embedded-software/stsw-img005.html
 *   https://github.com/pololu/vl53l0x-arduino
 */
#include "vl53l0x.h"
#include <stddef.h>

#define REG_SYSRANGE_START           0x00U
#define REG_SEQUENCE_CONFIG          0x01U
#define REG_INTERRUPT_CONFIG         0x0AU
#define REG_INTERRUPT_CLEAR          0x0BU
#define REG_INTERRUPT_STATUS         0x13U
#define REG_RESULT_RANGE_STATUS      0x14U
#define REG_SPAD_MAP                 0xB0U
#define REG_MODEL_ID                 0xC0U
#define REG_PAD_CONFIG               0x89U
#define REG_MSRC_CONTROL             0x60U
#define REG_SIGNAL_RATE_LIMIT        0x44U

#define I2C_IO_TIMEOUT_MS            100U
#define POLL_TIMEOUT_MS              500U

#define TRY(call) do { VL53L0X_Result _r = (call); \
                       if (_r != VL53L0X_OK) return _r; } while (0)

/* LEVEL 1: read/write a register over STM32 HAL I2C.
 * LEVEL 2: r8/w8/r16/w16 below use these two building blocks.
 * LEVEL 3: Init/ReadSingleMM below combine them into sensor operations. */

static VL53L0X_Result reg_read(VL53L0X_Device *d, uint8_t reg,
                               uint8_t *data, uint16_t length)
{
    if (HAL_I2C_Mem_Read(d->i2c, VL53L0X_ADDR_HAL, reg,
                         I2C_MEMADD_SIZE_8BIT, data, length,
                         I2C_IO_TIMEOUT_MS) != HAL_OK) {
        d->hal_i2c_error = HAL_I2C_GetError(d->i2c);
        return VL53L0X_ERR_I2C;
    }
    return VL53L0X_OK;
}

static VL53L0X_Result reg_write(VL53L0X_Device *d, uint8_t reg,
                                const uint8_t *data, uint16_t length)
{
    if (HAL_I2C_Mem_Write(d->i2c, VL53L0X_ADDR_HAL, reg,
                          I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, length,
                          I2C_IO_TIMEOUT_MS) != HAL_OK) {
        d->hal_i2c_error = HAL_I2C_GetError(d->i2c);
        return VL53L0X_ERR_I2C;
    }
    return VL53L0X_OK;
}

static VL53L0X_Result r8(VL53L0X_Device *d, uint8_t reg, uint8_t *v)
{
    return reg_read(d, reg, v, 1U);
}

static VL53L0X_Result w8(VL53L0X_Device *d, uint8_t reg, uint8_t v)
{
    return reg_write(d, reg, &v, 1U);
}

static VL53L0X_Result r16(VL53L0X_Device *d, uint8_t reg, uint16_t *v)
{
    uint8_t data[2];
    TRY(reg_read(d, reg, data, 2U));
    *v = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    return VL53L0X_OK;
}

static VL53L0X_Result w16(VL53L0X_Device *d, uint8_t reg, uint16_t v)
{
    uint8_t data[2];
    data[0] = (uint8_t)(v >> 8);
    data[1] = (uint8_t)v;
    return reg_write(d, reg, data, 2U);
}

static VL53L0X_Result wait_mask(VL53L0X_Device *d, uint8_t reg,
                                uint8_t mask, uint8_t want_set)
{
    uint32_t start = HAL_GetTick();
    uint8_t value;
    do {
        TRY(r8(d, reg, &value));
        if (((value & mask) != 0U) == (want_set != 0U)) {
            return VL53L0X_OK;
        }
        HAL_Delay(1U);
    } while ((uint32_t)(HAL_GetTick() - start) < POLL_TIMEOUT_MS);
    return VL53L0X_ERR_TIMEOUT;
}

static VL53L0X_Result read_spad_info(VL53L0X_Device *d,
                                     uint8_t *spad_count,
                                     uint8_t *is_aperture)
{
    uint8_t value;
    d->last_step = "SPAD info";
    TRY(w8(d, 0x80, 0x01));
    TRY(w8(d, 0xFF, 0x01));
    TRY(w8(d, 0x00, 0x00));
    TRY(w8(d, 0xFF, 0x06));
    TRY(r8(d, 0x83, &value));
    TRY(w8(d, 0x83, (uint8_t)(value | 0x04U)));
    TRY(w8(d, 0xFF, 0x07));
    TRY(w8(d, 0x81, 0x01));
    TRY(w8(d, 0x80, 0x01));
    TRY(w8(d, 0x94, 0x6B));
    TRY(w8(d, 0x83, 0x00));
    TRY(wait_mask(d, 0x83, 0xFF, 1U));
    TRY(w8(d, 0x83, 0x01));
    TRY(r8(d, 0x92, &value));
    *spad_count = (uint8_t)(value & 0x7FU);
    *is_aperture = (uint8_t)(value >> 7);
    TRY(w8(d, 0x81, 0x00));
    TRY(w8(d, 0xFF, 0x06));
    TRY(r8(d, 0x83, &value));
    TRY(w8(d, 0x83, (uint8_t)(value & (uint8_t)~0x04U)));
    TRY(w8(d, 0xFF, 0x01));
    TRY(w8(d, 0x00, 0x01));
    TRY(w8(d, 0xFF, 0x00));
    TRY(w8(d, 0x80, 0x00));
    if (*spad_count == 0U || *spad_count > 48U) return VL53L0X_ERR_SPAD;
    return VL53L0X_OK;
}

/* Default tuning is an ordered register/value script. Order matters. */
static const uint8_t default_tuning[][2] = {
    {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},
    {0x10,0x00},{0x11,0x00},{0x24,0x01},{0x25,0xFF},
    {0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
    {0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},
    {0x31,0x04},{0x32,0x03},{0x40,0x83},{0x46,0x25},
    {0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
    {0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},
    {0x62,0x00},{0x64,0x00},{0x65,0x00},{0x66,0xA0},
    {0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
    {0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},
    {0x78,0x21},{0xFF,0x01},{0x23,0x34},{0x42,0x00},
    {0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
    {0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},
    {0x34,0x03},{0x35,0x44},{0xFF,0x01},{0x31,0x04},
    {0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
    {0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},
    {0x67,0x00},{0x70,0x04},{0x71,0x01},{0x72,0xFE},
    {0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
    {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},
    {0x8E,0x01},{0x00,0x01},{0xFF,0x00},{0x80,0x00}
};

static VL53L0X_Result reference_calibration(VL53L0X_Device *d, uint8_t start)
{
    TRY(w8(d, REG_SYSRANGE_START, (uint8_t)(start | 0x01U)));
    TRY(wait_mask(d, REG_INTERRUPT_STATUS, 0x07, 1U));
    TRY(w8(d, REG_INTERRUPT_CLEAR, 0x01));
    TRY(w8(d, REG_SYSRANGE_START, 0x00));
    return VL53L0X_OK;
}

static uint16_t decode_timeout(uint16_t reg)
{
    uint32_t multiplier = (uint32_t)(reg >> 8);
    if (multiplier > 15U) multiplier = 15U;
    return (uint16_t)(((uint32_t)(reg & 0xFFU) << multiplier) + 1U);
}

static uint16_t encode_timeout(uint32_t mclks)
{
    uint16_t exp = 0U;
    uint32_t mantissa;
    if (mclks == 0U) return 0U;
    mantissa = mclks - 1U;
    while (mantissa > 255U && exp < 255U) {
        mantissa >>= 1;
        exp++;
    }
    return (uint16_t)((exp << 8) | (mantissa & 0xFFU));
}

static uint32_t macro_period_ns(uint8_t vcsel_pclks)
{
    return (uint32_t)((2304UL * vcsel_pclks * 1655UL + 500UL) / 1000UL);
}

static uint32_t mclks_to_us(uint32_t mclks, uint8_t vcsel)
{
    return (mclks * macro_period_ns(vcsel) + 500UL) / 1000UL;
}

static uint32_t us_to_mclks(uint32_t us, uint8_t vcsel)
{
    return (us * 1000UL + macro_period_ns(vcsel) / 2UL) /
            macro_period_ns(vcsel);
}

/* Apply a simple 33 ms timing budget for the standard 0xE8 sequence. */
static VL53L0X_Result configure_timing(VL53L0X_Device *d)
{
    uint8_t pre_period_reg, final_period_reg, msrc_timeout_reg;
    uint16_t pre_encoded, pre_mclks;
    uint8_t pre_pclks, final_pclks;
    uint32_t used_us, remain_us, final_mclks;
    const uint32_t budget_us = 33000UL;

    d->last_step = "timing budget";
    TRY(r8(d, 0x50, &pre_period_reg));
    TRY(r8(d, 0x70, &final_period_reg));
    TRY(r8(d, 0x46, &msrc_timeout_reg));
    TRY(r16(d, 0x51, &pre_encoded));

    pre_pclks = (uint8_t)((pre_period_reg + 1U) * 2U);
    final_pclks = (uint8_t)((final_period_reg + 1U) * 2U);
    pre_mclks = decode_timeout(pre_encoded);
    /* Sequence 0xE8: DSS, pre-range, final-range. */
    used_us = 1910UL + 960UL +
              2UL * (mclks_to_us((uint32_t)msrc_timeout_reg + 1UL,
                                  pre_pclks) + 690UL) +
              mclks_to_us(pre_mclks, pre_pclks) + 660UL + 550UL;

    if (used_us >= budget_us) return VL53L0X_ERR_BUDGET;
    remain_us = budget_us - used_us;
    final_mclks = us_to_mclks(remain_us, final_pclks) + pre_mclks;
    TRY(w16(d, 0x71, encode_timeout(final_mclks)));
    return VL53L0X_OK;
}

VL53L0X_Result VL53L0X_Init(VL53L0X_Device *d, I2C_HandleTypeDef *i2c)
{
    /* Keep the register sequence in order: device-specific tuning is NOT
     * ordinary CubeMX configuration. Read HUONG_DAN_DE_HOC.md first. */
    uint8_t value, spad_count, aperture;
    uint8_t map[6];
    uint8_t enabled = 0U;
    uint8_t first;
    uint8_t i;
    uint32_t k;

    if (d == NULL || i2c == NULL) return VL53L0X_ERR_PARAM;
    d->i2c = i2c;
    d->initialized = 0U;
    d->stop_variable = 0U;
    d->hal_i2c_error = 0U;
    d->last_step = "I2C ACK";
    /* Step 1: Make sure the sensor replies to address 0x29. */
    if (HAL_I2C_IsDeviceReady(i2c, VL53L0X_ADDR_HAL, 3U,
                              I2C_IO_TIMEOUT_MS) != HAL_OK) {
        d->hal_i2c_error = HAL_I2C_GetError(i2c);
        return VL53L0X_ERR_I2C;
    }
    d->last_step = "model ID";
    /* Step 2: Check VL53L0X identification register (0xC0 = 0xEE). */
    TRY(r8(d, REG_MODEL_ID, &value));
    if (value != 0xEEU) return VL53L0X_ERR_ID;

    d->last_step = "base init";
    /* Step 3: Prepare pad, clock and stop variable. */
    /* 3.3 V I/O module: enable 2V8-capable sensor pad mode. */
    TRY(r8(d, REG_PAD_CONFIG, &value));
    TRY(w8(d, REG_PAD_CONFIG, (uint8_t)(value | 0x01U)));
    TRY(w8(d, 0x88, 0x00));
    TRY(w8(d, 0x80, 0x01));
    TRY(w8(d, 0xFF, 0x01));
    TRY(w8(d, 0x00, 0x00));
    TRY(r8(d, 0x91, &d->stop_variable));
    TRY(w8(d, 0x00, 0x01));
    TRY(w8(d, 0xFF, 0x00));
    TRY(w8(d, 0x80, 0x00));
    TRY(r8(d, REG_MSRC_CONTROL, &value));
    TRY(w8(d, REG_MSRC_CONTROL, (uint8_t)(value | 0x12U)));
    TRY(w16(d, REG_SIGNAL_RATE_LIMIT, 0x0020));
    TRY(w8(d, REG_SEQUENCE_CONFIG, 0xFF));

    TRY(read_spad_info(d, &spad_count, &aperture));
    /* Step 4: Configure reference SPADs (optical receiving elements). */
    d->last_step = "SPAD map";
    TRY(reg_read(d, REG_SPAD_MAP, map, 6U));
    TRY(w8(d, 0xFF, 0x01));
    TRY(w8(d, 0x4F, 0x00));
    TRY(w8(d, 0x4E, 0x2C));
    TRY(w8(d, 0xFF, 0x00));
    TRY(w8(d, 0xB6, 0xB4));
    first = (aperture != 0U) ? 12U : 0U;
    for (i = 0U; i < 48U; ++i) {
        if (i < first || enabled >= spad_count) {
            map[i / 8U] &= (uint8_t)~(1U << (i % 8U));
        } else if ((map[i / 8U] & (1U << (i % 8U))) != 0U) {
            enabled++;
        }
    }
    if (enabled != spad_count) return VL53L0X_ERR_SPAD;
    TRY(reg_write(d, REG_SPAD_MAP, map, 6U));

    d->last_step = "tuning sequence";
    /* Step 5: Apply vendor/reference tuning table exactly as ordered. */
    for (k = 0U; k < sizeof(default_tuning) / sizeof(default_tuning[0]); ++k) {
        TRY(w8(d, default_tuning[k][0], default_tuning[k][1]));
    }
    d->last_step = "interrupt config";
    TRY(w8(d, REG_INTERRUPT_CONFIG, 0x04));
    TRY(r8(d, 0x84, &value));
    TRY(w8(d, 0x84, (uint8_t)(value & (uint8_t)~0x10U)));
    TRY(w8(d, REG_INTERRUPT_CLEAR, 0x01));
    TRY(w8(d, REG_SEQUENCE_CONFIG, 0xE8));
    TRY(configure_timing(d));

    d->last_step = "VHV calibration";
    /* Step 6: Two calibration passes before first ranging operation. */
    TRY(w8(d, REG_SEQUENCE_CONFIG, 0x01));
    TRY(reference_calibration(d, 0x40));
    d->last_step = "phase calibration";
    TRY(w8(d, REG_SEQUENCE_CONFIG, 0x02));
    TRY(reference_calibration(d, 0x00));
    TRY(w8(d, REG_SEQUENCE_CONFIG, 0xE8));
    d->initialized = 1U;
    d->last_step = "ready";
    return VL53L0X_OK;
}

VL53L0X_Result VL53L0X_ReadSingleMM(VL53L0X_Device *d,
                                     uint16_t *distance_mm,
                                     uint8_t *raw_range_status)
{
    uint8_t raw[12];
    if (d == NULL || distance_mm == NULL || raw_range_status == NULL)
        return VL53L0X_ERR_PARAM;
    if (d->initialized == 0U) return VL53L0X_ERR_NOT_INITIALIZED;

    d->last_step = "start single range";
    TRY(w8(d, 0x80, 0x01));
    TRY(w8(d, 0xFF, 0x01));
    TRY(w8(d, 0x00, 0x00));
    TRY(w8(d, 0x91, d->stop_variable));
    TRY(w8(d, 0x00, 0x01));
    TRY(w8(d, 0xFF, 0x00));
    TRY(w8(d, 0x80, 0x00));
    TRY(w8(d, REG_SYSRANGE_START, 0x01));

    d->last_step = "wait start cleared";
    TRY(wait_mask(d, REG_SYSRANGE_START, 0x01, 0U));
    d->last_step = "wait measurement ready";
    TRY(wait_mask(d, REG_INTERRUPT_STATUS, 0x07, 1U));
    d->last_step = "read result";
    /* Registers 0x14-0x1F contain status and range; mm is 0x1E:0x1F. */
    TRY(reg_read(d, REG_RESULT_RANGE_STATUS, raw, 12U));
    *raw_range_status = raw[0];
    *distance_mm = (uint16_t)(((uint16_t)raw[10] << 8) | raw[11]);
    TRY(w8(d, REG_INTERRUPT_CLEAR, 0x01));
    d->last_step = "ready";
    return VL53L0X_OK;
}

const char *VL53L0X_ResultText(VL53L0X_Result r)
{
    switch (r) {
    case VL53L0X_OK: return "OK";
    case VL53L0X_ERR_PARAM: return "bad argument";
    case VL53L0X_ERR_I2C: return "I2C error";
    case VL53L0X_ERR_ID: return "model ID mismatch";
    case VL53L0X_ERR_TIMEOUT: return "measurement timeout";
    case VL53L0X_ERR_SPAD: return "reference SPAD setup error";
    case VL53L0X_ERR_BUDGET: return "timing budget error";
    case VL53L0X_ERR_NOT_INITIALIZED: return "driver not initialized";
    default: return "unknown error";
    }
}
