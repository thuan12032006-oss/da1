#include "ssd1306.h"
#include <stddef.h>
#include <string.h>

#define SSD1306_W 128U
#define SSD1306_H 64U
#define SSD1306_PAGES (SSD1306_H / 8U)
#define SSD1306_TIMEOUT_MS 100U

static I2C_HandleTypeDef *lcd_i2c;
static uint8_t framebuffer[SSD1306_W * SSD1306_PAGES];

/* Five columns per character, least significant bit = top pixel. */
static const uint8_t number_font[10][5] = {
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}
};

static const uint8_t letter_font[26][5] = {
    {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */
    {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */
    {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x09,0x01}, /* F */
    {0x3E,0x41,0x49,0x49,0x7A}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */
    {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */
    {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */
    {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */
    {0x3F,0x40,0x38,0x40,0x3F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */
    {0x07,0x08,0x70,0x08,0x07}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}  /* Z */
};

static const uint8_t *glyph(char c)
{
    static const uint8_t blank[5] = {0,0,0,0,0};
    static const uint8_t dot[5] = {0,0x60,0x60,0,0};
    static const uint8_t colon[5] = {0,0x36,0x36,0,0};
    static const uint8_t minus[5] = {0x08,0x08,0x08,0x08,0x08};
    static const uint8_t slash[5] = {0x20,0x10,0x08,0x04,0x02};
    static const uint8_t plus[5] = {0x08,0x08,0x3E,0x08,0x08};
    if (c >= '0' && c <= '9') return number_font[c - '0'];
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    if (c >= 'A' && c <= 'Z') return letter_font[c - 'A'];
    if (c == '.') return dot;
    if (c == ':') return colon;
    if (c == '-') return minus;
    if (c == '/') return slash;
    if (c == '+') return plus;
    return blank;
}

static HAL_StatusTypeDef command(uint8_t cmd)
{
    uint8_t buf[2] = {0x00U, cmd};
    if (lcd_i2c == NULL) return HAL_ERROR;
    return HAL_I2C_Master_Transmit(lcd_i2c, SSD1306_I2C_ADDR_HAL,
                                   buf, sizeof(buf), SSD1306_TIMEOUT_MS);
}

HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *hi2c)
{
    static const uint8_t init_seq[] = {
        0xAE,       /* display off */
        0xD5,0x80,  /* clock divider */
        0xA8,0x3F,  /* multiplex 64 */
        0xD3,0x00,  /* display offset */
        0x40,       /* start line 0 */
        0x8D,0x14,  /* enable charge pump */
        0x20,0x00,  /* horizontal addressing */
        0xA1,       /* segment remap */
        0xC8,       /* scan direction remap */
        0xDA,0x12,  /* COM pins for 128x64 */
        0x81,0x7F,  /* contrast */
        0xD9,0xF1,  /* pre-charge */
        0xDB,0x40,  /* VCOMH */
        0xA4,0xA6,  /* resume RAM, normal color */
        0xAF        /* display on */
    };
    uint32_t i;
    if (hi2c == NULL) return HAL_ERROR;
    lcd_i2c = hi2c;
    if (HAL_I2C_IsDeviceReady(lcd_i2c, SSD1306_I2C_ADDR_HAL,
                              2U, SSD1306_TIMEOUT_MS) != HAL_OK) return HAL_ERROR;
    for (i = 0U; i < sizeof(init_seq); ++i) {
        if (command(init_seq[i]) != HAL_OK) return HAL_ERROR;
    }
    SSD1306_Clear();
    return SSD1306_Update();
}

void SSD1306_Clear(void)
{
    (void)memset(framebuffer, 0, sizeof(framebuffer));
}

void SSD1306_Print(uint8_t x_pixel, uint8_t page, const char *text)
{
    uint16_t pos;
    uint8_t col;
    const uint8_t *bits;
    if (text == NULL || page >= SSD1306_PAGES) return;
    pos = x_pixel;
    while (*text != '\0' && pos + 5U < SSD1306_W) {
        bits = glyph(*text++);
        for (col = 0U; col < 5U; ++col) {
            framebuffer[page * SSD1306_W + pos + col] = bits[col];
        }
        framebuffer[page * SSD1306_W + pos + 5U] = 0U;
        pos += 6U;
    }
}

HAL_StatusTypeDef SSD1306_Update(void)
{
    uint8_t page;
    uint8_t data[SSD1306_W + 1U];
    if (lcd_i2c == NULL) return HAL_ERROR;
    if (command(0x21U) != HAL_OK || command(0U) != HAL_OK ||
        command(127U) != HAL_OK || command(0x22U) != HAL_OK ||
        command(0U) != HAL_OK || command(7U) != HAL_OK) return HAL_ERROR;

    data[0] = 0x40U; /* data stream */
    for (page = 0U; page < SSD1306_PAGES; ++page) {
        (void)memcpy(&data[1], &framebuffer[page * SSD1306_W], SSD1306_W);
        if (HAL_I2C_Master_Transmit(lcd_i2c, SSD1306_I2C_ADDR_HAL,
                                     data, sizeof(data), SSD1306_TIMEOUT_MS)
            != HAL_OK) return HAL_ERROR;
    }
    return HAL_OK;
}
