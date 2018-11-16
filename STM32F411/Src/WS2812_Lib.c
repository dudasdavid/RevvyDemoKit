/**
 * WS2812 Neopixel LED driver for STM32
 * Based upon the WS2812 Library for STM32F4 from Uwe Becker, http://mikrocontroller.bplaced.net/wordpress/?page_id=3665
 *
 * @Author: Nicolas Dammin, 2016
 * @Author: Dávid Dudás, 2018
 * @Author: Dániel Buga, 2018
 */
#include "WS2812_Lib.h"
#include "stm32f4xx.h"
#include <string.h>

/* Raw color values for the individual LEDs */
static WS2812_RGB_t WS2812_LED_BUF_CH1[WS2812_NUM_LEDS_CH1];

#ifdef WS2812_USE_TIMER
extern TIM_HandleTypeDef htim4;

/* Buffer that contains duty cycle values for PWM generation using timer */
static uint16_t WS2812_TIM_BUF[WS2812_OUT_BITS];
/* Flag to indicate that PWM generation is in progress */
static uint8_t dma_ready = 1;
#endif

#ifdef WS2812_USE_SPI
extern SPI_HandleTypeDef hspi5;

/* Buffer that contains bit patterns for PWM generation using SPI peripherial
 * We are using 3 bits per LED bit, so 3/8 bytes, rounded up */
static uint8_t WS2812_SPI_BUF[(WS2812_OUT_BITS * 3 + 7) / 8];
/* Flag to indicate that PWM generation is in progress */
static uint8_t spi_dma_ready = 1;
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)           (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef IS_BIT_SET
#define IS_BIT_SET(var, bit)    (((var) & (1 << (7 - (bit)))) != 0u)
#endif

#ifdef WS2812_USE_TIMER
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	dma_ready = 1;
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        //while(1);
    }
}

void timer_set_byte(uint32_t pos, uint8_t color)
{
    for (uint8_t i = 0u; i < 8u; i++)
    {
        WS2812_TIM_BUF[pos + i] = IS_BIT_SET(color, i) ? WS2812_HI_TIME : WS2812_LO_TIME;
    }
}

/**
 * Writes data of a pixel to the output buffer used by the timer-based PWM
 *
 * One pixel requires 24 data bits, 8 bits per color. WS2812 uses pulse width
 * encoding to allow communicating over a single wire. The encoding requires
 * 2/3 duty cycle for a high bit and 1/3 for a low bit, which we are generating
 * using a 16bit timer, so we need 16 bits of memory for every single "LED-bit".
 *
 * @param in pixelPosition  The index of the pixel to write to the output buffer
 */
static void timer_set_pixel(uint32_t pixelPosition)
{
    uint32_t startingBytePosition = pixelPosition * 24u;

    timer_set_byte(startingBytePosition, WS2812_LED_BUF_CH1[pixelPosition].green);
    timer_set_byte(startingBytePosition + 8, WS2812_LED_BUF_CH1[pixelPosition].red);
    timer_set_byte(startingBytePosition + 16, WS2812_LED_BUF_CH1[pixelPosition].blue);
}

/**
 * Internal function, calculates the HI or LO values for the 800 kHz WS2812 signal and puts them into a buffer for the Timer-DMA
 *
 */
static void calcBuf_Timer(void)
{
    /* set timings for all LEDs */
    for (uint32_t n = 0u; n < WS2812_NUM_LEDS_CH1; n++)
    {
        timer_set_pixel(n);
    }

    /* reset pulse after all LEDs have been updated */
    memset(&WS2812_TIM_BUF[WS2812_NUM_LEDS_CH1], 0, WS2812_NUM_RESET_BITS);
}
#endif /* WS2812_USE_TIMER */

#ifdef WS2812_USE_SPI
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_dma_ready = 1;
}

/**
 * Sets a single LED-bit value in the output buffer
 *
 * @param bitPosition   The position of the bit to set, 0 indexed
 * @param bitValue      The desired bit value
 */
void spi_set_bit(uint32_t bitPosition, uint8_t bitValue)
{
    /* precomputed bit patterns for the '1' bit value */
    static const uint8_t highBitPatternLUT[] =
    {
        0b11000000,
        0b00011000,
        0b00000011,
        0b01100000,
        0b00001100,
        0b00000001,
        0b00110000,
        0b00000011
    };

    /* which byte we need to write to */
    uint32_t byte = (bitPosition * 3) / 8;

    uint8_t index = bitPosition % 8u;
    if (bitValue == 0)
    {
        /* we can set the bit pattern in one go */
        WS2812_SPI_BUF[byte] |= (1 << (7u - index));
    }
    else
    {
        /* we can set the bit pattern in one go */
        WS2812_SPI_BUF[byte] |= highBitPatternLUT[index];
        if (index == 5u)
        {
            /* we need to write to two bytes
             * let's assume that bits are written serially so no need to OR */
            WS2812_SPI_BUF[byte + 1] = 0x80u;
        }
    }
}

/**
 * Sets a single LED-byte value in the output buffer
 *
 * @param bytePosition   The position of the byte to set, 0 indexed
 * @param byteValue      The desired byte value
 */
void spi_set_byte(uint32_t bytePosition, uint8_t byteValue)
{
    uint32_t startingBitPosition = bytePosition * 8u;
    for (uint8_t i = 0; i < 8; i++)
    {
        spi_set_bit(startingBitPosition + i, IS_BIT_SET(byteValue, i));
    }
}

/**
 * Writes data of a pixel to the output buffer used by the SPI-based PWM
 *
 * One pixel requires 24 data bits, 8 bits per color. WS2812 uses pulse width
 * encoding to allow communicating over a single wire. The encoding requires
 * 2/3 duty cycle for a high bit and 1/3 for a low bit, so we need to shift 3
 * bits out for every single "LED-bit".
 *
 * @param in pixelPosition  The index of the pixel to write to the output buffer
 */
static void spi_set_pixel(uint32_t pixelPosition)
{
    uint32_t startingBytePosition = pixelPosition * 3u;

    spi_set_byte(startingBytePosition, WS2812_LED_BUF_CH1[pixelPosition].green);
    spi_set_byte(startingBytePosition + 1u, WS2812_LED_BUF_CH1[pixelPosition].red);
    spi_set_byte(startingBytePosition + 2u, WS2812_LED_BUF_CH1[pixelPosition].blue);
}

static void calcBuf_SPI( void )
{
    /* Filling the SPI buffer uses logical or, so we need to clear it first */
    memset(WS2812_SPI_BUF, 0, sizeof(WS2812_SPI_BUF));

    /* set timings for all LEDs */
    for (uint32_t n = 0u; n < WS2812_NUM_LEDS_CH1; n++)
    {
        spi_set_pixel(n);
    }
}
#endif /* WS2812_USE_SPI */

void WS2812_Refresh(void)
{
#ifdef WS2812_USE_TIMER
    while (!dma_ready);
    dma_ready = 0;

    calcBuf_Timer();
    HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, (uint32_t *) WS2812_TIM_BUF, ARRAY_SIZE(WS2812_TIM_BUF));
#endif

#ifdef WS2812_USE_SPI
    while (!spi_dma_ready);
    spi_dma_ready = 0;

    calcBuf_SPI();
    HAL_SPI_Transmit_DMA(&hspi5, WS2812_SPI_BUF, ARRAY_SIZE(WS2812_SPI_BUF));
#endif
}

/**
 * Set all LEDs to 0 (off) and update
 */
void WS2812_Clear(void)
{
    WS2812_All_RGB((WS2812_RGB_t) { 0, 0, 0 }, 1);
}

/**
 * Convert HSV-Value to RGB Value for WS2812 LEDs
 * (from www.ulrichradig.de)
 */
void WS2812_HSV2RGB(WS2812_HSV_t hsv_col, WS2812_RGB_t *rgb_col)
{
    /* calculate base color from hue */
    hsv_col.h  = hsv_col.h % 360;
    uint16_t h = hsv_col.h % 60;

    uint16_t c = (425 * h) / 100;
    if (hsv_col.h <= 60)
    {
        rgb_col->red = 255;
        rgb_col->green = c;
        rgb_col->blue = 0;
    }
    else if (hsv_col.h <= 120)
    {
        rgb_col->red = 255 - c;
        rgb_col->green = 255;
        rgb_col->blue = 0;
    }
    else if (hsv_col.h <= 180)
    {
        rgb_col->red = 0;
        rgb_col->green = 255;
        rgb_col->blue = c;
    }
    else if (hsv_col.h <= 240)
    {
        rgb_col->red = 0;
        rgb_col->green = 255 - c;
        rgb_col->blue = 255;
    }
    else if (hsv_col.h <= 300)
    {
        rgb_col->red = c;
        rgb_col->green = 0;
        rgb_col->blue = 255;
    }
    else
    {
        rgb_col->red = 255;
        rgb_col->green = 0;
        rgb_col->blue = 255 - c;
    }

    /* adjust saturation */
    uint16_t s = (hsv_col.s > 100) ? 0 : (100 - hsv_col.s);
    rgb_col->red   = rgb_col->red   + ((255 - rgb_col->red  ) * s) / 100;
    rgb_col->green = rgb_col->green + ((255 - rgb_col->green) * s) / 100;
    rgb_col->blue  = rgb_col->blue  + ((255 - rgb_col->blue ) * s) / 100;

    /* adjust lightness */
    uint16_t v = (hsv_col.v > 100) ? 100 : hsv_col.v;
    rgb_col->red   = (rgb_col->red   * v) / 100;
    rgb_col->green = (rgb_col->green * v) / 100;
    rgb_col->blue  = (rgb_col->blue  * v) / 100;
}
 
/**
 * Set one LED (R, G, B values). If refresh == 1, update LEDs, otherwise just update buffer (if several function calls are to be done before refresh)
 */
void WS2812_One_RGB(uint32_t nr, WS2812_RGB_t rgb_col, uint8_t refresh)
{
    if (nr < WS2812_NUM_LEDS_CH1)
    {
        WS2812_LED_BUF_CH1[nr] = rgb_col;

        if (refresh == 1)
        {
            WS2812_Refresh();
        }
    }
}

/**
 * Set all LEDs (R, G, B values). If refresh == 1, update LEDs, otherwise just update buffer (if several function calls are to be done before refresh)
 */
void WS2812_All_RGB(WS2812_RGB_t rgb_col, uint8_t refresh)
{
    for (uint32_t n = 0u; n < WS2812_NUM_LEDS_CH1; n++)
    {
        WS2812_LED_BUF_CH1[n] = rgb_col;
    }
    if (refresh == 1)
    {
        WS2812_Refresh();
    }
}

/**
 * Set one LED (H, S, V values). If refresh == 1, update LEDs, otherwise just update buffer (if several function calls are to be done before refresh)
 */
void WS2812_One_HSV(uint32_t nr, WS2812_HSV_t hsv_col, uint8_t refresh)
{
    if (nr < WS2812_NUM_LEDS_CH1)
    {
        WS2812_HSV2RGB(hsv_col, &WS2812_LED_BUF_CH1[nr]);

        if (refresh == 1)
        {
            WS2812_Refresh();
        }
    }
}

/**
 * Set all LEDs (H, S, V values). If refresh == 1, update LEDs, otherwise just update buffer (if several function calls are to be done before refresh)
 */
void WS2812_All_HSV(WS2812_HSV_t hsv_col, uint8_t refresh)
{
    WS2812_RGB_t rgb_col;
    WS2812_HSV2RGB(hsv_col, &rgb_col);
    WS2812_All_RGB(rgb_col, refresh);
}

/**
 * Shift all LED values one to the right.
 *
 * @param colorToInsert The new LED color to shift in
 * @param refresh       Start sending the buffer
 */
static void WS2812_Internal_Shift_Right_Using(WS2812_RGB_t colorToInsert, uint8_t refresh)
{
#if (WS2812_NUM_LEDS_CH1 > 1)
    for (uint32_t n = WS2812_NUM_LEDS_CH1 - 1; n > 0; n--)
    {
        WS2812_LED_BUF_CH1[n] = WS2812_LED_BUF_CH1[n - 1];
    }
    WS2812_LED_BUF_CH1[0] = colorToInsert;

    if (refresh == 1)
    {
        WS2812_Refresh();
    }
#endif
}

/**
 * Shift all LED values one to the left.
 *
 * @param colorToInsert The new LED color to shift in
 * @param refresh       Start sending the buffer
 */
static void WS2812_Internal_Shift_Left_Using(WS2812_RGB_t colorToInsert, uint8_t refresh)
{
#if (WS2812_NUM_LEDS_CH1 > 1)
    for (uint32_t n = 1; n < WS2812_NUM_LEDS_CH1; n++)
    {
        WS2812_LED_BUF_CH1[n - 1] = WS2812_LED_BUF_CH1[n];
    }
    WS2812_LED_BUF_CH1[WS2812_NUM_LEDS_CH1 - 1] = colorToInsert;

    if (refresh == 1)
    {
        WS2812_Refresh();
    }
#endif
}

/**
 * Shift all LED values one to the left. Last one will be turned off
 */
void WS2812_Shift_Left(uint8_t refresh)
{
#if (WS2812_NUM_LEDS_CH1 > 1)
    WS2812_Internal_Shift_Left_Using((WS2812_RGB_t) { 0, 0, 0 }, refresh);
#endif
}


/**
 * Shift all LED values one to the right. First one will be turned off
 */
void WS2812_Shift_Right(uint8_t refresh)
{
#if (WS2812_NUM_LEDS_CH1 > 1)
    WS2812_Internal_Shift_Right_Using((WS2812_RGB_t) { 0, 0, 0 }, refresh);
#endif
}

/**
 * Shift all LED values one to the left. Last LED value will be the previous first value
 */
void WS2812_Rotate_Left(uint8_t refresh)
{
#if (WS2812_NUM_LEDS_CH1 > 1)
    WS2812_Internal_Shift_Left_Using(WS2812_LED_BUF_CH1[0], refresh);
#endif
}

/**
 * Shift all LED values one to the right. Last LED value will be the previous first value
 */
void WS2812_Rotate_Right(uint8_t refresh)
{
#if (WS2812_NUM_LEDS_CH1 > 1)
    WS2812_Internal_Shift_Right_Using(WS2812_LED_BUF_CH1[WS2812_NUM_LEDS_CH1 - 1], refresh);
#endif
}

