/**
 * Copyright (c) 2017-2018, Łukasz Marcin Podkalicki <lpodkalicki@gmail.com>
 *
 * This is ATtiny13/25/45/85 library for 4-Digit LED Display based on TM1637 chip.
 *
 * Features:
 * - display raw segments
 * - display digits
 * - display colon
 * - display on/off
 * - brightness control
 *
 * References:
 * - library: https://github.com/lpodkalicki/attiny-tm1637-library
 * - documentation: https://github.com/lpodkalicki/attiny-tm1637-library/README.md
 * - TM1637 datasheet: https://github.com/lpodkalicki/attiny-tm1637-library/blob/master/docs/TM1637_V2.4_EN.pdf
 */

//#include <avr/io.h>
//#include <avr/pgmspace.h>
//#include <util/delay.h>
#include "tm1637.h"
#include "main.h"

#define	TM1637_DIO_HIGH()		HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_SET);
#define	TM1637_DIO_LOW()		HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_RESET);
#define	TM1637_DIO_OUTPUT()		HAL_GPIO_Init(TM1637_DIO_PORT, &GPIO_InitStruct_DIO_Output);
#define	TM1637_DIO_INPUT()		HAL_GPIO_Init(TM1637_DIO_PORT, &GPIO_InitStruct_DIO_Input);

#define	TM1637_DIO_READ() 		HAL_GPIO_ReadPin(TM1637_DIO_PORT,TM1637_DIO_PIN)

#define	TM1637_CLK_HIGH()		HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_SET);
#define	TM1637_CLK_LOW()		HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_RESET);

static void TM1637_send_config(const uint8_t enable, const uint8_t brightness);
static void TM1637_send_command(const uint8_t value);
static void TM1637_start(void);
static void TM1637_stop(void);
static uint8_t TM1637_write_byte(uint8_t value);

static uint8_t _config = TM1637_SET_DISPLAY_ON | TM1637_BRIGHTNESS_MAX;
static uint8_t _segments = 0xff;
GPIO_InitTypeDef GPIO_InitStruct_DIO_Input = {.Mode = GPIO_MODE_INPUT, .Pin=TM1637_DIO_PIN};
GPIO_InitTypeDef GPIO_InitStruct_DIO_Output = {.Mode = GPIO_MODE_OUTPUT_PP, .Pin=TM1637_DIO_PIN};
const uint8_t _digit2segments[] =
{
	0x3F, // 0
	0x06, // 1
	0x5B, // 2
	0x4F, // 3
	0x66, // 4
	0x6D, // 5
	0x7D, // 6
	0x07, // 7
	0x7F, // 8
	0x6F,  // 9
    0x77,		/* A */
    0x7c,		/* B */
    0x39,		/* C */
    0x5e,		/* D */
    0x79,		/* E */
    0x71,		/* F */
};

void
TM1637_init(const uint8_t enable, const uint8_t brightness)
{

//	DDRB |= (_BV(TM1637_DIO_PIN)|_BV(TM1637_CLK_PIN));
//	PORTB &= ~(_BV(TM1637_DIO_PIN)|_BV(TM1637_CLK_PIN));
	TM1637_send_config(enable, brightness);
}

void
TM1637_enable(const uint8_t value)
{

	TM1637_send_config(value, _config & TM1637_BRIGHTNESS_MAX);
}

void
TM1637_set_brightness(const uint8_t value)
{

	TM1637_send_config(_config & TM1637_SET_DISPLAY_ON,
		value & TM1637_BRIGHTNESS_MAX);
}

void
TM1637_display_segments(const uint8_t position, const uint8_t segments)
{

	TM1637_send_command(TM1637_CMD_SET_DATA | TM1637_SET_DATA_F_ADDR);
	TM1637_start();
	TM1637_write_byte(TM1637_CMD_SET_ADDR | (position & (TM1637_POSITION_MAX - 1)));
	TM1637_write_byte(segments);
	TM1637_stop();
}

void
TM1637_display_digit(const uint8_t position, const uint8_t digit)
{
	uint8_t segments = ( _digit2segments[digit & 0x0f]);

	if (position == 0x01) {
		segments = segments | (_segments & 0x80);
		_segments = segments;
	}

	TM1637_display_segments(position, segments);
}

void
TM1637_display_colon(const uint8_t value)
{

	if (value) {
		_segments |= 0x80;
	} else {
		_segments &= ~0x80;
	}
	TM1637_display_segments(0x01, _segments);
}

void
TM1637_clear(void)
{
	uint8_t i;

	for (i = 0; i < TM1637_POSITION_MAX; ++i) {
		TM1637_display_segments(i, 0x00);
	}
}

void
TM1637_send_config(const uint8_t enable, const uint8_t brightness)
{

	_config = (enable ? TM1637_SET_DISPLAY_ON : TM1637_SET_DISPLAY_OFF) |
		(brightness > TM1637_BRIGHTNESS_MAX ? TM1637_BRIGHTNESS_MAX : brightness);

	TM1637_send_command(TM1637_CMD_SET_DSIPLAY | _config);
}

void
TM1637_send_command(const uint8_t value)
{

	TM1637_start();
	TM1637_write_byte(value);
	TM1637_stop();
}

void
TM1637_start(void)
{
	TM1637_DIO_OUTPUT()
	TM1637_DIO_HIGH();
	TM1637_CLK_HIGH();
	delay_us(TM1637_DELAY_US);
	TM1637_DIO_LOW();
}

void
TM1637_stop(void)
{

	TM1637_CLK_LOW();
	delay_us(TM1637_DELAY_US);

	TM1637_DIO_LOW();
	delay_us(TM1637_DELAY_US);

	TM1637_CLK_HIGH();
	delay_us(TM1637_DELAY_US);

	TM1637_DIO_HIGH();
}

uint8_t
TM1637_write_byte(uint8_t value)
{
	uint8_t i, ack;
	TM1637_DIO_OUTPUT()
	for (i = 0; i < 8; ++i, value >>= 1) {
		TM1637_CLK_LOW();
		delay_us(TM1637_DELAY_US);

		if (value & 0x01) {
			TM1637_DIO_HIGH();
		} else {
			TM1637_DIO_LOW();
		}

		TM1637_CLK_HIGH();
		delay_us(TM1637_DELAY_US);
	}

	TM1637_CLK_LOW();
	TM1637_DIO_INPUT();
	TM1637_DIO_HIGH();
	delay_us(TM1637_DELAY_US);

	ack = TM1637_DIO_READ();
	if (ack) {
		TM1637_DIO_OUTPUT();
		TM1637_DIO_LOW();
	}
	delay_us(TM1637_DELAY_US);

	TM1637_CLK_HIGH();
	delay_us(TM1637_DELAY_US);

	TM1637_CLK_LOW();
	delay_us(TM1637_DELAY_US);

	TM1637_DIO_OUTPUT();

	return ack;
}

/*!
 * \brief Read byte from TM1637.
 * \return 8-bit value.
 */
uint8_t TM1637_read_byte()
{
    uint8_t retval = 0;

    // Prepare DIO to read data
    TM1637_DIO_HIGH();
    TM1637_DIO_INPUT();
    delay_us(TM1637_DELAY_US);

    // Data is shifted out by the TM1637 on the CLK falling edge
    for (uint8_t bit = 0; bit < 8; bit++) {
        TM1637_CLK_HIGH();
        delay_us(TM1637_DELAY_US);

        // Read next bit
        retval <<= 1;
        if (TM1637_DIO_READ()) {
            retval |= 0x01;
        }

        TM1637_CLK_LOW();
        delay_us(TM1637_DELAY_US);
    }
    // Return DIO to output mode
    TM1637_DIO_OUTPUT();
    delay_us(TM1637_DELAY_US);

    // Dummy ACK
    TM1637_DIO_LOW();
    delay_us(TM1637_DELAY_US);

    TM1637_CLK_HIGH();
    delay_us(TM1637_DELAY_US);
    TM1637_CLK_LOW();
    delay_us(TM1637_DELAY_US);

    TM1637_DIO_HIGH();
    delay_us(TM1637_DELAY_US);

    return retval;
}

// -------------------------------------------------------------------------------------------------

uint8_t TM1637_getKeys()
{
    uint8_t keyCode;

    TM1637_start();
    TM1637_write_byte(TM1637_CMD_SET_DATA | TM1637_SET_DATA_READ);
    TM1637_start();
    keyCode = TM1637_read_byte();
    TM1637_stop();

    // Check if key is down (at least one bit is zero)
    if (keyCode != 0xFF) {
        // Invert keyCode:
        //    Bit |  7  6  5  4  3  2  1  0
        //  ------+-------------------------
        //   From | S0 S1 S2 K1 K2 1  1  1
        //     To | S0 S1 S2 K1 K2 0  0  0
        keyCode = ~keyCode;

        // Shift bits to:
        //    Bit | 7  6  5  4  3  2  1  0
        //  ------+------------------------
        //     To | 0  0  0  0  K2 S2 S1 S0
//        keyCode = (uint8_t)((keyCode & 0x80) >> 7 |
//                            (keyCode & 0x40) >> 5 |
//                            (keyCode & 0x20) >> 3 |
//                            (keyCode & 0x08));
        switch(keyCode){
        case 0x08:
        	keyCode = KEY_UP; // SW2
        	break;
        case 0x10:
        	keyCode = KEY_DOWN; // SW4
        	break;
        case 0x90:
        	keyCode = KEY_START; // SW3
        	break;
        case 0x88:
        	keyCode = KEY_STOP; // SW1
        	break;
        default:
        	keyCode = 0; //Unknown
        	break;
        }
    }
    else keyCode = 0;

    return keyCode;
}
