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

#ifndef	_ATTINY_TM1637_H_
#define	_ATTINY_TM1637_H_

#include <stdint.h>
#include "stm32f0xx_hal.h"

// Main Settings
#define	TM1637_DIO_PIN			GPIO_PIN_4
#define	TM1637_DIO_PORT			GPIOB
#define	TM1637_CLK_PIN			GPIO_PIN_15
#define	TM1637_CLK_PORT			GPIOA
#define	TM1637_DELAY_US			(5)
#define	TM1637_BRIGHTNESS_MAX		(7)
#define	TM1637_POSITION_MAX		(4)

// TM1637 commands
#define	TM1637_CMD_SET_DATA		    0x40   //!< Display data command
#define	TM1637_CMD_SET_ADDR		    0xC0   //!< Display address command
#define	TM1637_CMD_SET_DSIPLAY		0x80   //!< Display control command


// TM1637 data settings (use bitwise OR to contruct complete command)
#define	TM1637_SET_DATA_WRITE		0x00 // write data to the display register
#define	TM1637_SET_DATA_READ		0x02 // read the key scan data
#define	TM1637_SET_DATA_A_ADDR		0x00 // automatic address increment
#define	TM1637_SET_DATA_F_ADDR		0x04 // fixed address
#define	TM1637_SET_DATA_M_NORM		0x00 // normal mode
#define	TM1637_SET_DATA_M_TEST		0x10 // test mode

// TM1637 display control command set (use bitwise OR to consruct complete command)
#define	TM1637_SET_DISPLAY_OFF		0x00 // off
#define	TM1637_SET_DISPLAY_ON		0x08 // on

// Control command bits
#define TM1637_CTRL_PULSE_1_16          0x00 //!< Pulse width 1/16
#define TM1637_CTRL_PULSE_2_16          0x01 //!< Pulse width 2/16
#define TM1637_CTRL_PULSE_4_16          0x02 //!< Pulse width 4/16
#define TM1637_CTRL_PULSE_10_16         0x03 //!< Pulse width 10/16
#define TM1637_CTRL_PULSE_11_16         0x04 //!< Pulse width 11/16
#define TM1637_CTRL_PULSE_12_16         0x05 //!< Pulse width 12/16
#define TM1637_CTRL_PULSE_13_16         0x06 //!< Pulse width 13/16
#define TM1637_CTRL_PULSE_14_16         0x07 //!< Pulse width 14/16


#define KEY_UP    2 // SW2
#define KEY_DOWN  4 // SW4
#define KEY_START 3 // SW3
#define KEY_STOP  1 // SW1



/**
 * Initialize TM1637 display driver.
 * Clock pin (TM1637_CLK_PIN) and data pin (TM1637_DIO_PIN)
 * are defined at the top of this file.
 */
void TM1637_init(const uint8_t enable, const uint8_t brightness);

/**
 * Turn display on/off.
 * value: 1 - on, 0 - off
 */
void TM1637_enable(const uint8_t value);

/**
 * Set display brightness.
 * Min value: 0
 * Max value: 7
 */
void TM1637_set_brightness(const uint8_t value);

/**
 * Display raw segments at position (0x00..0x03)
 *
 *      bits:
 *        -- 0 --
 *       |       |
 *       5       1
 *       |       |
 *        -- 6 --
 *       |       |
 *       4       2
 *       |       |
 *        -- 3 --
 *
 * Example segment configurations:
 * - for character 'H', segments=0b01110110
 * - for character '-', segments=0b01000000
 * - etc.
 */
void TM1637_display_segments(const uint8_t position, const uint8_t segments);

/**
 * Display digit ('0'..'9') at position (0x00..0x03)
 */
void TM1637_display_digit(const uint8_t position, const uint8_t digit);

/**
 * Display colon on/off.
 * value: 1 - on, 0 - off
 */
void TM1637_display_colon(const uint8_t value);

/**
 * Clear all segments (including colon).
 */
void TM1637_clear(void);

/**
 * Read  keypad
 */
uint8_t TM1637_getKeys();

#endif	/* !_ATTINY_TM1637_H_ */
