/*
 * main.c
 *
 *  Created on: Aug 1, 2012
 *      Author: tuwuhs
 *      (c) 2012 Tuwuh Sarwoprasojo
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "lcd.h"
#include "adc.h"

#define PB_LEFT  (1<< 2)
#define PB_DOWN  (1<< 3)
#define PB_UP    (1<< 4)
#define PB_RIGHT (1<< 5)
#define PB_A     (1<< 6)
#define PB_B     (1<< 7)

#define STABLE_ERROR  (4)
#define STABLE_WAIT   (300)

static const PROGMEM unsigned char f_lock[] = {
	0x0,0xe,0x11,0x15,0x11,0xe,0x0,0x0,
	0x0,0xe,0x15,0x15,0x11,0xe,0x0,0x0,
	0x0,0xe,0x15,0x17,0x11,0xe,0x0,0x0,
	0x0,0xe,0x15,0x17,0x15,0xe,0x0,0x0,
	0x0,0xe,0x15,0x1f,0x15,0xe,0x0,0x0,
	0x0,0x1,0x3,0x16,0x1c,0x8,0x0,0x0
};

static const PROGMEM unsigned char f_ph[] = {
	0x0,0x0,0x1e,0x1b,0x1b,0x1e,0x18,0x18,
	0x1b,0x1b,0x1b,0x1f,0x1b,0x1b,0x1b,0x0
};

uint8_t task_pb_sample(void)
{
	static uint8_t pb_pressed = 0;
	static uint8_t pb_pin = 0;
	uint8_t pb_flag = 0;

	// Sample pushbutton pin
	pb_pin = PIND & 0xFC;

	// Update pressed and released state
	// Note: pb_flag is non-persistent,
	//       the event is assumed to be handled right after the flag is set
	pb_flag = pb_pressed & pb_pin;
	pb_pressed = ~pb_pin;

	return pb_flag;
}

void lcd_load_char(void)
{
	uint8_t i;
	/*
	 * load userdefined characters from program memory
	 * into LCD controller CG RAM location 0~5
	 */
	lcd_command(_BV(LCD_CGRAM) | (0));  /* set CG RAM start address 0 */
	for (i = 0; i < sizeof(f_lock); i++) {
		lcd_data(pgm_read_byte_near(&f_lock[i]));
	}
	lcd_command(_BV(LCD_CGRAM) | (6*8));  /* set CG RAM start address 0 */
	for (i = 0; i < sizeof(f_ph); i++) {
		lcd_data(pgm_read_byte_near(&f_ph[i]));
	}
	lcd_command(_BV(LCD_DDRAM));
}

int main(void)
{
	char lcd_string[20];
	int16_t curr_val = 0;
	int16_t centre_val = 0;

	uint16_t stable_count = 0;
	uint8_t stable = 0;

	uint8_t led_state = 0;
	uint8_t pb_event = 0;

	// Set PORTD for LEDs and pushbuttons
	// Turn-off LEDs, enable pull-up
	DDRD = (1<< 0) | (1<< 1);
	PORTD = 0xFF;

	// Initialize the LCD
	lcd_init(LCD_DISP_ON);
	lcd_puts("Hello World!");
	lcd_load_char();

	// Start ADC sampling
	adc_init();
	adc_start(2);
	sei();

	while (1) {
		curr_val = (int16_t) g_adc_val;

		// Print raw averaged value
		sprintf(lcd_string, "%4u - %7lu", curr_val, g_adc_avg);
		lcd_gotoxy(0, 0);
		lcd_puts(lcd_string);

		// Stability indicator
		if (abs(centre_val - curr_val) < STABLE_ERROR) {
			if (stable_count > STABLE_WAIT) {
				stable = 1;
				stable_count = 0;
				centre_val = curr_val;
			} else {
				stable_count++;
			}
		} else {
			stable = 0;
			stable_count = 0;
			centre_val = curr_val;
		}

		// Display pH
		sprintf(lcd_string, "\006\007 %5.2f   %5d",
				(double) (curr_val - 2113) / ((2113 - 1474) / (7.0 - 4.0)) + 7.0,
				centre_val);
		lcd_gotoxy(0, 1);
		lcd_puts(lcd_string);

		// Display stability indicator
		lcd_gotoxy(10, 1);
		if (stable) {
			lcd_putc(5);
		} else {
			switch (stable_count / (STABLE_WAIT / 5)) {
				case 0: lcd_putc(0); break;
				case 1:	lcd_putc(1); break;
				case 2:	lcd_putc(2); break;
				case 3:	lcd_putc(3); break;
				case 4:	lcd_putc(4); break;
				default: lcd_putc(' ');	break;
			}
		}

		// Sample pushbutton
		pb_event = task_pb_sample();

		// Handle pushbutton events
		if (pb_event & PB_A) {
			led_state ^= 0xFF;
		}
		if (pb_event & PB_LEFT) {
			lcd_command(LCD_MOVE_DISP_LEFT);
		}
		if (pb_event & PB_RIGHT) {
			lcd_command(LCD_MOVE_DISP_RIGHT);
		}
		if (pb_event & PB_UP) {
		}

		// Update LED state
		if (led_state) {
			PORTD |= (1<< 0);
		} else {
			PORTD &= ~(1<< 0);
		}

		_delay_ms(20);
	}

	return 0;
}
