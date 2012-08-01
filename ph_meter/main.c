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
#include <util/delay.h>

#include "lcd.h"
#include "adc.h"

#define PB_LEFT  (1<< 2)
#define PB_DOWN  (1<< 3)
#define PB_UP    (1<< 4)
#define PB_RIGHT (1<< 5)
#define PB_A     (1<< 6)
#define PB_B     (1<< 7)

#define STABLE_ERROR  (2)
#define STABLE_WAIT   (200)

int main(void)
{
	char lcd_string[20];
	int16_t curr_val = 0;
	int16_t centre_val = 0;

	uint16_t stable_count = 0;
	uint8_t stable = 0;

	uint8_t led_state = 0;
	uint8_t pb_pressed = 0;
	uint8_t pb_released = 0;
	uint8_t pb_pin = 0;

	// Set PORTD for LEDs and pushbuttons
	// Turn-off LEDs, enable pull-up
	DDRD = (1<< 0) | (1<< 1);
	PORTD = 0xFF;

	// Initialize the LCD
	lcd_init(LCD_DISP_ON);
	lcd_puts("Hello World!");

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

		lcd_gotoxy(15, 0);
		if (stable) {
			lcd_putc('S');
		} else {
			lcd_putc('-');
		}

		// Display pH
		sprintf(lcd_string, "pH %2.2f    %5d",
				(double) (curr_val - 2113) / ((2113.0 - 1474.0) / (7.0 - 4.0)) + 7.0,
				centre_val);
		lcd_gotoxy(0, 1);
		lcd_puts(lcd_string);


		// Sample pushbutton pin
		pb_pin = PIND & 0xFC;

		// Update pressed and released state
		// Note: b_released must be cleared manually after the event is handled
		pb_released = pb_pressed & pb_pin;
		pb_pressed = ~pb_pin;

		// Handle pushbutton events
		if (pb_released & PB_A) {
			pb_released &= ~PB_A;
			led_state ^= 0xFF;
		}

		// Update LED state
		if (led_state) {
			PORTD |= (1<< 0);
		} else {
			PORTD &= ~(1<< 0);
		}

		_delay_ms(20);
	}

	while (1) {

		if (pb_released & PB_LEFT) {
			pb_released &= ~PB_LEFT;
			sprintf(lcd_string, "Left            ");
			lcd_gotoxy(0, 0);
			lcd_puts(lcd_string);
		}

		if (pb_released & PB_UP) {
			pb_released &= ~PB_UP;
			sprintf(lcd_string, "Up              ");
			lcd_gotoxy(0, 0);
			lcd_puts(lcd_string);
		}

		if (pb_released & PB_DOWN) {
			pb_released &= ~PB_DOWN;
			sprintf(lcd_string, "Down            ");
			lcd_gotoxy(0, 0);
			lcd_puts(lcd_string);
		}

		if (pb_released & PB_RIGHT) {
			pb_released &= ~PB_RIGHT;
			sprintf(lcd_string, "Right           ");
			lcd_gotoxy(0, 0);
			lcd_puts(lcd_string);
		}


	}

	return 0;
}
