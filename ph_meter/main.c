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

int main(void)
{
	char lcd_string[20];
	int16_t curr_val = 0;
	int16_t last_val = 0;
	uint8_t stable_count = 0;

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
		last_val = curr_val;
		curr_val = (int16_t) g_adc_val;

		if (((last_val - curr_val) > -5) &&
			((last_val - curr_val) < 5)) {
			if (stable_count > 100) {
				sprintf(lcd_string, "Yeah  ");
			} else {
				stable_count++;
				sprintf(lcd_string, "Unyeah");
			}
		} else {
			stable_count = 0;
			sprintf(lcd_string, "Unyeah");
		}
		lcd_gotoxy(0, 1);
		lcd_puts(lcd_string);

		sprintf(lcd_string, "%4u - %7lu", curr_val, g_adc_avg);
		lcd_gotoxy(0, 0);
		lcd_puts(lcd_string);

//		sprintf(lcd_string, "pH %2.2f   ", (double) (curr_val - 537) / ((537.0 - 370.0) / (7.0 - 4.0)) + 7.0);
//		lcd_gotoxy(0, 1);
//		lcd_puts(lcd_string);

		/* Oversampling then EMA */
		// sprintf(lcd_string, "%4lu - %6lu", g_adc_ovs_avg_val, g_adc_ovs_avg);
		// lcd_gotoxy(0, 1);
		// lcd_puts(lcd_string);

		_delay_ms(20);
	}

	while (1) {
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


		if (led_state) {
			PORTD |= (1<< 0);
		} else {
			PORTD &= ~(1<< 0);
		}
	}

	return 0;
}
