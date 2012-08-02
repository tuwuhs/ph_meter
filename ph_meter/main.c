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

enum {
	MODE_NORMAL,
	MODE_CALIBRATE
} g_operating_mode;

// Calibration data
double g_ph_const, g_ph_slope;

//
uint8_t g_pb_flag = 0;
int16_t g_curr_val = 0;
uint16_t g_stable_count = 0;
uint8_t g_stable = 0;

//
char g_lcd_string[20];

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
	lcd_command(_BV(LCD_CGRAM) | (6*8));  /* set CG RAM start address 6 */
	for (i = 0; i < sizeof(f_ph); i++) {
		lcd_data(pgm_read_byte_near(&f_ph[i]));
	}
	lcd_command(_BV(LCD_DDRAM));
}

void task_pb_sample(void)
{
	static uint8_t pb_pressed = 0;
	static uint8_t pb_pin = 0;

	// Sample pushbutton pin
	pb_pin = PIND & 0xFC;

	// Update pressed and released state
	// Note: pb_flag is non-persistent,
	//       the event is assumed to be handled right after the flag is set
	g_pb_flag = pb_pressed & pb_pin;
	pb_pressed = ~pb_pin;
}

void task_stability_indicator(void)
{
	static int16_t centre_val = 0;

	if (abs(centre_val - g_curr_val) < STABLE_ERROR) {
		if (g_stable_count > STABLE_WAIT) {
			g_stable = 1;
			g_stable_count = 0;
			centre_val = g_curr_val;
		} else {
			g_stable_count++;
		}
	} else {
		g_stable = 0;
		g_stable_count = 0;
		centre_val = g_curr_val;
	}
}

void ui_ph_calibrate(void)
{
	static enum {
		PH7_START, PH7_PROBE, PH7_SAVE,
		PH4_START, PH4_PROBE, PH4_SAVE,
		CALCULATE, VERIFY, SAVE_CAL
	} state = PH7_START;

	static int16_t raw_ph7, raw_ph4;
	static double ph_slope, ph_const;

	double ph;

	// Print raw averaged value
	sprintf(g_lcd_string, "RAW %4u", g_curr_val);
	lcd_gotoxy(0, 1);
	lcd_puts(g_lcd_string);

	// Display stability indicator
	lcd_gotoxy(10, 1);
	if (g_stable) {
		lcd_putc(5);
	} else {
		switch (g_stable_count / (STABLE_WAIT / 5)) {
			case 0: lcd_putc(0); break;
			case 1:	lcd_putc(1); break;
			case 2:	lcd_putc(2); break;
			case 3:	lcd_putc(3); break;
			case 4:	lcd_putc(4); break;
			default: lcd_putc(' ');	break;
		}
	}

	lcd_gotoxy(0, 0);
	switch (state) {
	case PH7_START:
		lcd_puts("Probe pH 7.0?   ");
		g_stable_count = 0;
		g_stable = 0;
		if (g_pb_flag & PB_A) state = PH7_PROBE;
		break;
	case PH7_PROBE:
		lcd_puts("Probing ph 7.0..");
		if (g_stable) state = PH7_SAVE;
		break;
	case PH7_SAVE:
		raw_ph7 = g_curr_val;
		state = PH4_START;
		break;
	case PH4_START:
		lcd_puts("Probe pH 4.0?   ");
		g_stable_count = 0;
		g_stable = 0;
		if (g_pb_flag & PB_A) state = PH4_PROBE;
		break;
	case PH4_PROBE:
		lcd_puts("Probing ph 4.0..");
		if (g_stable) state = PH4_SAVE;
		break;
	case PH4_SAVE:
		raw_ph4 = g_curr_val;
		state = CALCULATE;
		break;
	case CALCULATE:
		ph_slope = (4.0 - 7.0) / (raw_ph4 - raw_ph7);
		ph_const = 7.0 - raw_ph7 * ph_slope;
		state = VERIFY;
		break;
	case VERIFY:
		ph = ph_const + ph_slope * g_curr_val;
		sprintf(g_lcd_string, "\06\07 %5.2f   OK?  ", ph);
		lcd_puts(g_lcd_string);
		if (g_pb_flag & PB_A) state = SAVE_CAL;
		break;
	case SAVE_CAL:
		g_ph_const = ph_const;
		g_ph_slope = ph_slope;
		lcd_clrscr();
		state = PH7_START;
		g_operating_mode = MODE_NORMAL;
		break;
	}

	// Cancel button
	if (g_pb_flag & PB_B) {
		lcd_clrscr();
		state = PH7_START;
		g_operating_mode = MODE_NORMAL;
	}
}

void ui_normal(void)
{
	double ph;

	// Display pH
	ph = g_ph_const + g_ph_slope * g_curr_val;
	sprintf(g_lcd_string, "\06\07 %5.2f", ph);
	lcd_gotoxy(0, 1);
	lcd_puts(g_lcd_string);

	// Display stability indicator
	lcd_gotoxy(10, 1);
	if (g_stable) {
		lcd_putc(5);
	} else {
		switch (g_stable_count / (STABLE_WAIT / 5)) {
			case 0: lcd_putc(0); break;
			case 1:	lcd_putc(1); break;
			case 2:	lcd_putc(2); break;
			case 3:	lcd_putc(3); break;
			case 4:	lcd_putc(4); break;
			default: lcd_putc(' ');	break;
		}
	}

	// OK button for calibration
	if (g_pb_flag & PB_A) {
		g_operating_mode = MODE_CALIBRATE;
	}
}

int main(void)
{
	// Set PORTD for LEDs and pushbuttons
	// Turn-off LEDs, enable pull-up
	DDRD = (1<< 0) | (1<< 1);
	PORTD = 0xFF;

	// Initialize the LCD
	lcd_init(LCD_DISP_ON);
	lcd_load_char();

	// Start ADC sampling
	adc_init();
	adc_start(2);
	sei();

	// Initialize default calibration value
	g_ph_slope = 0.0;
	g_ph_const = 0.0;

	while (1) {
		// -------------------------------------
		// ------------ BASE SYSTEM ------------
		// -------------------------------------
		// The base system outputs the following global vars:
		// - g_curr_val: current ADC reading
		// - g_stable: stability indicator
		// - g_stable_count: stability progress counter
		// - g_pb_flag: pushbutton event
		// The tasks are allowed to access global variables directly

		// Get (averaged oversampled) ADC reading
		g_curr_val = (int16_t) g_adc_val;

		// Stability indicator
		task_stability_indicator();

		// Sample pushbutton
		task_pb_sample();

		// ------------------------------------
		// ---------- USER INTERFACE ----------
		// ------------------------------------
		// The ui_...() functions are responsible to:
		// - Display to LCD
		// - Handle pushbutton events
		// - Update LED if any
		// - Determine next operating mode

		switch (g_operating_mode) {
		case MODE_NORMAL:
			ui_normal();
			break;
		case MODE_CALIBRATE:
			ui_ph_calibrate();
			break;
		default:
			g_operating_mode = MODE_NORMAL;
			break;
		}

		_delay_ms(20);
	}

	return 0;
}
