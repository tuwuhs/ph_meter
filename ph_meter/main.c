/*
 * main.c
 *
 *  Created on: Aug 1, 2012
 *      Author: tuwuhs
 */

#include <stdint.h>

#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	uint8_t led_state = 0;
	uint8_t pb_pressed = 0;
	uint8_t pb_released = 0;
	uint8_t pb_pin = 0;

	// Set PORTD for LEDs and pushbuttons
	// Turn-off LEDs, enable pull-up
	DDRD = (1<< 0) | (1<< 1);
	PORTD = 0xFF;

	while (1) {
		// Sample pushbutton pin
		pb_pin = PIND & 0xFC;

		// Update pressed and released state
		// Note: b_released must be cleared manually after the event is handled
		pb_released = pb_pressed & pb_pin;
		pb_pressed = ~pb_pin;

		if (pb_released & (1<< 6)) {
			led_state ^= 0xFF;
			pb_released &= ~(1<< 6);
		}

		if (led_state) {
			PORTD |= (1<< 0);
		} else {
			PORTD &= ~(1<< 0);
		}
	}

	return 0;
}
