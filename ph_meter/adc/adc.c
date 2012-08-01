/*
 * adc.c
 *
 *  Created on: Aug 1, 2012
 *      Author: tuwuhs
 *      (c) 2012 Tuwuh Sarwoprasojo
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "adc.h"

// These two variables are globally accessible by including adc.h
uint32_t g_adc_avg = 0;
uint16_t g_adc_val = 0;

ISR(ADC_vect)
{
	static uint32_t sum = 0;
	static uint8_t samples = 0;

	/* Clear Timer/Counter0 Overflow flag */
	TIFR |= (1<< OCF0);

	/* Get ADC result
	 * Use oversampling to get a 12-bit result
	 * 2 additional bits --> 16 samples needed
	 * Sample rate after oversampling: 7812.5 / 16 = 488.2 Hz
	 */
	sum += ADC;
	samples = (samples + 1) % 16;

	/* Use fixed-point arithmetic for Exponential Moving Average filter
	 * Fixed-point arithmetic scaling factor: 2**10 (see macro EMA_SCALE)
	 * Decay factor: 1:32 (see macros EMA_DECAY_NUM, EMA_DECAY_DENOM)
	 *   Settling time (95%) is 3/alpha = 3/(1/32) = 96 samples
	 *   With sampling rate ~488Hz, the settling time is therefore ~20ms
	 *
	 * Apply EMA to the oversampled result
	 * Update only when the samples have accumulated
	 *
	 * Algorithm:
	 * Decay factor defined as NUM / DENOM
	 * curr_value = oversampling_sum / 4
	 * avg = (curr_value * NUM + avg * (DENOM - NUM)) / DENOM,
	 *    round properly
	 * avg_val: de-scale and round properly
	 */
	if (samples == 0) {
		g_adc_avg = ((  (((sum / 4) << EMA_SCALE) * EMA_DECAY_NUM) +
		                 (              g_adc_avg * (EMA_DECAY_DENOM - EMA_DECAY_NUM)))
		            + (EMA_DECAY_DENOM / 2)) / EMA_DECAY_DENOM;
		g_adc_val = (g_adc_avg + ((1 << EMA_SCALE) / 2)) >> EMA_SCALE;
		sum = 0;
	}
}

void adc_init(void)
{
	/* Initialize the ADC
	 * Prescale clock by 128 (system clock is 16MHz, so we obtain an ADC clock of 125kHz)
	 * Enable ADC interrupt
	 * Reference: AVCC
	 * ADC auto trigger source: Timer/Counter0 Overflow
	 */
	ADMUX = (1<< REFS0) | (1<< REFS1);
	ADCSRA = (1<< ADEN) | (1<< ADIE) | (1<< ADPS2) | (1<< ADPS1) | (1<< ADPS0);
	SFIOR = (1<< ADTS2);
}

void adc_start(uint8_t ch)
{
	/* Set ADC mux to channel ch */
	ADMUX &= ~((1<< MUX4) | (1<< MUX3) | (1<< MUX2) | (1<< MUX1) | (1<< MUX0));
	ADMUX |= ch & ((1<< MUX4) | (1<< MUX3) | (1<< MUX2) | (1<< MUX1) | (1<< MUX0));

	/* Initialize timer for ADC sampling trigger
	 * ADC sample rate = sysclk / prescaler / 256
	 * Prescaler: 8 (sample rate: 7812.5 Hz)
	 */
	TCNT0 = 0;
	TCCR0 = (1<< CS01);

	/* Start ADC conversion by enabling ADC auto trigger */
	ADCSRA |= (1<< ADATE);
}

void adc_stop(void)
{
	/* Stop ADC conversion by disabling ADC auto trigger */
	ADCSRA &= ~(1<< ADATE);

	/* Stop the timer */
	TCCR0 = 0;
}
