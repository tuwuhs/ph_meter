/*
 * adc.h
 *
 *  Created on: Aug 1, 2012
 *      Author: tuwuhs
 *      (c) 2012 Tuwuh Sarwoprasojo
 */

#ifndef ADC_H_
#define ADC_H_

/* Exponential Moving Average filter parameters */
#define EMA_SCALE               (10)
#define EMA_DECAY_NUM           (1)
#define EMA_DECAY_DENOM         (32)

extern uint32_t g_adc_avg;
extern uint16_t g_adc_val;

void adc_init(void);
void adc_start(uint8_t ch);
void adc_stop(void);

#endif /* ADC_H_ */
