/*
 * functions.c
 *
 *  Created on: Dec 11, 2025
 *      Author: Admin
 */
#include "main.h"

uint16_t adc_read(void)
{
	//Start adc conversion
	ADC1->CR2|=ADC_CR2_ADON;
	ADC1->CR2|=ADC_CR2_SWSTART;

	//wait till end of conversion
	while(!(ADC1->SR & ADC_SR_EOC));
	return (uint16_t) ADC1->DR;        //readind Data register of adc it also clears eoc
}

uint16_t pwm_mapping(void){

	uint16_t adc_value=adc_read();
	if(adc_value>4095)
	{
		adc_value=4095;
	}

	//duty = (adc * PWM_MAX)/ADC_MAx
	uint16_t duty=(adc_value*(TIM1->ARR))/4095;

	return duty;

}

// Read hall states: PA0=HA, PA1=HB, PA2=HC
uint8_t read_hall(void)
{
    return (uint8_t)(GPIOA->IDR & 0x07);
}

