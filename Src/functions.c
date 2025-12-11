/*
 * functions.c
 *
 *  Created on: Dec 11, 2025
 *      Author: Admin
 */
#include"stm32f1xx.h"

uint16_t adc_read(void)
{
	//Start adc conversion
	ADC1->CR2|=ADC_CR2_ADON;
	ADC1->CR2|=ADC_CR2_SWSTART;

	//wait till end of conversion
	while(!(ADC1->SR & ADC_SR_EOC));
	return (uint16_t) ADC1->DR;
}

void pwm_mapping(void){

	uint16_t adc_value=adc_read();
	//duty = (adc * PWM_MAX)/ADC_MAx
	uint32_t duty=((uint32_t)adc_value*3599)/4095;

	//updating value to caputre compare register
	//CNT<<CCrx ->high vice versa low
	//ARR vaue only for pwm period

	TIM1->CCR1=duty;    //channel 1
	TIM1->CCR2=duty;	//channel 2
	TIM1->CCR3=duty;	//channel 3
}


