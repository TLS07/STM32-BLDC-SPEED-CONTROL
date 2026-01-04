/*
 * functions.c
 *
 *  Created on: Dec 11, 2025
 *      Author: Admin
 */
#include "main.h"
 char speed_buf[16];       //lcd max 16 charcters
 char target_buf[16];
uint16_t adc_read(void)
{
	//Start adc conversion
	ADC1->CR2|=ADC_CR2_ADON;
	ADC1->CR2|=ADC_CR2_SWSTART;

	//wait till end of conversion
	while(!(ADC1->SR & ADC_SR_EOC));
	return (uint16_t) ADC1->DR;        //readind Data register of adc it also clears eoc
}


// Read hall states: PA0=HA, PA1=HB, PA2=HC
uint8_t read_hall(void)
{
    return (uint8_t)(GPIOA->IDR & 0x07);
}


//to display motor speed on lcd
void lcd_display_rpm(uint32_t rpm,uint32_t target)
{
	lcd_clear();
	lcd_set_cursor(0,0);     //first row
	sprintf(speed_buf,"RPM: %lu",rpm);
	lcd_print_string(speed_buf);

	lcd_set_cursor(1,0);    //second row
	sprintf(target_buf,"Target RPM: %lu",target);
	lcd_print_string(target_buf);


}

