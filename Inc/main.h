/*
 * main.h
 *
 *  Created on: Dec 12, 2025
 *      Author: Admin
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f1xx.h"
#include <stdint.h>

//Motor parameters
#define POLE_PAIRS   4       // motor pole pairs
#define SPEED_WINDOW_MS  50  // 50 ms window
#define MAX_RPM     3000    // max RPM
#define INTEGRAL_MAX	17995  //to limit intergal error below ARR
#define INTEGRAL_MIN   -17995

//sectors macro
#define SECTOR1	0b110
#define SECTOR2	0b100
#define SECTOR3	0b101
#define SECTOR4	0b001
#define SECTOR5	0b011
#define SECTOR6	0b010


/* Low-side masks */
//BSRR ->BIt set/reset register  // to set/reset pins
//BRR ->bit reset register       // to reset pins

#define LS_A   GPIO_BSRR_BS3     //to set
#define LS_B   GPIO_BSRR_BS4
#define LS_C   GPIO_BSRR_BS5

#define LS_OFF_A GPIO_BRR_BR3    //to reset
#define LS_OFF_B GPIO_BRR_BR4
#define LS_OFF_C GPIO_BRR_BR5


//intialisation function protype
void clk_init(void);
void hall_sensor_init(void);
void low_side(void);
void pwm_init(void);
void adc_init(void);
void tim3_init(void);

//functions protype
uint16_t adc_read(void);
uint16_t pwm_mapping(void);
uint8_t  read_hall(void);


//commutation protypes

void commutation(uint8_t hall);

/* Turn OFF all low-side MOSFETs */
static inline void low_sides_off(void)
{
    GPIOB->BRR = LS_OFF_A | LS_OFF_B | LS_OFF_C;
}


/* Disable all high-side PWM outputs */
static inline void high_sides_off(void)
{
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
}





#endif /* MAIN_H_ */
