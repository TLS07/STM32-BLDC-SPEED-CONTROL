/*
 * commutation.c
 *
 *  Created on: Dec 12, 2025
 *      Author: Admin
 */
#include "main.h"

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



/* Read hall states: PA0=HA, PA1=HB, PA2=HC */
static inline uint8_t read_hall(void)
{
    return (GPIOA->IDR & 0x07);
}


//commutation acroding to the sectors
void commutation(uint8_t hall)
{
	uint16_t duty=pwm_mapping();

	// Turn everything off first
	high_sides_off();
	low_sides_off();

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	switch (hall)
	{

	case SECTOR1:   // Sector 1 → A+, B-

		TIM1->CCR1=duty;
		TIM1->CCER |= TIM_CCER_CC1E;
		GPIOB->BSRR = LS_B;
		break;

	case SECTOR2:   // Sector 2 → A+, C-

		 TIM1->CCR1 = duty;
		 TIM1->CCER |= TIM_CCER_CC1E;
		 GPIOB->BSRR = LS_C;
		 break;

	case SECTOR3:  // Sector 3 → B+, C-

		 TIM1->CCR2 = duty;
		 TIM1->CCER |= TIM_CCER_CC2E;
		 GPIOB->BSRR = LS_C;
		 break;

	case SECTOR4:  // Sector 4 → B+, A-

		 TIM1->CCR2 = duty;
		 TIM1->CCER |= TIM_CCER_CC2E;
		 GPIOB->BSRR = LS_A;
		 break;

	case SECTOR5:   // Sector 5 → C+, A

		 TIM1->CCR3 = duty;
		 TIM1->CCER |= TIM_CCER_CC3E;
		 GPIOB->BSRR = LS_A;
		 break;

	case SECTOR6:  // Sector 6 → C+, B-

		 TIM1->CCR3 = duty;
		 TIM1->CCER |= TIM_CCER_CC3E;
		 GPIOB->BSRR = LS_B;
		 break;

	}
}
