/*
 * isr.c
 *
 *  Created on: Dec 11, 2025
 *      Author: Admin
 */
#include"stm32f1xx.h"
/*
 * NOTE
 * EXTI Pending Register (EXTI-PR)
 *
 * =>The EXTI_PR register stores the pending interrupt flags for each EXTI line.
 * Whenever an external interrupt event occurs (rising/falling edge),
 * the corresponding PR bit is set to 1, indicating that an interrupt is waiting to be serviced.
 * =>To clear a pending bit, you must write a 1 to that bit (write-1-to-clear).
 * =>it understand intterupt if bit is set by hardware or software*/
void EXTI0_IRQHAndler(void)
{
	if(EXTI->PR & EXTI_PR_PR0)
	{
		EXTI->PR = EXTI_PR_PR0;
		hall_state=read_hall();
		commutate(hall_state);
	}
}

void EXTI1_IRQHAndler(void)
{
	if(EXTI->PR & EXTI_PR_PR1)
	{
		EXTI->PR = EXTI_PR_PR1;
		hall_state=read_hall();
		commutate(hall_state);
	}
}

void EXTI2_IRQHAndler(void)
{
	if(EXTI->PR & EXTI_PR_PR2)
	{
		 EXTI->PR = EXTI_PR_PR2;
		hall_state=read_hall();
		commutate(hall_state);
	}
}

