/*
 * isr.c
 *
 *  Created on: Dec 11, 2025
 *      Author: Admin
 */

/*
 * NOTE
 * EXTI Pending Register (EXTI-PR)
 *
 * =>The EXTI_PR register stores the pending interrupt flags for each EXTI line.
 * Whenever an external interrupt event occurs (rising/falling edge),
 * the corresponding PR bit is set to 1, indicating that an interrupt is waiting to be serviced.
 * =>To clear a pending bit, you must write a 1 to that bit (write-1-to-clear).
 * =>it understand intterupt if bit is set by hardware or software*/
#include"main.h"
volatile uint8_t hall_state;

/* Common handler to avoid code duplication */
static inline void hall_exti_handler(uint32_t pr_bit)
{
    if (EXTI->PR & pr_bit)
    {
        EXTI->PR = pr_bit;              // clear pending bit
        hall_state = read_hall();       // read all 3 hall pins
        commutation(hall_state);        // do commutation
    }
}

void EXTI0_IRQHandler(void)
{
    hall_exti_handler(EXTI_PR_PR0);
}

void EXTI1_IRQHandler(void)
{
    hall_exti_handler(EXTI_PR_PR1);
}

void EXTI2_IRQHandler(void)
{
    hall_exti_handler(EXTI_PR_PR2);
}


