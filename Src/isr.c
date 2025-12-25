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
extern volatile uint32_t hall_pulse_count;
extern volatile uint32_t motor_rpm ;

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
    if(EXTI->PR & EXTI_PR_PR0)
    {
    	EXTI->PR = EXTI_PR_PR0; //Clear pending

    	// Rising edge → count pulse for speed
    	if (GPIOA->IDR & (1 << 0)) {
    		hall_pulse_count++;
    	 }
    	hall_state = read_hall();
    	commutation(hall_state);

    }

}

void EXTI1_IRQHandler(void)
{
    hall_exti_handler(EXTI_PR_PR1);
}

void EXTI2_IRQHandler(void)
{
    hall_exti_handler(EXTI_PR_PR2);
}



//timer 3 interrupt for speed caluction
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF)        // check interrupt
    {
        TIM3->SR &= ~TIM_SR_UIF;    // clear interrupt flag

        uint32_t pulses = hall_pulse_count;   // update pulse count
        hall_pulse_count = 0;

        // calculate speed
        /* MOTOR RPM=(Pulses counted in window×60×1000​)/ (time window (ms) *pole pairs)*/
        motor_rpm = (60 * pulses * 1000) / (SPEED_WINDOW_MS * POLE_PAIRS);
    }
}


