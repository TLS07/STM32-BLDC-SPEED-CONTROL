/*
 * init.c
 *
 *  Created on: Dec 10, 2025
 *      Author: Admin
 */
#include "main.h"
//clock initalization
void clk_init(void)
{
    //step 1 selecting clock source
	RCC->CR|=RCC_CR_HSEON;              //clock source
	while(!(RCC->CR & RCC_CR_HSERDY));     //wait till clock source is ready

	// step 2 cofigure flash prefetch and latency
	FLASH->ACR|=FLASH_ACR_PRFTBE;        //enabling prefetch buffer
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR|=FLASH_ACR_LATENCY_2;       //system clk 72mhz

	//step 3 configuring PLL
	RCC->CFGR|=RCC_CFGR_PLLSRC;            //HSE as source for PLL
	RCC->CFGR&=~RCC_CFGR_PLLXTPRE;          // HSE not divided
	RCC->CFGR &= ~RCC_CFGR_PLLMULL;         //clearing the bits
	RCC->CFGR|=RCC_CFGR_PLLMULL9;            //8*9=72 mhz

	RCC->CFGR|=RCC_CFGR_HPRE_DIV1;          //APB2 not divided =>72mhz
	RCC->CFGR|=RCC_CFGR_PPRE1_DIV2;        //APB1 divided by 2 =>36mhz
	RCC->CFGR|=RCC_CFGR_PPRE2_DIV1;        //AHB not divided =>72mhz


	//step 4 enable  PLL and wait to get ready
	RCC->CR|=RCC_CR_PLLON;                  //PLL on
	while(!(RCC->CR& RCC_CR_PLLRDY));       //wait till PLL is ready

	//step 5 enable clock source
	RCC->CFGR &= ~RCC_CFGR_SW;               // clear SW[1:0]
	RCC->CFGR|=RCC_CFGR_SW_PLL;              //PLL as sytem clock
}

void hall_sensor_init(void)
{
	//step 1 enable clock for GPIO
	RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;    //GPIOA clock
	RCC->APB2ENR|=RCC_APB2ENR_AFIOEN;   //alternate function clock

	//step 2 configuring PA0 PA1 PA2 in input mode wit pull up

	//PA0 setup
	GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);  // input mode, clear CNF
	GPIOA->CRL|=GPIO_CRL_CNF0_1;                      // input with pull up/ down
	GPIOA->ODR |=  GPIO_ODR_ODR0;                     // enable pull-up

	//PA1 setup
	GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);  // input mode, clear CNF
	GPIOA->CRL|=GPIO_CRL_CNF1_1;                      // input with pull up/ down
	GPIOA->ODR |=  GPIO_ODR_ODR1;                     // enable pull-up

	//PA2 setup
	GPIOA->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2);  // input mode, clear CNF
	GPIOA->CRL|=GPIO_CRL_CNF2_1;                      // input with pull up/ down
	GPIOA->ODR |=  GPIO_ODR_ODR2;                     // enable pull-up

	//step 3 mapping PA0 PA1 PA2 to EXTI0,EXTI1,EXTI2

	// Clear bits for EXTI0, EXTI1, EXTI2
	AFIO->EXTICR[0] &= ~((0xF << 0) | (0xF << 4) | (0xF << 8));

	//step 4 Mapping
	AFIO->EXTICR[0]|=AFIO_EXTICR1_EXTI0_PA;   //EXTI0 → PA0
	AFIO->EXTICR[0]|=AFIO_EXTICR1_EXTI1_PA;   //EXTI1 → PA1
	AFIO->EXTICR[0]|=AFIO_EXTICR1_EXTI2_PA;   //EXTI2 → PA2

	//step 5 enabling interrupt on EXTI lines
	EXTI->IMR|=EXTI_IMR_MR0;    //PA0
	EXTI->IMR|=EXTI_IMR_MR1;    //PA1
	EXTI->IMR|=EXTI_IMR_MR2;    //PA2

	//step 6 enabling interrupt to trigger on falling edge and rising edge

	//rising edge
	EXTI->RTSR|=EXTI_RTSR_TR0;
	EXTI->RTSR|=EXTI_RTSR_TR1;
	EXTI->RTSR|=EXTI_RTSR_TR2;

	//falling edge
	EXTI->FTSR|=EXTI_FTSR_TR0;
	EXTI->FTSR|=EXTI_FTSR_TR1;
	EXTI->FTSR|=EXTI_FTSR_TR2;

	 // step 7. Enable NVIC for EXTI0, EXTI1, EXTI2
	 NVIC_EnableIRQ(EXTI0_IRQn);
	 NVIC_EnableIRQ(EXTI1_IRQn);
	 NVIC_EnableIRQ(EXTI2_IRQn);


	 EXTI->PR |= (1 << 0) | (1 << 1) | (1 << 2); // clear pending bits

}

//setting PB3 PB4 PB5 as gpio output to drive the low side of the bridge
void low_side(void)
{
	//step 1 enable clock for GPIO
	RCC->APB2ENR|=RCC_APB2ENR_IOPBEN;    //GPIOA clock

	//step 2 GPIO output mode configuration 50 mhz
	GPIOB->CRL|=(GPIO_CRL_MODE3_0|GPIO_CRL_MODE3_1);
	GPIOB->CRL|=(GPIO_CRL_MODE4_0|GPIO_CRL_MODE4_1);
	GPIOB->CRL|=(GPIO_CRL_MODE5_0|GPIO_CRL_MODE5_1);

	//step 3 configuring in push pull configuration
	GPIOB->CRL&=~(GPIO_CRL_CNF3_0 |GPIO_CRL_CNF3_1);
	GPIOB->CRL&=~(GPIO_CRL_CNF4_0 |GPIO_CRL_CNF4_1);
	GPIOB->CRL&=~(GPIO_CRL_CNF5_0 |GPIO_CRL_CNF5_1);

	//to initially keep all pins low
	GPIOB->BRR|=(GPIO_BRR_BR3|GPIO_BRR_BR4|GPIO_BRR_BR5);  // initially PB3,PB4,PB5 low


}

void pwm_init(void)   //TIM1 CH1 PA8 CH2 PA9 CH3 PA10
{
	//step 1 enable the clock
	RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;   //GPIOA Clock
	RCC->APB2ENR|=RCC_APB2ENR_AFIOEN;    // AFIO clock
	RCC->APB2ENR|=RCC_APB2ENR_TIM1EN;    //TIM1 clock

	//step 2  Configure PA8, PA9, PA10 as AF push-pull, 50 MHz

	  //PA8 TIM1->CH1
	  GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
	  GPIOA->CRH |=  (GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0);    // Output 50 MHz
	  GPIOA->CRH |=  (GPIO_CRH_CNF8_1);

	  //PA9 TIM1->CH2
	  GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
	  GPIOA->CRH |=  (GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0);    // Output 50 MHz
	  GPIOA->CRH |=  (GPIO_CRH_CNF9_1);

	 //PA10 TIM1->CH3
	  GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
	  GPIOA->CRH |=  (GPIO_CRH_MODE10_1 | GPIO_CRH_MODE10_0);  // Output 50 MHz
	  GPIOA->CRH |=  (GPIO_CRH_CNF10_1);


	  // step 3 configrung the timer base for 20khz PWM
	  // F_pwm=F_clk/(arr+1)*(psc+1)

	  TIM1->PSC=0; 	        // Prescaler = 0
	  TIM1->ARR=3599;       // Auto-reload = 3599 → 20 kHz


	  //step 4 PWM mode for ch1 , ch2 ch3
	  TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk | TIM_CCMR2_OC3M_Msk);
	  TIM1->CCMR1|=(TIM_CCMR1_OC1M_1 |TIM_CCMR1_OC1M_2);    //PWM mode1 ch1
	  TIM1->CCMR1|=(TIM_CCMR1_OC2M_1 |TIM_CCMR1_OC2M_2);    //PWM mode1 ch2
	  TIM1->CCMR2|=(TIM_CCMR2_OC3M_1 |TIM_CCMR2_OC3M_2);    //PWM mode1 ch3

	  //step 5 enble preload for each channel and ARR
	  TIM1->CCMR1|=TIM_CCMR1_OC1PE;
	  TIM1->CCMR1|=TIM_CCMR1_OC2PE;
	  TIM1->CCMR2|=TIM_CCMR2_OC3PE;
	  TIM1->CR1|=TIM_CR1_ARPE;     // autoreload preload enable

	  //step 6 enabling the channel's
	  TIM1->CCER|=TIM_CCER_CC1E;      //ch1
	  TIM1->CCER|=TIM_CCER_CC2E;	  //ch2
	  TIM1->CCER|=TIM_CCER_CC3E;	  //ch3

	  //step 7 enabling the timer
	  TIM1->BDTR|=TIM_BDTR_MOE;        //it should be done for advance timers
	  TIM1->CR1|=TIM_CR1_CEN;          //timer enabled

}

//channel 3 PA3
void adc_init(void)
{
	// step1 Enable clock
	 RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   // GPIOA
	 RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;   // ADC1

	 //step 2 configure PA3 as analog
	 GPIOA->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3);


	 //step 3 ADC prescaler  ADC max 14mhz , here 12Mhz
	 RCC->CFGR &= ~RCC_CFGR_ADCPRE;
	 RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

	 //step 4 set sampling time ch3-> 55.5 cycles (note total sample time =12.5 cycles +sampling cycles)
	 ADC1->SMPR2 &= ~ADC_SMPR2_SMP3;
	 ADC1->SMPR2 |= (ADC_SMPR2_SMP3_1 | ADC_SMPR2_SMP3_0);

	 //step 5 configure number of conversion , and channel
	 ADC1->SQR1 = 0;        // L=0 → 1 conversion, SQR1 describes number of conversion
	 ADC1->SQR3 = 3;        // First conversion ch3 SQ1[4:0]=3

	 //step 6 power on  and calibrate
	 ADC1->CR2 = ADC_CR2_ADON;
	 for (volatile short i = 0; i < 1000; i++);

	 //calibration
	 ADC1->CR2 |= ADC_CR2_CAL;
	 while (ADC1->CR2 & ADC_CR2_CAL);
}

void tim3_init(void)
{
	//step 1 enbaling clock for timer3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	//step 2 set prescaler and autoreload value
	TIM3->PSC = 7199;  // 72 MHz / (7199+1) = 10 kHz
	TIM3->ARR = 499;   // 10 kHz / 500 = 20 Hz → 50 ms

	//step 3 enable interrupt
	TIM3->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM3_IRQn);

	//step 4 start timer
	TIM3->CR1 |= TIM_CR1_CEN;

}


