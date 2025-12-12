/*
 * main.h
 *
 *  Created on: Dec 12, 2025
 *      Author: Admin
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f1xx.h"

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

//functions protype
uint16_t adc_read(void);
void pwm_mapping(void);



//commutation protypes
static inline void low_sides_off(void);
static inline void high_sides_off(void);
inline uint8_t read_hall(void);
void commutation(uint8_t hall);



#endif /* MAIN_H_ */
