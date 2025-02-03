/*
 * timer.h
 *
 *  Created on: Oct 31, 2024
 *      Author: enrique
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "stm32g4xx_hal.h"

extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim6;

void MX_TIM8_Init(void);

void MX_TIM7_Init(void);

void MX_TIM6_Init(void);


#endif /* INC_TIMER_H_ */
