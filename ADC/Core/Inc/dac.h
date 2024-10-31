/*
 * dac.h
 *
 *  Created on: Oct 30, 2024
 *      Author: enrique
 */

#ifndef INC_DAC_H_
#define INC_DAC_H_

#include "stm32g4xx_hal.h"

extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac1_ch1;

void MX_DAC1_Init(void);

#endif /* INC_DAC_H_ */
