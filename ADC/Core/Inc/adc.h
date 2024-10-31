

#ifndef ADC_H_
#define ADC_H_
#include "stm32g4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

void MX_ADC1_Init(void);


#endif /* INC_ADC_H_ */
