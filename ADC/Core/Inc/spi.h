/*
 * spi.h
 *
 *  Created on: Mar 30, 2025
 *      Author: enrique
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_


#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_hal_gpio.h"
#include <sys_clk_config.h>

#define LCD_RS_Pin   		GPIO_PIN_0
#define LCD_RST_Pin  		GPIO_PIN_1
#define LCD_CS_Pin   		GPIO_PIN_4

extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern SPI_HandleTypeDef hspi1;


void SPI1_Init(void);

void spi_init(void);

void transferComm(uint8_t send_data);

void transferData(uint8_t send_data);

#endif /* INC_SPI_H_ */
