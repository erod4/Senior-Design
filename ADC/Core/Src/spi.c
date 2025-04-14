/*
 * spi.c
 *
 *  Created on: Mar 30, 2025
 *      Author: enrique
 */


#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_hal_gpio.h"
#include <sys_clk_config.h>
#include "main.h"
#include "spi.h"
#include <stdio.h>
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

SPI_HandleTypeDef hspi1;

void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	    /* Enable clocks for all required GPIO ports */
	    __HAL_RCC_GPIOC_CLK_ENABLE();
	    __HAL_RCC_GPIOF_CLK_ENABLE();
	    __HAL_RCC_GPIOA_CLK_ENABLE();
	    __HAL_RCC_GPIOB_CLK_ENABLE();

	    /* Enable SPI1 clock */
	    __HAL_RCC_SPI1_CLK_ENABLE();

	    /* Initialize LCD control pins on GPIOA (LCD_RS, LCD_RST, LCD_CS) */
	    HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin|LCD_RST_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

	    GPIO_InitStruct.Pin = LCD_RS_Pin | LCD_RST_Pin | LCD_CS_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	    /* Initialize PA7 for SPI1 MOSI */
	    GPIO_InitStruct.Pin = GPIO_PIN_7;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	    /* Initialize PB3 for SPI1 SCK */
	    GPIO_InitStruct.Pin = GPIO_PIN_3;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    /* Configure the SPI1 module */
	    hspi1.Instance = SPI1;
	    hspi1.Init.Mode = SPI_MODE_MASTER;
	    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	    hspi1.Init.NSS = SPI_NSS_SOFT;
	    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	    hspi1.Init.CRCPolynomial = 7;
	    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	    if (HAL_SPI_Init(&hspi1) != HAL_OK)
	    {
	        Error_Handler();
	    }
	    //configure DMA
	    __HAL_RCC_DMAMUX1_CLK_ENABLE();
	    __HAL_RCC_DMA2_CLK_ENABLE();
	    //rx
	    hdma_spi1_rx.Instance=DMA2_Channel2;
	    hdma_spi1_rx.Init.Direction=DMA_PERIPH_TO_MEMORY;
	    hdma_spi1_rx.Init.PeriphInc=DMA_PINC_DISABLE;
	    hdma_spi1_rx.Init.MemInc=DMA_MINC_ENABLE;
	    hdma_spi1_rx.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;
	    hdma_spi1_rx.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;
	    hdma_spi1_rx.Init.Mode=DMA_NORMAL;
	    hdma_spi1_rx.Init.Priority=DMA_PRIORITY_HIGH;
	    if(HAL_DMA_Init(&hdma_spi1_rx))
	    {
	    	printf("Error\r\n");

	    }
//	    __HAL_LINKDMA(hspi1,hdmarx,hdma_spi1_rx);

	    HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
	    HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
	    __HAL_LINKDMA(&hspi1,hdmarx,hdma_spi1_rx);
	    //tx
	    hdma_spi1_tx.Instance=DMA2_Channel3;
	    hdma_spi1_tx.Init.Direction=DMA_MEMORY_TO_PERIPH;
	    hdma_spi1_tx.Init.PeriphInc=DMA_PINC_DISABLE;
	    hdma_spi1_tx.Init.MemInc=DMA_MINC_ENABLE;
	    hdma_spi1_tx.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;
	    hdma_spi1_tx.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;
	    hdma_spi1_tx.Init.Mode=DMA_NORMAL;
	    hdma_spi1_tx.Init.Priority=DMA_PRIORITY_HIGH;
	    if(HAL_DMA_Init(&hdma_spi1_tx))
	    {
	    	printf("Error\r\n");
	    }
//	    __HAL_LINKDMA(hspi1,hdmatx,hdma_spi1_tx);

	    HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
	    HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
	    __HAL_LINKDMA(&hspi1,hdmatx,hdma_spi1_tx);

	}


void spi_init(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}



void transferComm(uint8_t send_data) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	// Set LCD_RS pin low to set as COMMAND
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	// Set LCD_CS pin low to use LCD

	HAL_SPI_Transmit(&hspi1, &send_data, 1, HAL_MAX_DELAY);	// Send the send_data data, also waits for transmission complete

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);		// Set LCD_CS pin high since LCD no longer used
}

void transferData(uint8_t send_data) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);		// Set LCD_RS pin high to set as DATA
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	// Set LCD_CS pin low to use LCD

	HAL_SPI_Transmit(&hspi1, &send_data, 1, HAL_MAX_DELAY);	// Send the send_data data, also waits for transmission complete

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);		// Set LCD_CS pin high since LCD no longer used
}
void DMA2_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel2_IRQn 0 */

  /* USER CODE END DMA2_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA2_Channel2_IRQn 1 */

  /* USER CODE END DMA2_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel3 global interrupt.
  */
void DMA2_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel3_IRQn 0 */

  /* USER CODE END DMA2_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Channel3_IRQn 1 */

  /* USER CODE END DMA2_Channel3_IRQn 1 */
}

