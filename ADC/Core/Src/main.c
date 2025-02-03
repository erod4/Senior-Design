/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <adc.h>
#include <arm_const_structs.h>
#include <arm_math.h>
#include <dac.h>
#include <dma.h>
#include <fft.h>
#include <iir.h>
#include <main.h>
#include "arm_math.h"
#include <stdio.h>
#include <sys_clk_config.h>
#include <timer.h>
#include <UART.h>

#define buffer_size 		512
#define half_buffer_size 	256
#define FFT_BUFFER_SIZE 	256


uint32_t adc_buffer				[buffer_size];
uint32_t dac_buffer				[buffer_size];


float32_t fft_buffer_out_1		[FFT_BUFFER_SIZE];
float32_t fft_buffer_in_1		[FFT_BUFFER_SIZE];

float32_t fft_buffer_out_2		[FFT_BUFFER_SIZE];
float32_t fft_buffer_in_2		[FFT_BUFFER_SIZE];

float32_t ifft_buffer_out_1		[FFT_BUFFER_SIZE];
float32_t ifft_buffer_out_2		[FFT_BUFFER_SIZE];

float32_t scaling_multiplier_buffer_1 [FFT_BUFFER_SIZE];
float32_t scaling_multiplier_buffer_2 [FFT_BUFFER_SIZE];



volatile uint8_t HALF_BUFFER_FULL_FLAG							=	0;
volatile uint8_t FULL_BUFFER_FULL_FLAG							=	0;



static void MX_GPIO_Init(void);




int main(void)
{


  HAL_Init();


  SystemClock_Config();

  uart_init();
  init_fft(FFT_BUFFER_SIZE);
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_DAC1_Init();

  HAL_ADC_Start_DMA(&hadc1,adc_buffer,buffer_size);
  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,dac_buffer,buffer_size,DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_Base_Start(&htim7);
  HAL_TIM_Base_Start(&htim6);
  __HAL_TIM_SET_COUNTER(&htim6, 0);  // Reset Timer 6 counter to 0



  while (1)
  {

	if(HALF_BUFFER_FULL_FLAG)
	{

		perform_fft(fft_buffer_in_1, fft_buffer_out_1);
		perform_ifft(fft_buffer_out_1,ifft_buffer_out_1);
		for(int i=0; i<half_buffer_size;i++)
		{
			dac_buffer[i] = (uint32_t)((ifft_buffer_out_1[i] / 3.3f) * 4095);

		}
		HALF_BUFFER_FULL_FLAG=0;
	}
	if(FULL_BUFFER_FULL_FLAG)
	{
		perform_fft(fft_buffer_in_2, fft_buffer_out_2);
		perform_ifft(fft_buffer_out_2,ifft_buffer_out_2);
		for(int i=half_buffer_size; i<buffer_size;i++)
		{
			dac_buffer[i]= (uint32_t)((ifft_buffer_out_2[i-half_buffer_size] / 3.3f) * 4095);

		}
		FULL_BUFFER_FULL_FLAG=0;
	}



  }

}






/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
 static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
if(!HALF_BUFFER_FULL_FLAG)
{
	for(int i=0;i<half_buffer_size;i++)
		{
			fft_buffer_in_1[i] = ((float)adc_buffer[i] / 4095.0f) * 3.3f;
			scaling_multiplier_buffer_1=((float)adc_buffer[i] / 4095.0f) * 3.3f;
		}
		HALF_BUFFER_FULL_FLAG=1;
}


}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

if(!FULL_BUFFER_FULL_FLAG)
	{
		for(int i=half_buffer_size;i<buffer_size;i++)
		{
		fft_buffer_in_2[i-half_buffer_size] = ((float)adc_buffer[i] / 4095.0f) * 3.3f;
		scaling_multiplier_buffer_2=((float)adc_buffer[i] / 4095.0f) * 3.3f;
		}
		FULL_BUFFER_FULL_FLAG=1;
	}
}



void TIM8_UP_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim8);


}










/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  printf("Error \n\r");

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
