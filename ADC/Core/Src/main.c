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
#include "main.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include "arm_math.h"
#include "arm_const_structs.h"
#include "UART.h"
#include "adc.h"
#include "dac.h"
#include "timer.h"
#include "sys_clk_config.h"
#include "dma.h"
#include <stdio.h>
#include "fft.h"
#include "iir.h"

#define buffer_size 		512
#define half_buffer_size 	256
#define FFT_BUFFER_SIZE 	256

//#define ADC_MAX				4095
//#define SAMPLE_RATE_HZ		44100.0f

uint32_t adc_buffer		[buffer_size];






uint32_t dac_buffer		[buffer_size];

float32_t fft_buffer_out_1	[FFT_BUFFER_SIZE];
float32_t fft_buffer_in_1		[FFT_BUFFER_SIZE];

float32_t fft_buffer_out_2	[FFT_BUFFER_SIZE];
float32_t fft_buffer_in_2		[FFT_BUFFER_SIZE];



uint8_t HALF_BUFFER_FULL_FLAG							=	0;
uint8_t FULL_BUFFER_FULL_FLAG							=	0;



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
  MX_TIM7_Init();
  MX_DAC1_Init();

  HAL_ADC_Start_DMA(&hadc1,adc_buffer,buffer_size);
  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,dac_buffer,buffer_size,DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim7);






  while (1)
  {
	  if (HALF_BUFFER_FULL_FLAG)
	    {
		 //Once DMA ADC buffer is half-full, we can perform FFT on the respective buffer.


		  //convert first half of adc buffer into floating point
		  for(uint16_t i=0; i<half_buffer_size;i++)
		  {
			  fft_buffer_in_1[i]=((float32_t)adc_buffer[i] / 4095.0f) * 3.3f;
		  }
		  //calculate FFT

		  perform_fft(fft_buffer_in_1,fft_buffer_out_1);


		  for (int i = 0; i < sizeof(fft_buffer_out_1)/sizeof(fft_buffer_out_1[0]); i++)
		  {
			printf("%d, %f\r\n", i, fft_buffer_out_1[i]);
		  }

	      HALF_BUFFER_FULL_FLAG=0;




			//Lower flag raised by ISR

	    }

	    if (FULL_BUFFER_FULL_FLAG)
	    {
	    	//convert first half of adc buffer into floating point
		  for(uint16_t i=half_buffer_size; i<buffer_size;i++)
		  {
			  fft_buffer_in_2[i-half_buffer_size]=((float32_t)adc_buffer[i] / 4095.0f) * 3.3f;
		  }
		  //calculate FFT

		  perform_fft(fft_buffer_in_2,fft_buffer_out_2);


		  for (int i = 0; i < sizeof(fft_buffer_out_2)/sizeof(fft_buffer_out_2[0]); i++)
		  {
			printf("%d, %f\r\n", i, fft_buffer_out_2[i]);
		  }

		  FULL_BUFFER_FULL_FLAG = 0;

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
	//Check to see if main program is still using ADC buffer data
	if(!HALF_BUFFER_FULL_FLAG)
	{

		HALF_BUFFER_FULL_FLAG=1;
	}

//	for(int i=0;i<half_buffer_size;i++)
//	{
//		dac_buffer[i]=adc_buffer[i];
//	}

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

//for(int i=half_buffer_size;i<buffer_size;i++)
//{
//	dac_buffer[i]=adc_buffer[i];
//}
////	printf("Sample Complete \n\r");

	if(!FULL_BUFFER_FULL_FLAG)
	{

		FULL_BUFFER_FULL_FLAG=1;
	}

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
