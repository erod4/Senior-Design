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
#define ADC_MAX				4095
#define SCALE_VAL			32767
#define SAMPLING_RATE		44100

uint32_t adc_buffer		[buffer_size];
uint32_t dac_buffer		[buffer_size];

q15_t Q15_fft_buffer_in1[FFT_BUFFER_SIZE];
q15_t Q15_fft_buffer_out1[FFT_BUFFER_SIZE];

q15_t Q15_fft_buffer_in2[FFT_BUFFER_SIZE];
q15_t Q15_fft_buffer_out2[FFT_BUFFER_SIZE];


q15_t Q15_iir_buffer_out1[FFT_BUFFER_SIZE];
q15_t Q15_iir_buffer_out2[FFT_BUFFER_SIZE];

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
		  //convert data from uint_16t to Q15 and store in FFT buffer
		for(int i=0; i<FFT_BUFFER_SIZE;i++)
		{
			Q15_fft_buffer_in1[i]=(q15_t)(adc_buffer[i]<<4);


		}
		  printf("Starting FFT\n\r");

	      perform_fft(Q15_fft_buffer_in1, Q15_fft_buffer_out1,FFT_BUFFER_SIZE);
		  printf("FFT Ended\n\r");

	      //Loop through each value of FFT out buff and print

		  for(int i=0; i<FFT_BUFFER_SIZE;i++)
		  {
			  printf("Bin %d: %d \n\r",i+1,Q15_fft_buffer_out1[i]);
		  }
		  printf("Ending Program Now\n\r");
		  HALF_BUFFER_FULL_FLAG = 0;

break;
printf("After Ending Program\n\r");

			//Lower flag raised by ISR

	    }

	    if (FULL_BUFFER_FULL_FLAG)
	    {
	    	 //Once DMA ADC buffer is half-full, we can perform FFT on the respective buffer.
			 //convert data from uint_16t to Q15 and store in FFT buffer
			for(int i=0; i<FFT_BUFFER_SIZE;i++)
			{

				Q15_fft_buffer_in2[i] = (q15_t)(uint16_t)adc_buffer[i+FFT_BUFFER_SIZE];
			}



	      perform_fft(Q15_fft_buffer_in2, Q15_fft_buffer_out2,FFT_BUFFER_SIZE);

	      //Loop through each value of FFT out buff and print

		  for(int i=0; i<FFT_BUFFER_SIZE;i++)
		  {
			  printf("%d \n\r",Q15_fft_buffer_out2[i]);
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
//printf("Sample Complete \n\r");
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
//	printf("Sample Complete \n\r");

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
