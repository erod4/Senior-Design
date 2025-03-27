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
#include <math.h>
#include <inttypes.h>

#define NUM_MEASUREMENTS   50

#define BUFFER_SIZE 		512
#define HALF_BUFFER_SIZE 	256

#define FFT_BUFFER_SIZE 	256


uint32_t adc_buffer				[BUFFER_SIZE];

float32_t fft_buffer_out_1		[FFT_BUFFER_SIZE];
float32_t fft_buffer_in_1		[FFT_BUFFER_SIZE];

float32_t fft_buffer_out_2		[FFT_BUFFER_SIZE];
float32_t fft_buffer_in_2		[FFT_BUFFER_SIZE];


float32_t iir_buffer_in_1		[FFT_BUFFER_SIZE/2];
float32_t iir_buffer_in_2		[FFT_BUFFER_SIZE/2];

float32_t iir_buffer_out_1		[FFT_BUFFER_SIZE/2];
float32_t iir_buffer_out_2		[FFT_BUFFER_SIZE/2];

float32_t lcd_buffer_1			[FFT_BUFFER_SIZE/2];
float32_t lcd_buffer_2			[FFT_BUFFER_SIZE/2];

volatile uint8_t HALF_BUFFER_FULL_FLAG							=	0;
volatile uint8_t FULL_BUFFER_FULL_FLAG							=	0;
volatile uint8_t LCD_UPDATE_FLAG 								=	0;


// Arrays to store WCET measurements
uint32_t wcet_half_buffer[NUM_MEASUREMENTS];
uint32_t wcet_full_buffer[NUM_MEASUREMENTS];
uint8_t half_idx = 0;
uint8_t full_idx = 0;

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

  HAL_ADC_Start_DMA(&hadc1,adc_buffer,BUFFER_SIZE);
//  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,dac_buffer,BUFFER_SIZE,DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_Base_Start(&htim7);
  HAL_TIM_Base_Start(&htim6);
  __HAL_TIM_SET_COUNTER(&htim6, 0);  // Reset Timer 6 counter to 0


  /////////////////////////////////////////////////////////////////////////////////////////////////////////





  while (1)
  {

	if(HALF_BUFFER_FULL_FLAG)
	{
		// Get start time from Timer8
		uint32_t start_time = __HAL_TIM_GET_COUNTER(&htim8);



		//Once converted to floating point the FFT is performed on the first half of buffer
		perform_fft(fft_buffer_in_1, fft_buffer_out_1);

		//Take absolute magnitude of FFT output
		arm_cmplx_mag_f32(fft_buffer_out_1,iir_buffer_in_1,(uint32_t)(FFT_BUFFER_SIZE/2));

		//Do IIR filtering for each FFT value
		for(int i=0;i<FFT_BUFFER_SIZE/2;i++)
		{
			iir_buffer_out_1[i]=lowpass_FIR_IIR_filter(iir_buffer_in_1[i]);
		}
		//Lastly store in a LCD screen buffer
		for(int i=0; i<FFT_BUFFER_SIZE/2;i++)
		{
			lcd_buffer_1[i]=iir_buffer_out_1[i];
		}
		// Get end time from Timer8 and compute elapsed time
		uint32_t end_time = __HAL_TIM_GET_COUNTER(&htim8);
		uint32_t elapsed;
		// Handle wrap-around (assuming TIM8 period is htim8.Init.Period)
		if (end_time >= start_time)
		{
			elapsed = end_time - start_time;
		}
		else
		{
			elapsed = (htim8.Init.Period + 1 - start_time) + end_time;
		}

		// Store the measurement if we haven't reached NUM_MEASUREMENTS yet
		if (half_idx < NUM_MEASUREMENTS)
		{
			wcet_half_buffer[half_idx] = elapsed;
			half_idx++;
		}
		//Measure WCET then make an ISR to have LCD update values at that time

		HALF_BUFFER_FULL_FLAG=0;
	}
	//Once converted to floating point the FFT is performed on the second half of buffer
	if(FULL_BUFFER_FULL_FLAG)
	{
        uint32_t start_time = __HAL_TIM_GET_COUNTER(&htim8);

		perform_fft(fft_buffer_in_2, fft_buffer_out_2);

		//Take absolute magnitude of FFT output
		arm_cmplx_mag_f32(fft_buffer_out_2,iir_buffer_in_2,(uint32_t)(FFT_BUFFER_SIZE/2));

		//Do IIR filtering for each FFT value
		for(int i=0;i<FFT_BUFFER_SIZE/2;i++)
		{
			iir_buffer_out_2[i]=lowpass_FIR_IIR_filter(iir_buffer_in_2[i]);
		}

		//Lastly store in a LCD screen buffer
		for(int i=0; i<FFT_BUFFER_SIZE/2;i++)
		{
			lcd_buffer_2[i]=iir_buffer_out_2[i];
		}
		//Measure WCET then make an ISR to have LCD update values at that time
		uint32_t end_time = __HAL_TIM_GET_COUNTER(&htim8);
		uint32_t elapsed;
		if (end_time >= start_time)
		{
			elapsed = end_time - start_time;
		}
		else
		{
			elapsed = (htim8.Init.Period + 1 - start_time) + end_time;
		}

		if (full_idx < NUM_MEASUREMENTS)
		{
			wcet_full_buffer[full_idx] = elapsed;
			full_idx++;
		}

		FULL_BUFFER_FULL_FLAG=0;
	}
	// Once both half-buffer and full-buffer measurements have been collected, print them
	if ((half_idx >= NUM_MEASUREMENTS) && (full_idx >= NUM_MEASUREMENTS))
	{
		// Print half-buffer WCET measurements
		printf("Half Buffer WCET Measurements (in timer ticks):\r\n");
		for (int i = 0; i < NUM_MEASUREMENTS; i++)
		{
			printf("%lu, ", wcet_half_buffer[i]);
		}
		printf("\r\n");

		// Print full-buffer WCET measurements
		printf("Full Buffer WCET Measurements (in timer ticks):\r\n");
		for (int i = 0; i < NUM_MEASUREMENTS; i++)
		{
			printf("%lu, ", wcet_full_buffer[i]);
		}
		printf("\r\n");

		// Reset indices if you wish to start over (or halt further measurements)
		half_idx = 0;
		full_idx = 0;
	}

	if(LCD_UPDATE_FLAG)
	{

		LCD_UPDATE_FLAG=0;
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

/**
 * DMA calls this ISR when half full and converts integer ADC values to floating point
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
if(!HALF_BUFFER_FULL_FLAG)
{
	for(int i=0;i<HALF_BUFFER_SIZE;i++)
		{
			fft_buffer_in_1[i] = ((float)adc_buffer[i] / 4095.0f) * 3.3f;
		}
		HALF_BUFFER_FULL_FLAG=1;
}


}
/**
 * DMA calls this ISR when full and converts integer ADC values to floating point
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

if(!FULL_BUFFER_FULL_FLAG)
	{
		for(int i=HALF_BUFFER_SIZE;i<BUFFER_SIZE;i++)
		{
		fft_buffer_in_2[i-HALF_BUFFER_SIZE] = ((float)adc_buffer[i] / 4095.0f) * 3.3f;
		}
		FULL_BUFFER_FULL_FLAG=1;
	}
}



void TIM8_UP_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim8);
//	LCD_UPDATE_FLAG=1;

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
