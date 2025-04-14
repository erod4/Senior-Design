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
#include "spi.h"
#include "display.h"
#include <stdio.h>
#include <sys_clk_config.h>
#include <timer.h>
#include <UART.h>
#include <math.h>
#include <inttypes.h>
#include <stdlib.h>


#define BUFFER_SIZE 		64
#define HALF_BUFFER_SIZE 	32
#define FFT_BUFFER_SIZE 	32
/////////////////////////////////////
float32_t values[] = {
    0.0f, 0.22955625f, 0.45587102f, 0.67576297f, 0.88616987f, 1.08420518f,
    1.26721117f, 1.43280753f, 1.57893444f, 1.70388939f, 1.80635686f, 1.88543044f,
    1.94062683f, 1.97189157f, 1.97959636f, 1.96452797f, 1.92786913f, 1.87117169f,
    1.79632266f, 1.70550375f, 1.60114541f, 1.48587611f, 1.36246805f, 1.2337803f,
    1.10270061f, 0.97208699f, 0.84471031f, 0.72319908f, 0.6099874f, 0.50726735f,
    0.41694655f, 0.3406119f
};

uint32_t adc_buffer				[BUFFER_SIZE];

float32_t fft_buffer_out_1		[FFT_BUFFER_SIZE];
float32_t fft_buffer_in_1		[FFT_BUFFER_SIZE];

float32_t fft_buffer_out_2		[FFT_BUFFER_SIZE];
float32_t fft_buffer_in_2		[FFT_BUFFER_SIZE];


float32_t iir_buffer_in_1		[FFT_BUFFER_SIZE/2];
float32_t iir_buffer_in_2		[FFT_BUFFER_SIZE/2];

int32_t iir_buffer_in_1_int		[FFT_BUFFER_SIZE/2];
int32_t iir_buffer_in_2_int		[FFT_BUFFER_SIZE/2];


int32_t iir_buffer_out_1		[FFT_BUFFER_SIZE/2];
int32_t iir_buffer_out_2		[FFT_BUFFER_SIZE/2];

int32_t lcd_buffer_1			[FFT_BUFFER_SIZE/2];
int32_t lcd_buffer_2			[FFT_BUFFER_SIZE/2];

volatile uint8_t HALF_BUFFER_FULL_FLAG							=	0;
volatile uint8_t FULL_BUFFER_FULL_FLAG							=	0;



static void MX_GPIO_Init(void);




int main(void)
{


  HAL_Init();

  SystemClock_Config();
  SPI1_Init();
  spi_init();

  LCD_reset();

  HAL_Delay(100);

  LCD_init();



  uart_init();
  init_fft(FFT_BUFFER_SIZE);

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
//  MX_TIM8_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
//  MX_DAC1_Init();

  HAL_ADC_Start_DMA(&hadc1,adc_buffer,BUFFER_SIZE);
//  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,dac_buffer,BUFFER_SIZE,DAC_ALIGN_12B_R);
//  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_Base_Start(&htim7);
  HAL_TIM_Base_Start(&htim6);

  LCD_Fill_Color(0x0101);
  LCD_Fill_Color(0x1111);
  LCD_Fill_Color(0x5511);

//  drawBackground(0x00,0x00);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////



  while (1)
  {
	if(HALF_BUFFER_FULL_FLAG)
	{
		// Get start time from Timer8

		//Once converted to floating point the FFT is performed on the first half of buffer
		perform_fft(values, fft_buffer_out_1);

		//Take absolute magnitude of FFT output
		arm_cmplx_mag_f32(fft_buffer_out_1,iir_buffer_in_1,(uint32_t)(FFT_BUFFER_SIZE/2));

		convertFFTMagArrayToInt(iir_buffer_in_1,iir_buffer_in_1_int,FFT_BUFFER_SIZE/2,MAX_SCREEN_HEIGHT);
		//Do IIR filtering for each FFT value
		for(int i=0;i<FFT_BUFFER_SIZE/2;i++)
				{
					iir_buffer_out_1[i]=lowpass_FIR_IIR_filter(iir_buffer_in_1_int[i]);

				}
		//Lastly store in a LCD screen buffer
		for(int i=0; i<FFT_BUFFER_SIZE/2;i++)
		{
			lcd_buffer_1[i]=iir_buffer_out_1[i];


		}
//		  drawBackground(0xff,0xff);

		updateSpectrumDisplay(lcd_buffer_1, FFT_BUFFER_SIZE/2, 0xff, 0xff);
		HALF_BUFFER_FULL_FLAG=0;

	}

	//Once converted to floating point the FFT is performed on the second half of buffer
	if(FULL_BUFFER_FULL_FLAG)
	{


		perform_fft(values, fft_buffer_out_2);

		//Take absolute magnitude of FFT output
		arm_cmplx_mag_f32(fft_buffer_out_2,iir_buffer_in_2,(uint32_t)(FFT_BUFFER_SIZE/2));
		convertFFTMagArrayToInt(iir_buffer_in_2,iir_buffer_in_2_int,FFT_BUFFER_SIZE/2,MAX_SCREEN_HEIGHT);


		//Do IIR filtering for each FFT value
		for(int i=0;i<FFT_BUFFER_SIZE/2;i++)
		{
			iir_buffer_out_2[i]=lowpass_FIR_IIR_filter(iir_buffer_in_2_int[i]);

		}

		//Lastly store in a LCD screen buffer
		for(int i=0; i<FFT_BUFFER_SIZE/2;i++)
		{
			lcd_buffer_2[i]=iir_buffer_out_2[i];

		}
		updateSpectrumDisplay(lcd_buffer_2, FFT_BUFFER_SIZE/2, 0xff, 0xff);
//		  drawBackground(0x00,0x00);

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


  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();


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
;
