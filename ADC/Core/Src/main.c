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
/////////////////////////////////////////////////////////////////
float32_t dummy_buffer[256] = {
    409.136597f, -1.520004f, -0.461070f, -0.014361f, -0.454691f, -0.026311f, -0.446092f, -0.034129f,
    -0.437714f, -0.037262f, -0.431940f, -0.036507f, -0.430396f, -0.033756f, -0.433518f, -0.031473f,
    -0.440396f, -0.032000f, -0.449069f, -0.036876f, -0.457041f, -0.046411f, -0.462034f, -0.059589f,
    -0.462602f, -0.074368f, -0.458556f, -0.088261f, -0.451029f, -0.099026f, -0.442133f, -0.105330f,
    -0.434368f, -0.107113f, -0.429912f, -0.105607f, -0.430000f, -0.102999f, -0.434569f, -0.101786f,
    -0.442275f, -0.104094f, -0.450895f, -0.111037f, -0.457925f, -0.122445f, -0.461316f, -0.136874f,
    -0.460056f, -0.152040f, -0.454454f, -0.165467f, -0.446049f, -0.175176f, -0.437169f, -0.180272f,
    -0.430291f, -0.181193f, -0.427295f, -0.179564f, -0.428979f, -0.177772f, -0.434787f, -0.178263f,
    -0.442995f, -0.182849f, -0.451190f, -0.192205f, -0.456962f, -0.205662f, -0.458582f, -0.221419f,
    -0.455503f, -0.237031f, -0.448508f, -0.250127f, -0.439491f, -0.259078f, -0.430920f, -0.263460f,
    -0.425129f, -0.264177f, -0.423652f, -0.263213f, -0.426799f, -0.263049f, -0.433556f, -0.265981f,
    -0.441867f, -0.273456f, -0.449253f, -0.285664f, -0.453464f, -0.301483f, -0.453161f, -0.318810f,
    -0.448281f, -0.335150f, -0.440068f, -0.348323f, -0.430684f, -0.357113f, -0.422637f, -0.361587f,
    -0.418029f, -0.363089f, -0.418006f, -0.363867f, -0.422364f, -0.366422f, -0.429665f, -0.372815f,
    -0.437621f, -0.384068f, -0.443739f, -0.399883f, -0.446055f, -0.418734f, -0.443656f, -0.438301f,
    -0.436974f, -0.456127f, -0.427640f, -0.470334f, -0.418026f, -0.480155f, -0.410569f, -0.486148f,
    -0.407074f, -0.490067f, -0.408222f, -0.494320f, -0.413326f, -0.501362f, -0.420571f, -0.512933f,
    -0.427512f, -0.529603f, -0.431770f, -0.550608f, -0.431701f, -0.574085f, -0.426871f, -0.597600f,
    -0.418174f, -0.618843f, -0.407580f, -0.636313f, -0.397557f, -0.649742f, -0.390388f, -0.660181f,
    -0.387504f, -0.669722f, -0.389071f, -0.680918f, -0.393959f, -0.696049f, -0.400036f, -0.716513f,
    -0.404785f, -0.742441f, -0.406009f, -0.772674f, -0.402457f, -0.805113f, -0.394163f, -0.837345f,
    -0.382442f, -0.867361f, -0.369505f, -0.894140f, -0.357809f, -0.918003f, -0.349362f, -0.940534f,
    -0.345135f, -0.964178f, -0.344774f, -0.991621f, -0.346676f, -1.025034f, -0.348438f, -1.065579f,
    -0.347519f, -1.113105f, -0.341918f, -1.166320f, -0.330744f, -1.223240f, -0.314413f, -1.281890f,
    -0.294470f, -1.341014f, -0.273166f, -1.400619f, -0.252635f, -1.462181f, -0.234314f, -1.528518f,
    -0.218371f, -1.603316f, -0.203543f, -1.690519f, -0.187314f, -1.793768f, -0.166408f, -1.916096f,
    -0.137411f, -2.060065f, -0.097293f, -2.228386f, -0.043691f, -2.425112f, 0.025370f, -2.657329f,
    0.112315f, -2.937573f, 0.221647f, -3.287472f, 0.362751f, -3.744215f, 0.555023f, -4.374178f,
    0.839422f, -5.306731f, 1.311207f, -6.834430f, 2.245728f, -9.801772f, 4.912558f, -18.118254f,
    61.381596f, -192.420532f, -7.632750f, 20.421156f, -4.016945f, 9.196051f, -2.897842f, 5.679187f,
    -2.356097f, 3.941942f, -2.041358f, 2.894098f, -1.842153f, 2.182505f, -1.711913f, 1.656375f,
    -1.627029f, 1.239185f, -1.573399f, 0.887366f, -1.541567f, 0.574183f, -1.525068f, 0.282370f
};


////////////////////////////////////////////////////////////////////

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

  HAL_ADC_Start_DMA(&hadc1,adc_buffer,BUFFER_SIZE);
//  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,dac_buffer,BUFFER_SIZE,DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_Base_Start(&htim7);
  HAL_TIM_Base_Start(&htim6);
  __HAL_TIM_SET_COUNTER(&htim6, 0);  // Reset Timer 6 counter to 0


  /////////////////////////////////////////////////////////////////////////////////////////////////////////

	//Once converted to floating point the FFT is performed on the first half of buffer
	perform_fft(dummy_buffer, fft_buffer_out_1);

	//Take absolute magnitude of FFT output
	arm_cmplx_mag_f32(fft_buffer_out_1,iir_buffer_in_1,(uint32_t)(FFT_BUFFER_SIZE/2));

	//Do IIR filtering for each FFT value
	for(int i=0;i<FFT_BUFFER_SIZE/2;i++)
	{
		printf("%d: %f\r\n",i,iir_buffer_in_1[i]);
		//round magnitudes to nearest integer
		iir_buffer_out_1[i]=lowpass_FIR_IIR_filter(iir_buffer_in_1[i]);
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	for(int i=0;i<FFT_BUFFER_SIZE/2;i++)
	{
		printf("%d: %f\r\n", i, iir_buffer_out_1[i]);
	}




  while (1)
  {

	if(HALF_BUFFER_FULL_FLAG)
	{
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

		//Measure WCET then make an ISR to have LCD update values at that time

		HALF_BUFFER_FULL_FLAG=0;
	}
	//Once converted to floating point the FFT is performed on the second half of buffer
	if(FULL_BUFFER_FULL_FLAG)
	{
		perform_fft(fft_buffer_in_2, fft_buffer_out_2);

		//Take absolute magnitude of FFT output
		arm_cmplx_mag_f32(fft_buffer_out_2,iir_buffer_in_2,(uint32_t)(FFT_BUFFER_SIZE/2));

		//Do IIR filtering for each FFT value
		for(int i=0;i<FFT_BUFFER_SIZE/2;i++)
		{
			iir_buffer_out_2[i]=lowpass_FIR_IIR_filter(iir_buffer_in_2[i]);
		}
		//Lastly store in a LCD screen buffer

		//Measure WCET then make an ISR to have LCD update values at that time

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
