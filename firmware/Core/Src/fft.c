/*
 * fft.c
 *
 *  Created on: Nov 4, 2024
 *      Author: enrique
 */

#include "fft.h"

#include "stm32g4xx_hal.h"
#include <stdio.h>
#include "arm_math.h"
#include "main.h"



// FFT instance
arm_rfft_fast_instance_f32 S;
arm_rfft_instance_q15 T;
int16_t convert_adc_to_q15(uint16_t adc_val)
{
    // Center the ADC value: range becomes -2048 to 2047
    int16_t centered = (int16_t)adc_val - 2048;
    // Scale to Q15: multiply by 16
    // This maps -2048 -> -32768 and 2047 -> ~32752 (close to the maximum 32767)
    int16_t q15_val = centered * 16;
    return q15_val;
}

void init_q15_fft(void)
{
	arm_rfft_init_q15(&T, 32,0,1);
}

void q_15_fft(int16_t* psrc, int16_t* pdst)
{
	arm_rfft_q15(&T, psrc,pdst);
}

// Initialize FFT
void init_fft(uint16_t fft_length)
{
	arm_status status;
	status = arm_rfft_fast_init_f32(&S, fft_length);


	  if ( status != ARM_MATH_SUCCESS)
	  {
		printf("Error in FFT Init\r\n");
	  }
}

// Perform FFT
void perform_fft(float32_t* psrc, float32_t* pdst)
{


	arm_rfft_fast_f32(&S, psrc, pdst, 0);
}



void perform_ifft(float32_t* psrc, float32_t* pdst)
{
	arm_rfft_fast_f32(&S, psrc, pdst, 1);
}








