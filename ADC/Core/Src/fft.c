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








