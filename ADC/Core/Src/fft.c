/*
 * fft.c
 *
 *  Created on: Nov 4, 2024
 *      Author: enrique
 */

#include "fft.h"
#include "stm32g4xx_hal.h"
#include "arm_math.h"
#include "main.h"
#include <stdio.h>

arm_rfft_instance_q15 fft_instance;


void init_fft(uint16_t fft_length)
{
    arm_status status =arm_rfft_init_q15(&fft_instance, fft_length, 0, 1);
    if(status!=0)
    {
    	printf("Error initializing FFT\n\r");
    }
}






void perform_fft(q15_t* fft_buffer_in,q15_t* fft_buffer_out,uint16_t buffer_size)
{
	uint16_t len=2*buffer_size;
	q15_t buff_out[len];
	q15_t buff_out_temp[len];
	// Perform the FFT
    arm_rfft_q15(&fft_instance, fft_buffer_in, buff_out_temp);

    for(int i=0;i<len;i++)
    {
    	buff_out[i]=buff_out_temp[i]<<8;
    }


    // Compute the magnitude for each complex pair and store it in fft_buffer_out
    arm_abs_q15(buff_out, fft_buffer_out, buffer_size);




}
