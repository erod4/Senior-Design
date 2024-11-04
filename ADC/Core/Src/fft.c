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

arm_rfft_instance_q15 fft_instance;


void init_fft(uint16_t fft_length)
{
    arm_rfft_init_q15(&fft_instance, fft_length, 0, 1);

}




void perform_fft(q15_t* fft_buffer_in,q15_t* fft_buffer_out,uint16_t buffer_size)
{
	uint16_t len=2*buffer_size;
	q15_t buff_out[len];

	// Perform the FFT
    arm_rfft_q15(&fft_instance, fft_buffer_in, buff_out);

    // Compute the magnitude for each complex pair and store it in fft_buffer_out
    arm_cmplx_mag_q15(buff_out, fft_buffer_out, buffer_size);


}
