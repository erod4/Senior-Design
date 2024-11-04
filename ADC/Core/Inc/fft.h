

#ifndef FFT_H_
#define FFT_H_
#include "stm32g4xx_hal.h"
#include "arm_math.h"
#include "main.h"




/**
 * Initializes Q15 fft
 */
void init_fft(uint16_t fft_length);

/**
 * Performs q15 fixed point fft and returns the magnitudes
 * @param fft_buffer_in, buffer of fft input data
 * @param fft_buffer_out, buffer storing magnitudes of fft output
 * @param buffer_size, length of input buffer
 */
void perform_fft(q15_t* fft_buffer_in, q15_t* fft_buffer_out,uint16_t buffer_size);

#endif /* INC_ADC_H_ */
