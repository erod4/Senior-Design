

#ifndef FFT_H_
#define FFT_H_
#include "stm32g4xx_hal.h"

#include "arm_math.h"
#include "main.h"


void init_q15_fft(void);
void q_15_fft(int16_t* psrc, int16_t* pdst);
int16_t convert_adc_to_q15(uint16_t adc_val);
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
void perform_fft(float32_t* psrc,float32_t * pdst);

/**
 * @brief Performs an Inverse Fast Fourier Transform (IFFT) on the input data.
 * @param[in]  psrc  Pointer to the input array containing frequency-domain samples.
 * @param[out] pdst  Pointer to the output array for time-domain samples.
 */
void perform_ifft(float32_t* psrc, float32_t* pdst);



void q_15_fft_init(uint16_t fft_length);


#endif /* INC_ADC_H_ */
