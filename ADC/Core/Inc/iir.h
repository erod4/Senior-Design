#ifndef IIR_H_
#define IIR_H_

#define FILTER_SHIFT	4
#include <stdint.h>


/**
 * 
 */
int lowpass_FIR_IIR_filter(int input);
void convertFFTMagArrayToInt(const float *input, int32_t *output, uint32_t length, uint16_t displayHeight);
float lowPassFilter_1(float input);
float lowPassFilter_2(float input);


#endif
