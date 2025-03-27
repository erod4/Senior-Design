#ifndef IIR_H_
#define IIR_H_
#define FILTER_SHIFT	4
#define FILTER_SCALE   (1 << FILTER_SHIFT)        // 16 if FILTER_SHIFT=4
#define FILTER_SCALE_2 (1 << (FILTER_SHIFT + 1))  // 32 if FILTER_SHIFT=4
#include "arm_math.h"

/**
 * 
 */
float32_t lowpass_FIR_IIR_filter(float32_t input);

#endif
