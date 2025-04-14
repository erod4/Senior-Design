#include "iir.h"
#include <stdint.h>
#include <math.h>
int lowpass_IIR_filter(int input, long* filter_reg)
{
	long filter_reg_store = *filter_reg;

	// Update register with current input
	*filter_reg = *filter_reg - (*filter_reg >> FILTER_SHIFT) + input;

	// Return the filtered result
	return (int)((*filter_reg + filter_reg_store) >> (FILTER_SHIFT + 1));
}

float lowPassFilter_1(float input)
{
    static float filter_reg = 0.0f;   // Filter state
    float filter_reg_store = filter_reg;  // Store previous state

    // Update register with the current input sample using floating-point arithmetic.
    // The operation filter_reg - (filter_reg/16.0f) is equivalent to filter_reg * (15/16)
    filter_reg = filter_reg - (filter_reg / 16.0f) + input;
    // Return the average of the old and new state, scaled down to match the integer version.
    return (filter_reg + filter_reg_store) / 32.0f;
}
float lowPassFilter_2(float input)
{
    static float filter_reg = 0.0f;   // Filter state
    float filter_reg_store = filter_reg;  // Store previous state

    // Update register with the current input sample using floating-point arithmetic.
    // The operation filter_reg - (filter_reg/16.0f) is equivalent to filter_reg * (15/16)
    filter_reg = filter_reg - (filter_reg / 16.0f) + input;
    // Return the average of the old and new state, scaled down to match the integer version.
    return (filter_reg + filter_reg_store) / 32.0f;
}
