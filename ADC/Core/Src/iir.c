#include "iir.h"
#include <stdint.h>
#include <math.h>
int lowpass_FIR_IIR_filter(int input)
{
	static long filter_reg;			//32-bit register
	long filter_reg_store;			//Temp storage

	filter_reg_store=filter_reg;	//temp store last register val

	//Update register with the current input sample.
	filter_reg=filter_reg-(filter_reg>>FILTER_SHIFT)+input;
	//Update the FIR section and scale output .
	return ((int)((filter_reg+filter_reg_store)>>(FILTER_SHIFT+1)));


}
void convertFFTMagArrayToInt(const float *input, int32_t *output, uint32_t length, uint16_t displayHeight)
{
    // Find the maximum magnitude in the array.
    float maxVal = 0.0f;
    for (uint32_t i = 0; i < length; i++)
    {
        if (input[i] > maxVal)
        {
            maxVal = input[i];
        }
    }

    // Avoid division by zero.
    if (maxVal <= 0.0f)
    {
        for (uint32_t i = 0; i < length; i++)
        {
            output[i] = 0;
        }
        return;
    }

    // Compute the scaling factor such that maxVal maps to displayHeight.
    float scale = (float)displayHeight / maxVal;

    // Convert each float value to an integer and clamp to [0, displayHeight].
    for (uint32_t i = 0; i < length; i++)
    {
        int32_t scaledVal = (int32_t)roundf(input[i] * scale);
        if (scaledVal < 0)
        {
            scaledVal = 0;
        }
        else if (scaledVal > displayHeight)
        {
            scaledVal = displayHeight;
        }
        output[i] = scaledVal;
    }
}
