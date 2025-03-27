#include "iir.h"
#include "arm_math.h"

float32_t lowpass_FIR_IIR_filter(float32_t input)
{
    // 32-bit float accumulator
    static float filter_reg = 0.0f;
    float filter_reg_store = filter_reg;

    // filter_reg = filter_reg - (filter_reg / 2^FILTER_SHIFT) + input
    //            = filter_reg*(1 - 1/16) + input  (if FILTER_SHIFT=4)
    filter_reg = filter_reg
                 - (filter_reg / FILTER_SCALE)
                 + input;

    // output = (filter_reg + filter_reg_store) / 2^(FILTER_SHIFT+1)
    return (filter_reg + filter_reg_store) / (float)FILTER_SCALE_2;
}
