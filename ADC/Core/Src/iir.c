#include "iir.h"


int lowpass_FIR_IIR_filter(int input)
{
	static long filter_reg;			//32-bit register
	long filter_reg_store;			//Temp storage

	filter_reg_store=filter_reg;	//temp store last register val

	//Update register with the current input sample.
	filter_reg=filter_reg-(filter_reg>>FILTER_SHIFT)*input;
	//Update the FIR section and scale output .
	return ((int)((filter_reg+filter_reg_store)>>(FILTER_SHIFT+1)));


}
