//Calculate the current offset from the sensor

#include "../main.h"
#include <unistd.h>

int calc_current_offset(volatile unsigned long *h2p_lw_adc){
	int counter = 300, offset;
	double adc_sum = 0;

	//sample at 100hz for 3 seconds
	while(counter > 0){
		*(h2p_lw_adc) = 0; //write starts adc read
		adc_sum = *(h2p_lw_adc) + adc_sum; //read
		counter = counter - 1;
		usleep(10000);
	}
	
	offset = (int)(adc_sum/400);
	return offset;
}