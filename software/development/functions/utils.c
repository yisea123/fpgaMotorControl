#include "../main.h"
#include <unistd.h>
#include <sys/time.h>


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
	
	offset = (int)(adc_sum/300);
	return offset;
}

uint32_t createMask(uint32_t startBit, int num_bits)
{
   uint32_t  mask;
	mask = ((1 << num_bits) - 1) << startBit;
   return mask;
}

uint32_t createNativeInt(uint32_t input, int size)
{
	int32_t nativeInt;
	const int negative = ((input & (1 << (size - 1))) != 0);
	if (negative)
		  nativeInt = input | ~((1 << size) - 1);
	else 
		  nativeInt = input;	
	return nativeInt;
}


uint64_t GetTimeStamp() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}
