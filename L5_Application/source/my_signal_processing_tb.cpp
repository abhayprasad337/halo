/*
 * my_signal_proecessing_tb.cpp (SIGNAL PROCESSING TOOL BOX)
 *
 * ABSTRACT:
 * 	This is a part of the halo project. This tool box will be used to process the data
 * 	that has been logged from the accelerometer.
 *
 *  Created on: Apr 30, 2017
 *
 *  Author: ABHAY PRASAD (abhayprasad.337[at]gmail.com)
 */

#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "motion_analysis.hpp"

/**
 * ABSTRACT: This function performs the function of a moving average filter.
 * 			  The mean of values specified by the window size is used to obtain the
 * 			  output of the kernel.
 *
 * INPUTS: in - pointer to the array containing data.
 * 		   window_size - number of elements in the input array (in).
 * 		   val - value to append to the array.
 *
 * OUTPUTS: None.
 *
 * RETURNS: Output of the kernel after performing moving averaging.
 * 			NOTE: Input array has also been appended with the new value.
 */
int16_t moving_average_filter (int16_t *in, int16_t val, uint16_t window_size){

	/*
	 * Shift the array and append values.
	 */
	 memmove(&in[1],&in[0],sizeof(int16_t)*(window_size-1));
	 in[0] = val;

	 /*
	  * Compute average after appending new value to the array
	  * and removing last value.
	  */
	double sum = 0;
	for (uint16_t i = 0; i < window_size; i++){
		sum += in[i];
	}
	return (int16_t)(sum/window_size);
}


/**
 * ABSTRACT : - This function is used to perform calibration of the
 * 			  	device during start up.
 * 			  - N values are recorded after booting and the mean of them
 * 			  	is considered to 0 offset the value.
 */
int16_t calibration (int16_t *in, uint16_t cal_length){
	double sum =0;
	for (uint16_t i=0;i < cal_length; i++){
		sum += in[i];
	}
	return (int16_t)(sum/cal_length);
}


