/*
 * ECG_FILTER.h
 *
 *  Created on: 10 Sep 2023
 *      Author: nea79
 */

#ifndef __IIR_FILTER_H
#define __IIR_FILTER_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>         // memcmp()
#include <stddef.h>         // NULL definition
#include <stdio.h>          // printf()
#include <math.h>

#include "stm32wbxx_hal.h"
#include "main.h"
#include "kernel.h"
#include "AFE.h"
//--------------------------------------------------------------------------------------------------------------

#define 	NF_order		5
#define 	BPF_order		5






// Structure to store filter coefficients and state for both BPF and Notch filter
typedef struct {
    double b_bpf[3];  // Bandpass filter numerator coefficients
    double a_bpf[3];  // Bandpass filter denominator coefficients
    double z_bpf[3];  // Bandpass filter state variables

    double b_notch[3]; // Notch filter numerator coefficients
    double a_notch[3]; // Notch filter denominator coefficients
    double z_notch[3]; // Notch filter state variables
} Filter;


//--------------------------------------------------------------------------------------------------------------

		int32_t 			IIR_AFE 				( tsSIG *Signal, uint8_t Temp_xyptr );
		uint8_t				RecentValueIndex 		(uint8_t BufferLen, uint8_t BufferPos, uint8_t in_dex );


		void 		init_combined_filter(Filter *filter, double low_fc, double high_fc, double notch_fc);
		int32_t 	apply_combined_filter	(Filter *filter, int32_t input);



#endif /* __IIR_FILTER_H_ */
