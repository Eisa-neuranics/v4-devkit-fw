/*
 * ECG_FILTER.c
 *
 *  Created on: 10 Sep 2023
 *      Author: nea79
 */

#include "stm32wbxx_hal.h"
#include "main.h"
#include "IIR_Filter.h"
#include "kernel.h"
#include "LSM6DSLTR.h"
#include "AFE.h"
#include <stdio.h>          // printf()
#include <math.h>


#define 	SAMPLE_RATE 1067.0
#define 	PI 3.14159265358979323846

//--------------------------------------------------------------------------------------------
//-------------------------640 SPS---SCG------------------------------------------------------
//--------------------------------------------------------------------------------------------

// IIR-HPF-Butt-2nd-7Hz
//const float b[3]		=	{ 0.9526f 	,	-1.9051f     ,	0.9526f };
//const float a[3]		=	{ 1.0000f	,	-1.9029f	 ,	0.9074f	};

// IIR-BPF-Butt-2nd-7-45Hz
//const float b[3]		=	{ 0.1588f 	,	0     		,	-0.1588f    };
//const float a[3]		=	{ 1.0000f	,	-1.6567f	,	 0.6825f	};

// IIR-BPF-Butt-2nd-7-100Hz
//const float b[3]		=	{ 0.2575f 	,	0     		,	-0.2575f    };
//const float a[3]		=	{ 1.0000f	,	-1.4462f	,	 0.4850f	};

// IIR Notch Filter-Notching(in Matlab)-2nd-Fc=50
//const float b_N50[3]	=	{ 0.9622f	,	-1.6985f	,	0.9622f	};
//const float a_N50[3]	=	{ 1.0000f	,	-1.6985f	,	0.9244f	};

// IIR Notch Filter-Notching(in Matlab)-2nd-Fc=60
//const float b_N60[3]	=	{ 0.9622f	,	-1.6013f	,	0.9622f	};
//const float a_N60[3]	=	{ 1.0000f	,	-1.6013f	,	0.9244f	};

// IIR Notch Filter-Notching(in Matlab)-2nd-Fc=50-60
//const float b_N50_60[3]	=	{ 0.9186f	,	-1.5820f	,	0.9186f	};
//const float a_N50_60[3]	=	{ 1.0000f	,	-1.5820f	,	0.8372f	};

//--------------------------------------------------------------------------------------------
//-------------------------640 SPS---MMG------------------------------------------------------
//--------------------------------------------------------------------------------------------

// IIR-BPF-Butt-2nd-7-100Hz
//const float b[3]		=	{ 0.8708f 	,	0     		,	-0.8708f    };
//const float a[3]		=	{ 1.0000f	,	0.0864f		,	 0.7417f	};

// IIR Notch Filter-Notching(in Matlab)-2nd-Fc=50-60
//const float b_N50_60[3]	=	{ 0.9103f	,	-1.5692f	,	0.9103f	};
//const float a_N50_60[3]	=	{ 1.0000f	,	-1.5692f	,	0.8207f	};

//--------------------------------------------------------------------------------------------
//------------------------- 1067 SPS----------------------------------------------------------
//--------------------------------------------------------------------------------------------

// 7-100 Hz
//const float b[3]		=	{ 0.2193f 	,	0     		,	-0.2193f    };
//const float a[3]		=	{ 1.0000f	,	-1.5420f	,	 0.5614f	};

// 1-300 Hz
//const float b[3]		=	{ 0.5476f 	,	0     		,	-0.5476f    };
//const float a[3]		=	{ 1.0000f	,	-0.8983f	,	 0.0952f	};

// 1-300 Hz
const float b[5]		=	{ 0.3508f ,	 0     	   ,	-0.7016f ,	 0  		,	 0.3508f  	};
const float a[5]		=	{ 1.000f  ,	-1.7645f   ,	 0.7211f ,	-0.1369f  	,	 0.1804f  	};


// 10-300 Hz
//const float b[5]		=	{ 0.3342f ,	 0     	   ,	-0.6683f ,	 0  		,	 0.3342f  	};
//const float a[5]		=	{ 1.000f  ,	-1.7121f   ,	 0.7193f ,	-0.1785f  	,	 0.1761f  	};

//const float b[7]		=	{ 0.2183f ,	 0     	   ,	-0.6549f ,	 0  		,	 0.6549f  	, 0 		 	,   -0.2183f };
//const float a[7]		=	{ 1.000f  ,	-2.6326f   ,	 2.2821f ,	-0.9974f  	,	 0.6464f  	,  -0.2660f 	,	-0.0325f };



// IIR Notch Filter-Notching(in Matlab)-2nd-Fc=50
//const float b_N50_60[3]	=	{ 0.9714f	,	-1.8600f	,	0.9714f	};
//const float a_N50_60[3]	=	{ 1.0000f	,	-1.8600f	,	0.9428f	};

// 4th order 45-65Hz
const float b_N50_60[5]		=	{ 0.9201f ,	-3.4951f   ,	5.1593f ,	 -3.4951f  ,	0.9201f  	 };
const float a_N50_60[5]		=	{ 1.000f  ,	-3.6408f   ,	5.1529f ,	 -3.3494f  ,	0.8466f  	 };


//const float b_N50_60[7]		=	{ 0.8888f ,	-5.0645f   ,	12.2856f ,	 -16.2190f  ,	12.2856f  	,  -5.0645f 	,    0.8888f };
//const float a_N50_60[7]		=	{ 1.000f  ,	-5.4744f   ,	12.7618f ,	 -16.1955f  ,	11.7971f  	,  -4.6782f 	,	 0.7900f };

//------------------------------------------------------------------------------------------

extern	tsSIG 	ECG;
extern  tsSIG 	MCG;
extern	tsCMD	tsCmd;

//--------------------------------------------------------------------------------------------


// Function to initialize Bandpass Filter
void init_combined_filter( Filter *filter, double low_fc, double high_fc, double notch_fc )
{
    // Bandpass Filter initialization
    double nyquist = SAMPLE_RATE / 2.0;
    double low = low_fc / nyquist;
    double high = high_fc / nyquist;

    double W0 = 2 * PI * (high + low) / 2.0;  // Center frequency
    double BW = high - low;                  // Bandwidth
    double Q = W0 / BW;                      // Quality factor

    // Bandpass filter coefficients for a second-order Butterworth filter (simplified)
    filter->b_bpf[0] = BW / 2.0;
    filter->b_bpf[1] = 0.0;
    filter->b_bpf[2] = -BW / 2.0;

    filter->a_bpf[0] = 1.0 + BW / Q;
    filter->a_bpf[1] = -2.0 * cos(W0);
    filter->a_bpf[2] = 1.0 - BW / Q;

    // Initialize bandpass filter state to zero
    filter->z_bpf[0] = filter->z_bpf[1] = filter->z_bpf[2] = 0.0;

    // Notch Filter initialization
    double W0_notch = 2 * PI * notch_fc / nyquist;
    double BW_notch = 5.0;  // Bandwidth of the notch (adjustable)

    // Notch filter coefficients (second-order IIR)
    filter->b_notch[0] = 1.0;
    filter->b_notch[1] = -2.0 * cos(W0_notch);
    filter->b_notch[2] = 1.0;

    filter->a_notch[0] = 1.0 + BW_notch;
    filter->a_notch[1] = -2.0 * cos(W0_notch);
    filter->a_notch[2] = 1.0 - BW_notch;

    // Initialize notch filter state to zero
    filter->z_notch[0] = filter->z_notch[1] = filter->z_notch[2] = 0.0;
}


// Combined function to apply both Bandpass and Notch Filter to a single data sample
int32_t apply_combined_filter(Filter *filter, int32_t input)
{
    // Convert int32_t input to double for filter processing
    double x = (double)input;

    // Apply Bandpass Filter (BPF)
    double bpf_output = filter->b_bpf[0] * x + filter->z_bpf[0];
    filter->z_bpf[0] = filter->b_bpf[1] * x + filter->z_bpf[1] - filter->a_bpf[1] * bpf_output;
    filter->z_bpf[1] = filter->b_bpf[2] * x + filter->z_bpf[2] - filter->a_bpf[2] * bpf_output;
    filter->z_bpf[2] = x;

    // Apply Notch Filter (NFB)
    double notch_output = filter->b_notch[0] * bpf_output + filter->z_notch[0];
    filter->z_notch[0] = filter->b_notch[1] * bpf_output + filter->z_notch[1] - filter->a_notch[1] * notch_output;
    filter->z_notch[1] = filter->b_notch[2] * bpf_output + filter->z_notch[2] - filter->a_notch[2] * notch_output;
    filter->z_notch[2] = bpf_output;

    // Convert filtered output back to int32_t (with rounding)
    return (int32_t)round(notch_output);
}

//--------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------
//								IIR Bandpass Filter
//--------------------------------------------------------------------------------------------
int32_t IIR_AFE ( tsSIG *Signal, uint8_t Temp_xyptr )
{

	float  		convM = 0;
	float  		conv1M = 0, conv2M = 0;
	uint8_t 	o = 0;


	// BPF Implementation ------------------------------------------------------------------------------------
	if (tsCmd.TMR_HPF)
	{
		for ( uint8_t i = 0 ; i < BPF_order ; i++ )
		{
			o= RecentValueIndex ( Temp_BUF_LEN, Temp_xyptr, i );					// Get buffer position for past values
			conv1M +=  b[i] * Signal->Raw[o];
		}

		o=0;

		for ( uint8_t i = 1 ;  i < BPF_order ; i++ )
		{
			o = RecentValueIndex ( Temp_BUF_LEN, Temp_xyptr, i );					// Get buffer position for past values
			if (tsCmd.TMR_N50) {conv2M += a[i] * Signal->HPF [o];} else {conv2M += a[i] * Signal->Filtered [o];}
		}

		Signal->HPF [Temp_xyptr] =  conv1M -  conv2M ;
	}

	// Notch Filter implementation ---------------------------------------------------------------------------
	if (tsCmd.TMR_N50)
	{
		conv1M= 0;		conv2M= 0;

		// 50 Notch Filter Implementation
		for ( uint8_t i=0 ; i < NF_order ; i++ )
		{
			o = RecentValueIndex ( Temp_BUF_LEN, Temp_xyptr, i );					// Get buffer position for past values
			if (tsCmd.TMR_HPF) { conv1M +=  b_N50_60[i] * Signal->HPF[o]; } else { conv1M +=  b_N50_60[i] * Signal->Raw[o]; }
		}

		o=0;

		for ( uint8_t i=1 ; i < NF_order ; i++ )
		{
			o=RecentValueIndex ( Temp_BUF_LEN, Temp_xyptr, i );					// Get buffer position for past values
			 conv2M += a_N50_60[i] * Signal->Filtered[o] ;
		}

		convM= conv1M - conv2M;
	}

	if (tsCmd.TMR_HPF == false && tsCmd.TMR_N50 == false)
	{
		return Signal->Raw[Temp_xyptr];
	}
	else
	{
		return  (int)convM; //(int)conv;
	}

}

//--------------------------------------------------------------------------------------------
//						Get buffer position for past values
//--------------------------------------------------------------------------------------------
uint8_t RecentValueIndex ( uint8_t BufferLen, uint8_t BufferPos, uint8_t in_dex )
{
	uint8_t out_xy = 0;

	if ( ( BufferPos - in_dex ) <  0 )	{ out_xy = ( BufferLen + BufferPos ) - in_dex; }	// Calculate pointer of the circular buffer for previous values
	if ( ( BufferPos - in_dex ) >= 0 )	{ out_xy = BufferPos - in_dex; }				// Calculate pointer of the circular buffer for previous values

	return out_xy;
}



