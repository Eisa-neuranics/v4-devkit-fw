/*
 * AFE.c
 *
 *  Created on: Jun 1, 2024
 *      Author: eisaa
 */


//Includes ------------------------------------------------------------------
#include <stdint.h>
#include <string.h>         // memcmp()
#include <stddef.h>         // NULL definition
#include <stdio.h>          // printf()

#include "stm32wbxx_hal.h"
#include "main.h"
#include "ADS1293.h"
#include "kernel.h"
#include "debug.h"
#include "hardware.h"
#include "IIR_Filter.h"
#include "LSM6DSLTR.h"
#include "AFE.h"
#include "timer.h"
#include "exti.h"

#ifdef WBxx_BLE
	#include "custom_app.h"
	#include "stm32_seq.h"
	#include <Bluetooth.h>
#endif

//-------------------------------------------------------------------------------------------
			teAFE_STATE 		eAFE_STATE = AFE_STATE_RESET;
 	 		tsADS				ADS;
 	 		tsSIG 				TMR1;
 	 		tsSIG 				TMR2;
 	 		tsSIG 				EMG;
 extern	 	tsBLE				tsBle;
			tsSTREAM			tsStreamAFE;
 extern		tsSTREAM			tsStreamIMU;
 extern 	tsCMD				tsCmd;
 extern 	tsDEBUG				tsDebug;
 extern		tsTIMER				tsTimer;
 extern 	tsAFE				AFE;

// extern 	UART_HandleTypeDef 	huart1;
 extern 	TIM_HandleTypeDef 	htim2;
 extern 	SPI_HandleTypeDef 	hspi1;




 Filter 	combined_filter;
 int 		num_samples = 3;

 float angle = 0.0f;
 const float step = 0.1f;
 const float TWO_PI = 6.28318530718f;



 bool 	ADS_DRDY_FLAG = false;

 // Functions -------------------------------------------------------------------------------



// Declaration ------------------------------------------------------------------------------
 uint8_t u8temp, u8TMR1DataLen, u8TMR2DataLen, u8EMGDataLen;
 uint8_t SpiRxData[10];
 uint8_t SpiTxData[10], Temp_buff_count, Output_buff_count;
 //------------------------------------------------------------------------------------------//
 //                                        Initialisation                                    //
 //------------------------------------------------------------------------------------------//
 void AFE_Init (void)
 {


	 // Initialize filters
	 init_combined_filter(&combined_filter, 10.0, 100.0, 50.0);

	 DIAG ("\t\tAFE initialised\r\n");
	 HAL_Delay (10);
 }

 //**********************************************************************************************
 //**********************************************************************************************
 //                                        ADS1293 control                                     //
 //**********************************************************************************************
 //**********************************************************************************************
 void AFE_Control ( void )
 {
 	  switch (eAFE_STATE)
 	  {

 	  	  default:
 		  	  #ifdef AFE_DEBUG_EN
 	  		  	  DIAG(">>> AFE STATE_DEFAULT\r\n");
 		  	  #endif

 	  	    eAFE_STATE = AFE_STATE_RESET;
 	  		break;
 //-----------------------------------------------------------------------------------------------

 	  	  case AFE_STATE_RESET:
 			  #ifdef AFE_DEBUG_EN
 	  		  	  DIAG(">>> AFE_RESET\r\n");
 			  #endif
 	  		  ENABLE_RSTB_ADS; 											// Pull RESET low
 	  		  HAL_Delay(5); 											// Wait for a brief period
 	  		  DISABLE_RSTB_ADS; 										// Release RESET
 	  		  HAL_Delay(25); 											// Wait for a brief period

 	  	      eAFE_STATE = AFE_STATE_INIT;
 	  		  break;
 //-----------------------------------------------------------------------------------------------
 	  	  case AFE_STATE_INIT:
 		  	  #ifdef AFE_DEBUG_EN
 	  		  	  DIAG(">>> AFE_INIT\r\n");
 			  #endif
 		  	  ADS1293_Init();

 		  	  eAFE_STATE = AFE_STATE_ID;
 	  		  break;

 //-----------------------------------------------------------------------------------------------
 	  	  case AFE_STATE_ID:
 		  	  #ifdef AFE_DEBUG_EN
 	  		  	  DIAG(">>> AFE_READ_DEVICE_ID\r\n");
 			  #endif

 	  		  u8temp = ADS1293_SPIReadReg(ADS1293_REVID_REG);
 			  #ifdef ADS_DEBUG_EN
 	  		  	  DIAG(">>> DEVICE_ID = [%X] \r\n", u8temp );
 	  		  	  HAL_Delay(1000);
        	  #endif

 	  		  if ( u8temp == ADS1293_ID )
 	  		  {
 	  			  eAFE_STATE= AFE_STATE_START ;		//ADS_STATE_START ADS_STATE_ID
 	  		  }
 	  		  else
 	  		  {
 	  			  eAFE_STATE= AFE_STATE_ERROR;
 	  		  }

 	  		  break;
 //-----------------------------------------------------------------------------------------------
 	  	  case AFE_STATE_START:
 			  #ifdef ADS_DEBUG_EN
 	  		  	  DIAG(">>> AFE_START_CONVERSION\r\n");
 			  #endif

 		  	  ADS1293_SPIWriteReg( ADS1293_CONFIG_REG, Stop );
 		  	  HAL_Delay(50);
 	  		  ADS1293_SPIWriteReg( ADS1293_CONFIG_REG, Start );
 	  		  HAL_Delay(50);

 	  		  eAFE_STATE = AFE_STATE_READ_DATA;
 	  		  break;
 //-----------------------------------------------------------------------------------------------
 	  	  case AFE_STATE_READ_DATA:


 	  		  // Check if DRDY pin is triggered, then start reading data.
 	  		  if (ADS.DRDY == true)
 	  		  {
				 #ifdef AFE_DEBUG_EN
						  DIAG(">>> AFE_READ_DATA\r\n");
				 #endif
				// Each time we sample from AFE, sample forom IMU too
 	  			  tsTimer.bImuFlag = true;

 	  			  ADS.DRDY = false;
 	  			  ADS1293_SPIStreamReadReg( SpiRxData, 9 );

 	  			  // Count SPS
// 	  			  AFE.u16SampleCount++;





 	  			  num_samples++;
 	  			  if ( num_samples > 5)
 	  			  {
 	  				  float value1 = sinf(angle);
 	  				  float value2 = cosf(angle);

 	  				  angle += step;
 	  				  if (angle > TWO_PI) angle -= TWO_PI; // Wrap around

 	  				  tsDebug.i16TxTMR1 = (int) (value1 * 1000);
 	  				  tsDebug.i16TxTMR2 = (int) (value2 * 1000);

 	  				  // Count SPS
 	 	  			  AFE.u16SampleCount++;
 	 	  			  num_samples = 0;
 	 	  			  eAFE_STATE = AFE_STATE_PROCESS_DATA;
 	  			  }
 	  			  else
 	  			  {
 	  				eAFE_STATE = AFE_STATE_READ_DATA;
 	  			  }








// 	  			  eAFE_STATE = AFE_STATE_PROCESS_DATA;
 	  		  }
 	  		  else
 	  		  {
 				  eAFE_STATE = AFE_STATE_READ_DATA;
 	  		  }
 	  		  break;
 //-----------------------------------------------------------------------------------------------
 	  	  case AFE_STATE_PROCESS_DATA:

 	  		  TMR1.Input = (int32_t)( (SpiRxData[1] << 16) | (uint16_t)(SpiRxData[2] << 8) | SpiRxData[3]);
 	  		  TMR1.Vin = ( 2.4 * ( ( (2.0 * (double)TMR1.Input ) / (double)ADC_MAX ) - 1.0 ) ) / (double)ADC_GAIN;
 	  		  TMR1.Raw[Temp_buff_count] = (int)(TMR1.Vin * 1000000);

 	  		  TMR2.Input = (int32_t)( (SpiRxData[4] << 16) | (uint16_t)(SpiRxData[5] << 8) | SpiRxData[6]);
 	  		  TMR2.Vin = ( 2.4 * ( ( (2.0 * (double)TMR2.Input ) / (double)ADC_MAX ) - 1.0 ) ) / (double)ADC_GAIN;
 	  		  TMR2.Raw[Temp_buff_count] = (int)(TMR2.Vin * 1000000);

 	  		  EMG.Input = (int32_t)( (SpiRxData[7] << 16) | (uint16_t)(SpiRxData[8] << 8) | SpiRxData[9]);
 	  		  EMG.Vin = ( 2.4 * ( ( (2.0 * (double)EMG.Input ) / (double)ADC_MAX ) - 1.0 ) ) / (double)ADC_GAIN;
 	  		  EMG.Raw[Temp_buff_count] = (int)(EMG.Vin * 1000000);

// 	  		  TMR1.Filtered [Temp_buff_count] = IIR_AFE ( &TMR1, Temp_buff_count);
// 	  		  TMR2.Filtered [Temp_buff_count] = IIR_AFE ( &TMR2, Temp_buff_count);
// 	  		  EMG.Filtered  [Temp_buff_count] = IIR_AFE ( &EMG, Temp_buff_count);


 	  		 	  		  TMR1.Filtered [Temp_buff_count] = tsDebug.i16TxTMR1;
 	  		 	  		  TMR2.Filtered [Temp_buff_count] = tsDebug.i16TxTMR2;
// 	  		 	  		  EMG.Filtered  [Temp_buff_count] = IIR_AFE ( &EMG, Temp_buff_count);



 	  		  TMR1.u8TxData[u8TMR1DataLen++] 	= ( (  		  TMR1.Filtered [Temp_buff_count] & 0xFF00 ) >> 8 );
 	  		  TMR1.u8TxData[u8TMR1DataLen++] 	= ( (uint8_t) TMR1.Filtered [Temp_buff_count] & 0x00FF );

 	  		  TMR2.u8TxData[u8TMR2DataLen++]   	= ( (  		  TMR2.Filtered [Temp_buff_count] & 0xFF00 ) >> 8 );
 	  		  TMR2.u8TxData[u8TMR2DataLen++]   	= ( (uint8_t) TMR2.Filtered [Temp_buff_count] & 0x00FF );

 	  		  EMG.u8TxData [u8EMGDataLen++]    	= ( (  		  EMG.Filtered [Temp_buff_count] & 0xFF00 ) >> 8 );
 	  		  EMG.u8TxData [u8EMGDataLen++]    	= ( (uint8_t) EMG.Filtered [Temp_buff_count] & 0x00FF );

 			  #ifdef DATA_DEBUG_EN
 				  if (tsCmd.TMR == true && tsCmd.EMG == true)
 				  {
// 					 tsDebug.i16TxTMR1 = TMR1.Filtered [Temp_buff_count];
// 					 tsDebug.i16TxTMR2 = TMR2.Filtered [Temp_buff_count];
// 					 tsDebug.i16TxEMG  = EMG.Filtered  [Temp_buff_count];
 				  }
 				  else if (tsCmd.TMR == true && tsCmd.EMG == false )
 				  {
//  					 tsDebug.i16TxTMR1 = TMR1.Filtered [Temp_buff_count];
//  					 tsDebug.i16TxTMR2 = TMR2.Filtered [Temp_buff_count];
//  					 tsDebug.i16TxEMG  = 0x00;
 				  }

 				  else if (tsCmd.TMR == false && tsCmd.EMG == true )
 				  {
  					 tsDebug.i16TxTMR1 = 0x00;
  					 tsDebug.i16TxTMR2 = 0x00;
  					 tsDebug.i16TxEMG  = EMG.Filtered [Temp_buff_count];
 				  }
 				 tsDebug.bReleaseAFEData= true;
 			  #endif


	  		  Temp_buff_count++;		if (Temp_buff_count  ==  Temp_BUF_LEN ) 	{ Temp_buff_count = 0; }

 	  		if ( u8TMR1DataLen >=  BLE_Data_Length )
 	  		{
 	  			memcpy ( tsStreamAFE.u8TMR1TxCpy, TMR1.u8TxData, 64 );
 	  			memcpy ( tsStreamAFE.u8TMR2TxCpy, TMR2.u8TxData, 64 );
 	  			memcpy ( tsStreamAFE.u8EMGTxCpy, EMG.u8TxData, 64 );


 	  			// Copy TMR data length
 	  			TMR1.u8TxLen = u8TMR1DataLen;
 	  			TMR2.u8TxLen = u8TMR2DataLen;
 	  			EMG.u8TxLen = u8EMGDataLen;

 	  			// Enable BLE data transmission
 	  			tsStreamAFE.bReleaseAfeData = true;
 	  			tsStreamIMU.bReleaseAccelData = true;
 	  			u8TMR1DataLen = 0;
 	  			u8TMR2DataLen = 0;
 	  			u8EMGDataLen = 0;
 	  		}

 	  		  eAFE_STATE = AFE_STATE_READ_DATA;
 	  		  break;
 //-----------------------------------------------------------------------------------------------
 	  	  case AFE_STATE_IDLE:
 			  #ifdef ADS_DEBUG_EN
 	  		  	  DIAG(">>> STATE_IDLE\r\n");
 			  #endif
 	  		  eAFE_STATE = AFE_STATE_IDLE;
 	  		  break;

 //-----------------------------------------------------------------------------------------------
 	  	  case AFE_STATE_ERROR:
 			  #ifdef ADS_DEBUG_EN
 	  		  	  DIAG(">>> STATE_ADS_ERROR\r\n");
      	 	  #endif
 	  		  eAFE_STATE = AFE_STATE_RESET;
 	  		  break;
 	  	  }
 }
 //***********************************************************************************************
 //***********************************************************************************************

 void AFE_StreamDataPack 		( uint8_t *TMR1Data, uint8_t TMR1DataLen,
		  	  	  	  	  	  	  uint8_t *TMR2Data, uint8_t TMR2DataLen,
								  uint8_t *EMGData , uint8_t EMGDataLen,
								  uint8_t *outputString )
 {
 	uint8_t index=0;

 	// Clear buffer
 	memset( tsStreamAFE.u8TxData, 0, sizeof(tsStreamAFE.u8TxData) );

//#ifdef IMU_ENABLE
 	outputString [index++] = 'S';
 	outputString [index++] = tsBle.Timestamp++;
 	outputString [index++] = ',';
 	outputString [index++] = 'C';
 	outputString [index++] = '0';
 	outputString [index++] = ',';
 	outputString [index++] = TMR1DataLen;
 	outputString [index++] = ',';
 	memcpy ( outputString + index, TMR1Data, TMR1DataLen );

 	index += TMR1DataLen;

 	outputString [index++] = ',';
 	outputString [index++] = 'C';
 	outputString [index++] = '1';
 	outputString [index++] = ',';
 	outputString [index++] = TMR2DataLen;
 	outputString [index++] = ',';
 	memcpy ( outputString + index, TMR2Data, TMR2DataLen );

 	index += TMR2DataLen;

 	outputString [index++] = ',';
 	outputString [index++] = 'C';
 	outputString [index++] = '2';
 	outputString [index++] = ',';
 	outputString [index++] = EMGDataLen;
 	outputString [index++] = ',';
 	memcpy ( outputString + index, EMGData, EMGDataLen );

 	index += EMGDataLen;

 	outputString [index++] = 'T';
 	outputString [index++] = 'M';
 	outputString [index++] = 'M';




//#endif

//#ifndef IMU_ENABLE
//	outputString [index++] = 'S';
//	outputString [index++] = 'N';
//	outputString [index++] = ',';
//	outputString [index++] = 'R';
//	outputString [index++] = 'E';
//	outputString [index++] = ',';
//	outputString [index++] = TMR1DataLen;
//	outputString [index++] = ',';
//
//	memcpy ( outputString + index, TMR1Data, TMR1DataLen );
//
//	index += TMR1DataLen;
//
//	outputString [index++] = ',';
//	outputString [index++] = 'R';
//	outputString [index++] = 'M';
//	outputString [index++] = ',';
//	outputString [index++] = TMR2DataLen;
//	outputString [index++] = ',';
//
//	memcpy ( outputString + index, TMR2Data, TMR2DataLen );
//#endif

 }

