/*******************************************************************************
 * @file    BLE.c
 * @author  Eisa Aghchehli
 * @brief   MCG-Rev4 BLE controller.
 * @Date	21/06/2024
  ******************************************************************************/

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
	#include "app_ble.h"
#endif


//-------------------------------------------------------------------------------------------
			teBLE_STATE 		eBLE_STATE = BLE_STATE_RESET;
 extern 	tsBLE				tsBle;
 extern		tsIMU				IMU;
 extern		tsSIG 				TMR1;
 extern		tsSIG 				TMR2;
 extern		tsSIG 				EMG;
 extern		tsSTREAM			tsStreamAFE;
 extern		tsSTREAM			tsStreamIMU;
 extern 	tsCMD				tsCmd;
 extern 	tsDEBUG				tsDebug;
 extern		tsTIMER				tsTimer;

//--------------------------------------------------------------------------------------------

 uint16_t u16temp;
//------------------------------------------------------------------------------------------//
//                                        Initialisation                                    //
//------------------------------------------------------------------------------------------//
 void BLE_Init (void)
 {

	 tsBle.App.AccelNotification = false;
	 tsBle.App.GyroNotification = false;
	 tsBle.App.AfeNotification = false;

	 DIAG ("\t\tBLE initialised\r\n");
	 HAL_Delay (10);
 }

 //**********************************************************************************************
 //**********************************************************************************************
 //                                        ADS1293 control                                     //
 //**********************************************************************************************
 //**********************************************************************************************
 void BLE_Control ( void )
 {

	 switch( eBLE_STATE )
	 {
// ---------------------------------------------

	 case BLE_STATE_RESET:

		 eBLE_STATE = BLE_STATE_INIT;
		 break;
// ---------------------------------------------

	 case BLE_STATE_INIT:

		 eBLE_STATE = BLE_STATE_ADV;
		 break;
// ---------------------------------------------

	 case BLE_STATE_ADV:

		 eBLE_STATE = BLE_STATE_CHECK_CONNECTION;
		 break;
// ---------------------------------------------

	 case BLE_STATE_CHECK_CONNECTION:

		 if ( tsBle.Connected )
		 {
			 #if (L2CAP_REQUEST_NEW_CONN_PARAM != 0 )
			 	 //BLE_SVC_L2CAP_Conn_Update(0x00);
			 #endif
			 eBLE_STATE = BLE_STATE_XFER_SCG;
		 }
		 else
		 {
			 eBLE_STATE = BLE_STATE_CHECK_CONNECTION;
		 }
		 break;
// ---------------------------------------------

	 case BLE_STATE_CONNECTED:
		 if ( tsBle.Connected )
		 {
			 eBLE_STATE = BLE_STATE_XFER_SCG;
		 }
		 else
		 {
			 eBLE_STATE = BLE_STATE_ADV;
		 }
		 break;
// ---------------------------------------------

	 case BLE_STATE_XFER_SCG:

		 if ( tsBle.App.AfeNotification )
		 {
			 if ( tsStreamAFE.bReleaseAfeData == true )
			 {
				 tsStreamAFE.bReleaseAfeData = false;
				 AFE_StreamDataPack ( tsStreamAFE.u8TMR1TxCpy, TMR1.u8TxLen,
						 	 	 	  tsStreamAFE.u8TMR2TxCpy, TMR2.u8TxLen,
									  tsStreamAFE.u8EMGTxCpy, EMG.u8TxLen,
									  tsStreamAFE.u8TxData );
				 // Reset data length
				 TMR1.u8TxLen = 0;
				 TMR2.u8TxLen = 0;
				 EMG.u8TxLen = 0;
				 // Set the task for BLE
				 tsBle.App.AfeSetTask = true;

				 eBLE_STATE = BLE_STATE_XFER_SCG_Cmplt;
			 }
			 else
			 {
				 eBLE_STATE = BLE_STATE_XFER_SCG;
			 }
		 }
		 else
		 {
			 eBLE_STATE = BLE_STATE_XFER_ACCEL;
		 }
		 break;
// ---------------------------------------------


	 case BLE_STATE_XFER_SCG_Cmplt:


		 if ( tsBle.AFE_XferComplete )
		 {
			 tsBle.AFE_XferComplete = false;

			 // By default the next step is waiting another SCG/MCG data, but if IMU is enabled, so ACCEL would be the next step.
			 eBLE_STATE = BLE_STATE_IDLE;
			 #ifdef IMU_ENABLE
			 	 eBLE_STATE = BLE_STATE_XFER_ACCEL;
			 #endif
		 }
		 else
		 {
			 eBLE_STATE = BLE_STATE_XFER_SCG_Cmplt;
		 }
		 break;
// ---------------------------------------------

	 case BLE_STATE_XFER_ACCEL:

		 if ( tsBle.App.AccelNotification )
		 {
			 if ( tsStreamIMU.bReleaseAccelData == true )
			 {
				 tsStreamIMU.bReleaseAccelData = false;
				 IMU_StreamDataPack ( Accelerometer, IMU.u8AxTxData, IMU.u8AyTxData, IMU.u8AzTxData, tsStreamIMU.u8TxData );
				 //Reset data length
				 IMU.u8AxTxLen = 0;
				 IMU.u8AyTxLen = 0;
				 IMU.u8AzTxLen = 0;
				 // Set the task for BLE
				 tsBle.App.AccelSetTask= true;

				 eBLE_STATE = BLE_STATE_XFER_ACCEL_Cmplt;
			 }
			 else
			 {
				 eBLE_STATE = BLE_STATE_XFER_ACCEL;
			 }

		 }
		 else
		 {
			 eBLE_STATE = BLE_STATE_IDLE;
		 }
		 break;
// ---------------------------------------------

	 case BLE_STATE_XFER_ACCEL_Cmplt:

		 if ( tsBle.ACCEL_XferComplete )
		 {
			 tsBle.ACCEL_XferComplete = false;

			 eBLE_STATE = BLE_STATE_IDLE;
			 #ifdef GYRO_ENABLE
			 	 eBLE_STATE = BLE_STATE_XFER_GYRO;
			 #endif
		 }
		 else
		 {
			 eBLE_STATE = BLE_STATE_XFER_ACCEL_Cmplt;
		 }
		 break;
// ---------------------------------------------

	 case BLE_STATE_XFER_GYRO:

		 tsStreamIMU.bReleaseGyroData = true;
		 if ( tsStreamIMU.bReleaseGyroData == true )
		 {
			 // Pack all the data
			 	IMU.u8GxTxLen = 0;
			 	IMU.u8GyTxLen = 0;
			 	IMU.u8GzTxLen = 0;

			 tsStreamIMU.bReleaseGyroData = false;
			 eBLE_STATE = BLE_STATE_XFER_GYRO_Cmplt;
		 }
		 else
		 {
			 eBLE_STATE = BLE_STATE_XFER_GYRO;
		 }
		 break;
// ---------------------------------------------

	 case BLE_STATE_XFER_GYRO_Cmplt:

		 tsBle.GYRO_XferComplete = true;
		 if ( tsBle.GYRO_XferComplete )
		 {
			 tsBle.GYRO_XferComplete = false;
			 eBLE_STATE = BLE_STATE_IDLE;
		 }
		 else
		 {
			 eBLE_STATE = BLE_STATE_XFER_GYRO;
		 }
		 break;
// ---------------------------------------------

	 case BLE_STATE_IDLE:

		 eBLE_STATE = BLE_STATE_CHECK_CONNECTION;
		 break;

	 case BLE_STATE_DISCONNECTED:

		 eBLE_STATE = BLE_STATE_ADV;
		 break;

// Unknown state--------------------------------

	 default:

		 eBLE_STATE = BLE_STATE_RESET;
		 break;

	 }
// ---------------------------------------------

 }


