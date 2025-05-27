/*******************************************************************************
 * @file    kernel.c
 * @author  Eisa Aghchehli
 * @brief   ECG-MCG kernel
 *
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


#ifdef	WB55_USB
	#include "usbd_cdc_if.h"
	#include "usb_device.h"
#endif

#ifdef WBxx_BLE
	#include "custom_app.h"
	#include "stm32_seq.h"
	#include <Bluetooth.h>
#endif

/*
const char TX_CMD__WRITE_TO_SERVER[3]= { "WS," };					// "WS,"	87, 83, 44, 0
const char TX_CMD__READ_FROM_SERVER[3] = { "RS," };					// "RS,"	 82, 83, 44, 0
const char TX_CMD__CLIENT_NOTIFICATION[3] = { "CN," };				// "CN,"	67, 78, 44, 0
const char TX_CMD__SERVER_NOTIFICATION[3] = { "SN," };				// "SN,"	83, 78, 44, 0

const char TX_SUB_CMD__REAL_TIME_ECG[3] = { "RE," };				// "RE,"	82, 69, 44, 00
const char TX_SUB_CMD__REAL_TIME_MCG[3] = { "RM," };				// "RM,"	82, 77, 44, 00
const char COMMA[1] = { "," };
*/

// typedef -------------------------------------------------------------------
 teSYS_STATE 	eSYS_STATE = SYS_STATE_INIT;


 	//USBD_HandleTypeDef hUsbDeviceFS;
 	 	tsDataPacket 	sDataPacket_ECG;
		tsDataPacket 	sDataPacket_MCG;
extern	tsBLE			tsBle;
		tsCMD			tsCmd;
		tsSYSTEM		tsSystem;
extern	tsTIMER			tsTimer;
	   	tsLED			LedRGB;

extern	I2C_HandleTypeDef 	hi2c1;

// Defines -------------------------------------------------------------------




// variables -----------------------------------------------------------------


uint8_t ReadAddr, ReadData, WriteAddr, WriteData;
uint8_t DBG_TX[64];

// functions prototypes-------------------------------------------------------


// Functions Definition ------------------------------------------------------


//------------------------------------------------------------------------------------------//
//                                        Initialisation                                    //
//------------------------------------------------------------------------------------------//
void Kernel_Init (void)
{
	SET_RGB_COLOR(GREEN);
	HAL_Delay(250);
	SET_RGB_COLOR(OFF);
	HAL_Delay(250);
	SET_RGB_COLOR(GREEN);
	HAL_Delay(250);
	SET_RGB_COLOR(OFF);
	HAL_Delay(250);
	SET_RGB_COLOR(GREEN);



	tsCmd.MODE		= Start;
	tsCmd.LED		= true;
	tsCmd.HLP 		= true;

	tsCmd.TMR 		= true;
	tsCmd.TMR_N50 	= true;
	tsCmd.TMR_HPF	= true;

	tsCmd.EMG		= true;
	tsCmd.EMG_N50	= true;
	tsCmd.EMG_HPF	= true;


	tsSystem.Power	= true;
	tsBle.Connected	= false;

	DIAG ("-----------------------------------------\r\n");
	HAL_Delay(1);
	DIAG (" System ON ->\t%s\r\n\n", DEVICE_INFO);
	HAL_Delay(1);

	// Wait until USB being recognised by the PC.


}

//------------------------------------------------------------------------------------------//
//                                        System control                                    //
//------------------------------------------------------------------------------------------//

void Main_Process (void)
{

	switch (eSYS_STATE)
	{

	default:
		eSYS_STATE = SYS_STATE_INIT;
		break;
		//----------------------------------------------------

	case SYS_STATE_INIT:

		AFE_Init();
		IMU_Init( &hi2c1 );
		BLE_Init();
		Debug_Init();
		Timer_Init();
		BLE_Init();

		DIAG ("-----------------------------------------\r\n\n");
		HAL_Delay(1);
		eSYS_STATE= SYS_STATE_NORMAL;
		break;
		//----------------------------------------------------

	case SYS_STATE_NORMAL:

		if (tsSystem.Power)
		{

			AFE_Control();

	        #ifdef IMU_ENABLE
				IMU_Control();
	        #endif

			Debug_Control();

			#ifdef WBxx_BLE
				BLE_Control();
			#endif

			if (!tsBle.Connected)
			{
				SET_RGB_COLOR(GREEN);

				// If BLE is disconnected, keep the timestamp zero
				tsBle.Timestamp = 0;
			}
			else
			{
				if (tsTimer.u8TimeFlag)
				{
					tsTimer.u8TimeFlag = false;

					switch (LedRGB.color)
					{
					default:
						SET_RGB_COLOR(OFF);
						LedRGB.color = BLUE;
						break;

					case OFF:
						SET_RGB_COLOR(OFF);
						LedRGB.color = BLUE;
						break;

					case BLUE:
						SET_RGB_COLOR(BLUE);
						LedRGB.color = YELLOW;
						break;

					case YELLOW:
						SET_RGB_COLOR(OFF);
						LedRGB.color = WHITE;
						break;

					case WHITE:
						SET_RGB_COLOR(OFF);
						LedRGB.color = RED;
						break;

					case RED:
						SET_RGB_COLOR(OFF);
						LedRGB.color = OFF;
						break;
					}
				}
			}
		}
		else
		{
			SET_RGB_COLOR(OFF);
		}

		eSYS_STATE = SYS_STATE_WAIT_CMD;
		break;
		//----------------------------------------------------
	case SYS_STATE_WAIT_CMD:

		//DIAG("BLE Notified- Kernel - > %d\r\n", McgTimer.TimeFlag );

		if (tsCmd.MODE == Start)
		{
			tsCmd.MODE = Idle;
			ADS1293_SPIWriteReg( ADS1293_CONFIG_REG, Start );
		}

		if (tsCmd.MODE == Stop)
		{
			tsCmd.MODE = Idle;
			ADS1293_SPIWriteReg( ADS1293_CONFIG_REG, Stop );
		}

		if (tsCmd.MODE == PrintSetting )
		{
			tsCmd.MODE = Idle;
			Print_Setting();
		}

		if ( tsCmd.HLP == true )
		{
			//tsCmd.MODE = Idle;
			Print_Help();
			tsCmd.HLP = false;
		}

					#ifdef WBxx_BLE
						if ( tsBle.Connected )
						{
							 if ( tsBle.App.AfeNotification )
							 {
								UTIL_SEQ_SetTask( 1 << CFG_TASK_ALL_TASK, CFG_SCH_PRIO_0);
							 }

							 #ifdef IMU_ENABLE
								 if ( tsBle.App.AccelNotification )
								 {
									UTIL_SEQ_SetTask( 1 << CFG_TASK_ACC_TASK, CFG_SCH_PRIO_1);
								 }
							 #endif
						}
					#endif

		eSYS_STATE = SYS_STATE_NORMAL;
		break;
		//----------------------------------------------------
	}
}

//------------------------------------------------------------------------------------------------

void Print_Help( void )
{
	/*
	DIAG("\n\n\n***************************************************************\r\n");
	HAL_Delay(10);
	DIAG("Neuranics Ltd.");
	HAL_Delay(10);
	DIAG("\r\n***************************************************************\r\n");
	HAL_Delay(10);
	DIAG("ECG-MCG DAQ Rev 1.0\r\n");
	HAL_Delay(10);

	DIAG("---------------------------------------------------------------\r\n");
	HAL_Delay(10);

	DIAG("(Command)\t\t\t(Description)\r\n");
	HAL_Delay(10);
	DIAG("---------------------------------------------------------------\r\n");
	HAL_Delay(10);

	DIAG(" HELP\t\t\t\tSee this guidance again\r\n");
	HAL_Delay(10);

	DIAG(" SETTING\t\t\tSee current setting at any time!\r\n\n");
	HAL_Delay(10);

	DIAG(" START\t\t\t\tStarts the data recording\r\n");
	HAL_Delay(10);
	DIAG(" STOP\t\t\t\tStops the data recording\r\n\n");
	HAL_Delay(10);

	DIAG(" LED ON\t\t\t\tEnable LED\r\n");
	HAL_Delay(10);
	DIAG(" LED OFF\t\t\tDisable LED\r\n\n");
	HAL_Delay(10);

	DIAG(" ECG ON\t\t\t\tEnable ECG\r\n");
	HAL_Delay(10);
	DIAG(" ECG OFF\t\t\tDisable ECG\r\n");
	HAL_Delay(10);

	DIAG(" ECG N50 ON\t\t\tEnable ECG 50Hz notch filter\r\n");
	HAL_Delay(10);
	DIAG(" ECG N50 OFF\t\t\tDisable ECG 50Hz notch filter\r\n");
	HAL_Delay(10);

	DIAG(" ECG HPF ON\t\t\tEnable ECG 5 Hz HPF\r\n");
	HAL_Delay(10);
	DIAG(" ECG HPF OFF\t\t\tDisable ECG 5 Hz HPF\r\n\n");
	HAL_Delay(10);

	DIAG(" MCG ON\t\t\t\tEnable MCG\r\n");
	HAL_Delay(10);
	DIAG(" MCG OFF\t\t\tDisable MCG\r\n");
	HAL_Delay(10);

	DIAG(" MCG N50 ON\t\t\tEnable MCG 50Hz notch filter\r\n");
	HAL_Delay(10);
	DIAG(" MCG N50 OFF\t\t\tDisable MCG 50Hz notch filter\r\n");
	HAL_Delay(10);

	DIAG(" MCG HPF ON\t\t\tEnable MCG 5 Hz HPF\r\n");
	HAL_Delay(10);
	DIAG(" MCG HPF OFF\t\t\tDisable MCG 5 Hz HPF\r\n\n\n");
	HAL_Delay(10);

	DIAG(" Note1:\tThe data order is: ECG,MCG following by CRLF.\r\n");
	HAL_Delay(10);
	DIAG(" Note2:\tPlease use serial plotter software for plotting the data.\r\n");
	HAL_Delay(10);
	DIAG(" Examples 1: https://hackaday.io/project/5334-serialplot-realtime\r\n");
	HAL_Delay(10);
	DIAG(" Examples 2: https://x-io.co.uk/serial-oscilloscope\r\n");
	HAL_Delay(10);
*/
}
//--------------------------------------------------------------------------------------------------------

void Print_Setting ( void )
{

	/*
	DIAG(" \r\n\n\nCurrent Settings:\r\n");
	HAL_Delay(10);
	DIAG("ECG\t\t\t: %s\r\n", tsCmd.ECG ? "active" : "deactive");
	HAL_Delay(10);

	DIAG("ECG 50 Hz Notch Filter\t: %s\r\n", tsCmd.ECG_N50 ? "active" : "deactive");
	HAL_Delay(10);

	DIAG("ECG 5 Hz HPF\t\t: %s\r\n", tsCmd.ECG_HPF ? "active" : "deactive");
	HAL_Delay(10);

	DIAG("MCG\t\t\t: %s\r\n", tsCmd.MCG ? "active" : "deactive");
	HAL_Delay(10);

	DIAG("MCG 50 Hz Notch Filter\t: %s\r\n", tsCmd.MCG_N50 ? "active" : "deactive");
	HAL_Delay(10);

	DIAG("MCG 5 Hz HPF\t\t: %s\r\n", tsCmd.MCG_HPF ? "active" : "deactive");
	HAL_Delay(10);

	DIAG("LED status\t\t: %s\r\n", tsCmd.LED ? "active" : "deactive");
	HAL_Delay(10);
*/
}
