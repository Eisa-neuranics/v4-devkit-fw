/*******************************************************************************
 * @file    debug.c
 * @author  Eisa Aghchehli
 * @brief   Debug port controller.
 * @Date	07/11/2023
  ******************************************************************************/



#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "stm32wbxx_hal.h"

#include "hardware.h"
#include "LSM6DSLTR.h"
#include "timer.h"
#include "debug.h"
#include <Bluetooth.h>

//-----------------------------------------------------------------------------------
#ifdef	WB55_USB
	#include "usb_device.h"
	#include "usbd_cdc_if.h"
#endif

#ifndef WB55_USB
	#define USART_DEBUG
#endif


		tsDEBUG				tsDebug;
extern	tsIMU				IMU;
extern  tsBLE				tsBle;
extern  tsTIMER				tsTimer;
extern	tsAFE				AFE;

extern	UART_HandleTypeDef 	huart1;
//-----------------------------------------------------------------------------------

uint16_t u16dummy =0;


//-----------------------------------------------------------------------------------
void Debug_Init (void)
{

	DIAG ("\t\tDebug initialised\r\n");
	HAL_Delay(10);
}

//-----------------------------------------------------------------------------------
void Debug_Control (void)
{

#ifndef BLE_Debug
#ifdef DATA_DEBUG_EN

		#ifndef IMU_ENABLE
			if ( tsDebug.bReleaseAFEData )
			{
//				DIAG ( "%d,%d,%d\r\n", tsDebug.i16TxTMR1, tsDebug.i16TxTMR2, tsDebug.i16TxEMG );
//				DIAG ( "%d,%d,%d,%d,%d\r\n", tsDebug.i16TxTMR1, tsDebug.i16TxTMR2, tsDebug.i16TxEMG, tsDebug.u8PacketCount[0], tsDebug.u8PacketCount[1] );

				DIAG (  "[%d - %d]-%d,%d\r\n", AFE.u16SPS, IMU.u16SPS, tsDebug.i16TxTMR1, tsDebug.i16TxTMR2 );

				tsDebug.bReleaseAFEData= false;
			}
		#endif


		#ifdef 	IMU_ENABLE
			if ( tsDebug.bReleaseAFEData )
			{
//				DIAG (  "%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
//						AFE.u16SPS, IMU.u16SPS,
#ifdef GYRO_ENABLE
				DIAG (  "%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
						tsDebug.i16TxTMR1, tsDebug.i16TxTMR2, tsDebug.i16TxEMG,
						tsDebug.u16Accel[0], tsDebug.u16Accel[1], tsDebug.u16Accel[2],
						tsDebug.u16Gyro[0], tsDebug.u16Gyro[1], tsDebug.u16Gyro[2] );
#endif

#ifndef GYRO_ENABLE
				DIAG (  "%d,%d,%d,%d,%d,%d\r\n",
						tsDebug.i16TxTMR1, tsDebug.i16TxTMR2, tsDebug.i16TxEMG,
						tsDebug.u16Accel[0], tsDebug.u16Accel[1], tsDebug.u16Accel[2]);
#endif


//				DIAG (  "%d,%d,%d,%d\r\n",
//						tsDebug.i16TxMCG, tsDebug.u16Accel[0], tsDebug.u16Accel[1], tsDebug.u16Accel[2]);

				tsDebug.bReleaseIMUData= false;
				tsDebug.bReleaseAFEData= false;
//				IMU.u16SampleCount++;

			}
		#endif

#endif
#endif
}

//-----------------------------------------------------------------------------------
#ifdef USART_DEBUG
void DIAG(const char *fmt, ...) {
    char buffer[255];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 10 );
   // HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buffer, strlen(buffer) );
   // HAL_UART_DMAResume(&huart1);
}
#else

void DIAG(const char *fmt, ...) {
    char buffer[4048];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
}

#endif

/*
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		tsDebug.bDma_Xfer_Cplt = true;
		HAL_UART_DMAStop(&huart1);
	}
}
*/
