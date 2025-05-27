/*******************************************************************************
 * @file    BLE.h
 * @author  Eisa Aghchehli
 * @brief   MCG-Rev4 BLE controller.
 * @Date	21/06/2024
  ******************************************************************************/
#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#ifdef __cplusplus
extern "C" {
#endif

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



//#define 		BLE_Debug												// 5ms = 200Hz
#define 		BLE_Data_Length		64	//128
#define 		GATT_DATA_LEN		BLE_Data_Length + 10
#define 		CMD_DATA_LEN		10
#define			All_Data_length		115


// Define the states of the state machine
typedef enum {
	BLE_STATE_RESET = 0,
	BLE_STATE_INIT,
	BLE_STATE_ADV,
	BLE_STATE_CHECK_CONNECTION,
	BLE_STATE_CONNECTED,
	BLE_STATE_XFER_SCG,
	BLE_STATE_XFER_SCG_Cmplt,
	BLE_STATE_XFER_ACCEL,
	BLE_STATE_XFER_ACCEL_Cmplt,
	BLE_STATE_XFER_GYRO,
	BLE_STATE_XFER_GYRO_Cmplt,
	BLE_STATE_IDLE,
	BLE_STATE_DISCONNECTED,
	BLE_STATE_ERROR,

	BLE_STATE_TOTAL
} teBLE_STATE;

typedef struct
{
	bool AfeNotification;
	bool AccelNotification;
	bool GyroNotification;

	bool AfeSetTask;
	bool AccelSetTask;
	bool GyroSetTask;

}tsAPP;

typedef struct
{
	volatile 	bool 	Connected;
				bool	Settask;

				bool 	AFE_XferComplete;
				bool 	ACCEL_XferComplete;
				bool 	GYRO_XferComplete;
				uint8_t	Timestamp;
				uint8_t u8PacketCount[2];

				tsAPP   App;
}tsBLE;




extern	void	BLE_Init					( void );
extern	void	BLE_Control					( void );


#endif /* INC_BLUETOOTH_H_ */
