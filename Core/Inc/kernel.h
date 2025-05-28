/*
 * kernel.h
 *
 *  Created on: Sep 11, 2023
 *      Author: nea79
 */

#ifndef __KERNEL_H_
#define __KERNEL_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/

#include "stm32wbxx_hal.h"
#include "main.h"
#include "IIR_Filter.h"
#include "ADS1293.h"
#include "AFE.h"
#include "hardware.h"
#include <Bluetooth.h>



// No IMU
#ifndef IMU_ENABLE
//#define 		DEVICE_INFO  	"MCG-04,411,310724"
#define 		DEVICE_INFO  	"MMG-01,411,070524"
#define 		HW_INFO  		"MCG-04,411,A"
#define 		FW_INFO  		"MCG-04,406,A"
#endif


// With IMU
#ifdef IMU_ENABLE
#define 		DEVICE_INFO  	"MCG-04,411,310724"
#define 		HW_INFO  		"MCG-04,411,B"
#define 		FW_INFO  		"MCG-04,406,B"
#endif


#define			Interrupt_ms		500
#define			PowerOffTime		30	// 60 = 3000ms @50ms in TIM2
#define 		CLEAR				0x00

#define ENABLE_PWR			GPIOA->BSRR = GPIO_BSRR_BS15;
#define DISABLE_PWR			GPIOA->BSRR = GPIO_BSRR_BR15;

/* Exported types ------------------------------------------------------------*/


typedef enum
{
	SYS_STATE_INIT = 0,
	SYS_STATE_INIT_AFE,
	SYS_STATE_INIT_BLE,
	SYS_STATE_NORMAL,
	SYS_STATE_WAIT_CMD,
	SYS_STATE_OBSERVE_POWER,
	SYS_STATE_ERROR,

	STATE_TOTAL
} teSYS_STATE;

enum eSYS_MODE
{
	Stop = 0,
	Start,
	Standby,
	PowerUp,
	PowerDown,
	PrintHelp,
	PrintSetting,

	Idle = 0xFF
};

enum eCMD
{
//------------------------------------------------------------------------------------
// [ Command ]	     [ Value (Hex) ]	   [ Description ]
//------------------------------------------------------------------------------------
	STOP =1,		// 0x01					Stop the data acquisition
	START,			// 0x02					Start the data acquisition

	ECG_ON,			// 0x03					Enable ECG
	ECG_OFF,		// 0x04					Disable ECG
	ECG_N50_ON,		// 0x05					Enable 50 Hz Notch filter for ECG
	ECG_N50_OFF,	// 0x06					Disable 50 Hz Notch filter for ECG
	ECG_HPF_ON,		// 0x07					Enable 5 Hz High-Pass Filter for ECG
	ECG_HPF_OFF,	// 0x08					Disable 5 Hz High-Pass Filter for ECG

	MCG_ON,			// 0x09					Enable MCG
	MCG_OFF,		// 0x0A					Disable MCG
	MCG_N50_ON,		// 0x0B					Enable 50 Hz Notch filter for MCG
	MCG_N50_OFF,	// 0x0C					Disable 50 Hz Notch filter for MCG
	MCG_HPF_ON,		// 0x0D					Enable 5 Hz High-Pass Filter for MCG
	MCG_HPF_OFF,	// 0x0E					Disable 5 Hz High-Pass Filter for MCG

	LED_ON,			// 0x0F					Turn ON LED
	LED_OFF			// 0x10					Turn OFF LED
};
//------------------------------------------------------------------------------------

typedef struct
{
	 volatile uint8_t 	DRDY;

}tsADS;

typedef struct
{
	  bool 	Power;
	  bool	ON;
	  bool	OFF;
	  bool	StandBy;
	  bool	PowerSaving;

	  bool	bPowerOnFlag;;

}tsSYSTEM;


 enum eLED
{
     OFF    = 0x00,
	 RED    = 0x01,
	 GREEN  = 0x02,
	 BLUE   = 0x04,
	 YELLOW = 0x03,
	 CYAN	= 0x06,
	 WHITE	= 0x07
};

 typedef struct
 {
 	uint8_t		mode;
 	uint16_t 	time;
 	uint8_t		color;
 	bool 		DataOutEN;
 } tsSYS;

typedef struct
{
	uint8_t		mode;
	uint16_t 	time;
	uint8_t		color;
} tsLED;

typedef struct
{
	uint8_t	CMD;
	uint8_t u8RxCmd[CMD_DATA_LEN];
	char 	c8RxCmd[CMD_DATA_LEN];
	uint8_t	MODE;
	bool	HLP;
	bool	LED;
	bool	TMR;
	bool	TMR_N50;
	bool	TMR_HPF;

	bool	EMG;
	bool	EMG_N50;
	bool	EMG_HPF;
}tsCMD;

// Stream Data Structure
typedef struct
{
    uint8_t  	Cmd;                                                 // Command to stream
    uint16_t 	nterval;                                            // Periodic Timer Value
    uint8_t 	Counter;                                             // Counter
    bool   		Enabled;                                             // Status

    uint8_t 	u8TxData [256];

    uint8_t 	u8TMR1TxCpy[72];
    uint8_t 	u8TMR2TxCpy[72];
    uint8_t 	u8EMGTxCpy[72];

	uint8_t 	u8AxTxData[72];
	uint8_t 	u8AyTxData[72];
	uint8_t 	u8AzTxData[72];

    uint8_t		u8TxLen;
    bool 		bReleaseAccelData;
    bool 		bReleaseGyroData;
    bool 		bReleaseAfeData;

} tsSTREAM;

// Data Packet Structure
typedef struct {
	// Data
    uint8_t u8RxHandle[5];                                        // Holds the Short Characteristic Handle to use when reading data
    uint8_t u8RxData[GATT_DATA_LEN];                              // Data packet containing Ble Read/Write data
    uint8_t u8RxPktLen;                                           // Length of the entire packet minus - Headers. Only has Cmd, Length & Data
    uint8_t u8RxDataLen;                                          // Data length minus - Headers, Command & Length bytes

    uint8_t u8RxCmd;                                              //
    uint8_t u8RxSubCmd;
    uint8_t u8RxChecksum;

    uint8_t u8TxHandle[5];                                        // Holds the Short Characteristic Handle to use when writing data
    uint8_t u8TxData[GATT_DATA_LEN];                              // Data packet containing Ble Read/Write data
    uint8_t u8TxLen;                                              //
    uint8_t u8TxCmd;                                              //
    uint8_t u8TxChecksum;                                         // Checksum

} tsDataPacket;

/* Exported constants --------------------------------------------------------*/
//static const char SERVICE_UUID[] = {"726d1977-543b-4858-ad16-ddde9e02d7f3"};


/* External variables --------------------------------------------------------*/







/* Exported functions prototypes ---------------------------------------------*/

extern 	void 	Kernel_Init  					( void );
extern 	void 	Main_Process 					( void );


		void 	Print_Help						( void );
		void 	Print_Setting					( void );


/* Private defines -----------------------------------------------------------*/
//#define ADS_DEBUG_EN
//#define ECG_DEBUG_EN









#endif /*
_KERNEL_H_ */
