/*
 * AFE.h
 *
 *  Created on: Jun 1, 2024
 *      Author: eisaa
 */

#ifndef INC_AFE_H_
#define INC_AFE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "kernel.h"
#include "IIR_Filter.h"
#include "ADS1293.h"
#include "hardware.h"


#define 		AFE_ENABLE

//#define		AFE_DEBUG_EN
#define 		ECG_DEBUG_EN

#define 		Temp_BUF_LEN	25


// Define the states of the state machine
typedef enum {
	AFE_STATE_RESET = 0,
	AFE_STATE_INIT,
	AFE_STATE_ID,
	AFE_STATE_START,
	AFE_STATE_READ_DATA,
	AFE_STATE_PROCESS_DATA,
	AFE_STATE_IDLE,
	AFE_STATE_ERROR,

	AFE_STATE_TOTAL
} teAFE_STATE;

typedef struct
{
	int32_t		Raw[Temp_BUF_LEN];
	int32_t		Filtered[Temp_BUF_LEN];
	int32_t		TempFilter[Temp_BUF_LEN];
	int32_t		HPF[Temp_BUF_LEN];
	int32_t		Notch50[Temp_BUF_LEN];
	int32_t		Notch60[Temp_BUF_LEN];

	int32_t		Input;
    uint8_t 	u8TxData[150];                              // Data packet containing Ble Read/Write data
    uint8_t 	u8TxLen;                                              //
    uint8_t 	u8TxCmd;                                              //

	uint8_t		Temp_xyptr;



	float		Vin;
	int32_t		Vout;

	uint8_t 	u8Error;
}tsSIG;


typedef struct
{
	volatile uint16_t 	u16SampleCount;
	volatile uint16_t 	u16SPS;
}tsAFE;

		void 		AFE_Init 				( void );
		void 		AFE_Control 			( void );
		void 		AFE_StreamDataPack 		( uint8_t *TMR1Data, uint8_t TMR1DataLen,
											  uint8_t *TMR2Data, uint8_t TMR2DataLen,
											  uint8_t *EMGData , uint8_t EMGDataLen,
											  uint8_t *outputString );

//extern	void 		AFE_StreamDataPack 		( uint8_t *ecgData, uint8_t ecgDataLen,   uint8_t *mcgData, uint8_t mcgDataLen, uint8_t *outputString  );
extern 	void 		Ble_Task				(void);
		uint8_t 	u8Transmit_DataPacket	( uint8_t cmd, uint8_t sub_cmd, uint8_t *buffer, uint8_t len );
		void 		HAL_GPIO_EXTI_Callback	(uint16_t GPIO_Pin);

#endif /* INC_AFE_H_ */
