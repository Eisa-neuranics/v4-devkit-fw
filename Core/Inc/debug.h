/*******************************************************************************
 * @file    debug.h
 * @author  Eisa Aghchehli
 * @brief   Debug port controller.
 * @Date	07/11/2023
  ******************************************************************************/
#ifndef DEBUG_H
#define DEBUG_H



#ifdef __cplusplus
extern "C" {
#endif


#include "stm32wbxx_hal.h"
#include "kernel.h"
#include "hardware.h"


#define 		DATA_DEBUG_EN



#define	DEBUG_RX_LEN	64
#define	DEBUG_TX_LEN	524


typedef struct
{
    		uint8_t 	u8RxData[DEBUG_RX_LEN];
    		uint8_t 	u8RxLen;
    		uint8_t 	u8RxCmd;

    		uint8_t 	u8TxData[DEBUG_TX_LEN];
    		uint8_t 	u8TxLen;
    		uint8_t 	u8TxCmd;

volatile 	bool		bDma_Xfer_Cplt;
			uint8_t		u8XferCount;

			int16_t		u16Accel[3];
			int16_t		u16Gyro[3];
			bool		bReleaseIMUData;
			int16_t		i16TxTMR1;
			int16_t		i16TxTMR2;
			int16_t		i16TxEMG;
			bool		bReleaseAFEData;

			uint16_t	u16TimeStamp;
			uint8_t		u8PacketCount[2];

}tsDEBUG;


		void 	DIAG				( const char *fmt, ... );
extern 	void 	Debug_Init			( void );
		void 	Debug_Control 		( void );

#endif // DEBUG_H
