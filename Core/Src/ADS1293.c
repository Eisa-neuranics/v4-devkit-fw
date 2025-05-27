/*******************************************************************************
 * @file    ADS1293.c
 * @author  Eisa Aghchehli
 * @brief   ADS1203 main code
 *
  ******************************************************************************/


#include <stdint.h>

#include "main.h"
#include "stm32wbxx_hal.h"
#include "stm32wbxx_hal_spi.h"

#include "kernel.h"
#include "debug.h"
#include "hardware.h"
#include "ADS1293.h"

//******************************************************************************

//-------------------------TYPEDEF----------------------------------------------
extern SPI_HandleTypeDef hspi1;
//extern UART_HandleTypeDef huart1;

//-------------------------VARIABLES--------------------------------------------
uint8_t x, TxData[10], TxValue[10], RxData[10];


//------------------------------------------------------------------------------
//  void ADS1293_SPISetup(void)
//
//  DESCRIPTION:
//  Configures the assigned interface to function as a SPI port and
//  initializes it.
//------------------------------------------------------------------------------
void ADS1293_Init(void)
{
	ENABLE_RSTB_ADS;			// Reset ADS
	HAL_Delay (25);
	DISABLE_RSTB_ADS;			// Release Reset pin
	HAL_Delay (100);

	ADS1293_SPIWriteReg( ADS1293_CONFIG_REG			, ADS_STOP );
	HAL_Delay (25);
	ADS1293_SPIWriteReg( ADS1293_OSC_CN_REG			, 0x04 );				// Use external crystal and feed the internal oscillator's output to the digital.
	HAL_Delay (25);
	ADS1293_SPIWriteReg( ADS1293_FLEX_CH1_CN_REG	, 0x0A );				// CH1 is routed to: N = IN1 , P = IN2
	HAL_Delay (25);
	ADS1293_SPIWriteReg( ADS1293_FLEX_CH2_CN_REG	, 0x1C );				// CH1 is routed to: N = IN1 , P = IN2
	HAL_Delay (25);
	ADS1293_SPIWriteReg( ADS1293_FLEX_CH3_CN_REG	, 0x2E );				// CH3 is routed to: N = IN6 , P = IN5   0x2E	0x35
	HAL_Delay (25);

	ADS1293_SPIWriteReg( ADS1293_RLD_CN_REG			, 0x06 );				// 0x04: RLD amplifier powered down, 0x03: Right-leg drive output connected to IN3
	HAL_Delay (25);
	ADS1293_SPIWriteReg( ADS1293_CMDET_EN_REG		, 0x30 );				// 0x04: RLD amplifier powered down
	HAL_Delay (25);

	ADS1293_SPIWriteReg( ADS1293_R2_RATE_REG		, R2_4 );				// Configures the R2 decimation rate as 5 for all channels.
	HAL_Delay (25);
	ADS1293_SPIWriteReg( ADS1293_R3_RATE1_REG		, R3_6 );				// Configures the R3 decimation rate as 6 for channel 1. (853 sps, 175 Hz)
	HAL_Delay (25);
	ADS1293_SPIWriteReg( ADS1293_R3_RATE2_REG		, R3_6 );				// Configures the R3 decimation rate as 6 for channel 3. (853 sps, 175 Hz)
	HAL_Delay (25);
	ADS1293_SPIWriteReg( ADS1293_R3_RATE3_REG		, R3_6 );				// Configures the R3 decimation rate as 6 for channel 3. (853 sps, 175 Hz)
	HAL_Delay (25);
	ADS1293_SPIWriteReg( ADS1293_DRDYB_SRC_REG		, 0x08 );				// Data Ready Pin Source CH1 ECG 0x08
	HAL_Delay (25);
	ADS1293_SPIWriteReg( ADS1293_CH_CNFG_REG		, 0x70 );				// Channel for Loop Read Back: CH1
	HAL_Delay (25);
	//ADS1293_SPIWriteReg( ADS1293_DIS_EFILTER_REG	, 0x05 );				// Channel for Loop Read Back: CH1 and CH2
	//HAL_Delay (25);

}

//------------------------------------------------------------------------------
//  void ADS1293_START(void)
//
//  DESCRIPTION:
//  START/STOP the conversion
//------------------------------------------------------------------------------
void ADS1293_PWR(uint8_t state)
{
	if (state == ADS_STOP)
	{
		ADS1293_SPIWriteReg( ADS1293_CONFIG_REG, ADS_STOP);
		#ifdef ADS_DEBUG_EN
			DIAG(">>> ADS STOP\r\n");				// Send DBG
			HAL_Delay(50);
		#endif
	}

	if (state == ADS_START)
	{
		ADS1293_SPIWriteReg( ADS1293_CONFIG_REG, ADS_START);
		#ifdef ADS_DEBUG_EN
			DIAG(">>> ADS START\r\n");				// Send DBG
			HAL_Delay(50);
		#endif
	}

	if (state == ADS_STANDBY)
	{
		ADS1293_SPIWriteReg( ADS1293_CONFIG_REG, ADS_STANDBY);
		#ifdef ADS_DEBUG_EN
			DIAG(">>> ADS STANDBY\r\n");				// Send DBG
			HAL_Delay(50);
		#endif
	}

	if (state == ADS_PWR_DOWN)
	{
		ADS1293_SPIWriteReg( ADS1293_CONFIG_REG, ADS_PWR_DOWN);
		#ifdef ADS_DEBUG_EN
			DIAG(">>> ADS POWER DOWN\r\n");				// Send DBG
			HAL_Delay(50);
		#endif
	}
}

//------------------------------------------------------------------------------
//  uint8_t ADS1293_SPIReadReg(uint8_t addr)
//
//  DESCRIPTION:
//  Reads a single configuration register at address "addr" and returns the
//  value read.
//------------------------------------------------------------------------------
uint8_t ADS1293_SPIReadReg(uint8_t addr)
{
	TxData[0] = addr | ADS1293_READ_BIT;											// register address
	TxData[1] = 0x00;

	ENABLE_CS; 												// CS enable
	HAL_SPI_TransmitReceive(&hspi1, TxData, RxData, 2,100);								// Send it, receive it
	DISABLE_CS; 											// CS disable

#ifdef ADS_DEBUG_EN
	DIAG(">>> Read Reg. -> [ Address= %0X, Value= %0X ]\r\n", addr, RxData[1]);				// Send DBG
	HAL_Delay(50);
#endif

	return RxData[1];
}

//------------------------------------------------------------------------------
//  void ADS1293_SPIWriteReg(uint8_t addr, uint8_t value)
//
//  DESCRIPTION:
//  Writes "value" to a single configuration register at address "addr".
//------------------------------------------------------------------------------
void ADS1293_SPIWriteReg(uint8_t addr, uint8_t value)
{
	TxData[0] = addr & ADS1293_WRITE_BIT;											// register address
	TxData[1] = value;

	ENABLE_CS; 												// CS enable
	HAL_SPI_Transmit(&hspi1, TxData, 2,100);						// Send it
	DISABLE_CS; 											// CS disable

	#ifdef ADS_DEBUG_EN
		DIAG(">>>Write Reg. -> [ Address= %0X, Value= %0X ]\r\n",addr, value);				// Send DBG
		HAL_Delay(50);
	#endif

}

//------------------------------------------------------------------------------
//  void ADS1293_SPIAutoIncWriteReg(uint8_t addr, uint8_t *buffer, uint8_t count)
//
//  DESCRIPTION:
//  Writes values to multiple configuration registers, the first register being
//  at address "addr".  First data byte is at "buffer", and both addr and
//  buffer are incremented sequentially until "count" writes have been performed.
//------------------------------------------------------------------------------
void ADS1293_SPIAutoIncWriteReg(uint8_t addr, uint8_t *buffer, uint8_t count)
{

	uint8_t i=0;

	TxData[0] = addr & ADS1293_WRITE_BIT;							// register address


	for ( i=0; i < count; i++ )
	{
		TxData[i+1] = *( buffer + i );
	}

	ENABLE_CS; 														// CS disable
	HAL_SPI_Transmit(&hspi1, TxData, count,100);						// Send it
	DISABLE_CS; 													// CS disable

}

//------------------------------------------------------------------------------
//  void ADS1293_SPIAutoIncReadReg(uint8_t addr, unit8_t *buffer, uint8_t count)
//
//  DESCRIPTION:
//  Reads multiple configuration registers, the first register being at address
//  "addr".  Values read are deposited sequentially starting at address
//  "buffer", until "count" registers have been read.
//------------------------------------------------------------------------------
void ADS1293_SPIAutoIncReadReg(uint8_t addr, uint8_t *buffer, uint8_t count)
{
	TxData[0] = ADS1293_READ_BIT | addr;														// register address
	TxData[1] = 0x00; TxData[2] = 0x00; TxData[3] = 0x00; TxData[4] = 0x00; TxData[5] = 0x00; TxData[6] = 0x00; TxData[7] = 0x00; TxData[8] = 0x00; TxData[9] = 0x00;	// write dummy data to read

	ENABLE_CS; 																					// CS enable
	HAL_SPI_TransmitReceive(&hspi1, TxData, (uint8_t *)buffer, count,100);						// Send it, receive it
	DISABLE_CS;																					// CS disable
}

//------------------------------------------------------------------------------
//  void ADS1293_SPIStreamReadReg(uint8_t *buffer, uint8_t count)
//
//  DESCRIPTION:
//  Special read function for reading status, pace and ecg data registers of selected
//  channels. Channels to be read must be selected in CH_CNFG before calling this function.
//  Data Loop Register read is extended "count+1" times where "count" is number of source bytes
//  enabled in CH_CNFG. Data read are deposited sequentially starting at address "buffer"
//  until "count" bytes have been read.
//------------------------------------------------------------------------------
void ADS1293_SPIStreamReadReg ( uint8_t *buffer, uint8_t count )
{
	TxData[0] = ADS1293_READ_BIT | ADS1293_DATA_LOOP_REG;											// register address
	TxData[1] = 0x00; TxData[2] = 0x00; TxData[3] = 0x00; TxData[4] = 0x00; TxData[5] = 0x00; TxData[6] = 0x00; TxData[7] = 0x00; TxData[8] = 0x00; TxData[9] = 0x00;	// write dummy data to read

	ENABLE_CS; 																						// CS enable
	HAL_SPI_TransmitReceive(&hspi1, TxData, (uint8_t *)buffer, ( count + 1 ), 100);					// Send it, receive it
	DISABLE_CS;

	#ifdef ADS_DEBUG_EN
		DIAG(">>> Stream read. -> [ Address= %0X, Value= %0X - %0X - %0X - %0X - %0X - %0X ]\r\n", ADS1293_DATA_LOOP_REG, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[6] );				// Send DBG
		HAL_Delay(50);
	#endif
}










































