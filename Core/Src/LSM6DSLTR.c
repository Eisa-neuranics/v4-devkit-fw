/**
 ******************************************************************************
 * @file    LSM6DSLTR.c
 * @author  Eisa Aghchehli
 * @brief   This file provides a set of functions needed to manage the LSM6DSL
 *          accelero and gyro devices
 ******************************************************************************/


// Notes:
/* 19/10/2024: The sampling rate for IMU data is 370 SPS as per measurement done using timer.
 *
 *
 *
 *
 *
 *
 */

//Includes ------------------------------------------------------------------


#include <stdint.h>
#include <string.h>         // memcmp()
#include <stddef.h>         // NULL definition
#include <stdio.h>          // printf()

#include "stm32wbxx_hal.h"
#include "stm32wbxx_hal_i2c.h"
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

//----------------------------------------------------------------------------------------------------
		teIMU_STATE 		teIMU_State = IMU_STATE_IDLE;
		tsIMU				IMU;
extern	tsBLE				tsBle;
		tsSTREAM			tsStreamIMU;
extern	tsSTREAM			tsStreamSCG;
extern	tsDEBUG				tsDebug;
extern	I2C_HandleTypeDef 	hi2c1;
extern	tsTIMER				tsTimer;
 		tsHighPassFilter 	accelFilter, gyroFilter;
		tsVector3D 			filteredAccelData, filteredGyroData;
		tsVector3D 			accelData;
		tsVector3D			gyroData;

	 	tsHighPassFilter 	accelFilter, gyroFilter;

uint8_t 	id 			= 	20;
uint8_t 	cutoff_freq = 	1; 					// Cutoff frequency for high-pass filter (Hz)
uint8_t		u8AxDataLen, u8AyDataLen, u8AzDataLen, u8GxDataLen, u8GyDataLen, u8GzDataLen;

#define 	M_PI 			3.14159265358979323846
#define		IMU_SPS			1067
//----------------------------------------------------------------------------------------------------
HAL_StatusTypeDef IMU_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t config[2];

    // Set the HW address on IMU to 0x6A
    ADDR_IMU_6A;

    // Enable accelerometer, 104 Hz, 2g
    config[0] = IMU_CTRL1_XL;
    config[1] = 0x40;
    if (HAL_I2C_Master_Transmit(hi2c, IMU_ADDR, config, 2, I2C_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // Enable gyroscope, 104 Hz, 250 dps
    config[0] = IMU_CTRL2_G;
    config[1] = 0x40;
    if (HAL_I2C_Master_Transmit(hi2c, IMU_ADDR, config, 2, I2C_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // BDU (Block Data Update) enabled
    config[0] = IMU_CTRL3_C;
    config[1] = 0x44;
    if (HAL_I2C_Master_Transmit(hi2c, IMU_ADDR, config, 2, I2C_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HighPassFilter_Init	( &accelFilter, cutoff_freq, IMU_SPS );
    HighPassFilter_Init	( &gyroFilter , cutoff_freq, IMU_SPS );

    DIAG ("\t\tIMU initialised\r\n");

    return HAL_OK;
}

//------------------------------------------------------------------------------------------//
//                                        System control                                    //
//------------------------------------------------------------------------------------------//
void IMU_Control ( void )
{
	switch (teIMU_State)

	{
		case IMU_STATE_IDLE:

			teIMU_State = IMU_STATE_INIT;
			break;

		case IMU_STATE_INIT:

			ENABLE_IMU_CS;
			if (IMU_Init(&hi2c1) != HAL_OK)
			{
				#ifdef IMU_DEBUG_EN
				DIAG (" IMU Initialized\r\n");
				#endif
			}
			else
			{
				#ifdef IMU_DEBUG_EN
				DIAG ( " IMU ERROR\r\n" );
				#endif
			}

			if ( IMU_ReadID(&hi2c1, &id) == HAL_OK )
			{
				#ifdef IMU_DEBUG_EN
				DIAG ( " IMU OK = %X\r\n", id );
				#endif
			}
			else
			{
				#ifdef IMU_DEBUG_EN
				DIAG ( "No IMU= %X\r\n", id );
				#endif
			}
			DISABLE_IMU_CS;

			teIMU_State = IMU_STATE_WAIT_INT;
			break;

		case IMU_STATE_WAIT_INT:

			teIMU_State = IMU_STATE_SELECT_SLAVE;

			break;

		case IMU_STATE_SELECT_SLAVE:

			if ( tsTimer.bImuFlag )
			{
				#ifdef IMU_DEBUG_EN
					DIAG ( "IMU Sampling\r\n" );
				#endif
				tsTimer.bImuFlag = false;
				ENABLE_IMU_CS;
				teIMU_State = IMU_STATE_REQ_ACCEL;
			}
			else
			{
				teIMU_State = IMU_STATE_SELECT_SLAVE;
			}

			break;

		case IMU_STATE_REQ_ACCEL:

			if ( IMU_ReadAccel(&hi2c1, &accelData ) == HAL_OK )
			{

				HighPassFilter_Apply ( &accelFilter, &accelData, &filteredAccelData );
				#ifdef IMU_DEBUG_EN
					DIAG ( "Accel. data received\r\n" );
				#endif
			}
			else
			{
				#ifdef IMU_DEBUG_EN
				DIAG ( "No Accel. Data\r\n" );
				#endif
			}

			teIMU_State = IMU_STATE_DESELECT_SLAVE;
			#ifdef GYRO_ENABLE
				teIMU_State = IMU_STATE_REQ_GYRO;
			#endif
			break;

		case IMU_STATE_REQ_GYRO:

			if ( IMU_ReadGyro(&hi2c1, &gyroData ) == HAL_OK )
			{
				 HighPassFilter_Apply( &gyroFilter, &gyroData, &filteredGyroData );
				 #ifdef IMU_DEBUG_EN
					DIAG ( "Gyro. data received\r\n" );
				 #endif
			}
			else
			{
				#ifdef IMU_DEBUG_EN
				DIAG ( "No Gyro. Data\r\n" );
				#endif
			}
			teIMU_State = IMU_STATE_DESELECT_SLAVE;
			break;

		case IMU_STATE_DESELECT_SLAVE:

			//DISABLE_IMU_CS;
			teIMU_State = IMU_STATE_PROCESS_DATA;
			break;

		case IMU_STATE_PROCESS_DATA:

			teIMU_State = IMU_STATE_RELEASE_DATA;
			break;

		case IMU_STATE_RELEASE_DATA:

			// Put into buffer for Serial/USB data output
			tsDebug.u16Accel[0] = filteredAccelData.x;
			tsDebug.u16Accel[1] = filteredAccelData.y;
			tsDebug.u16Accel[2] = filteredAccelData.z;
			tsDebug.u16Gyro [0] = filteredGyroData.x;
			tsDebug.u16Gyro [1] = filteredGyroData.y;
			tsDebug.u16Gyro [2] = filteredGyroData.z;

//			tsDebug.u16Accel[0] = accelData.x;
//			tsDebug.u16Accel[1] = accelData.y;
//			tsDebug.u16Accel[2] = accelData.z;
//			tsDebug.u16Gyro [0] = gyroData.x;
//			tsDebug.u16Gyro [1] = gyroData.y;
//			tsDebug.u16Gyro [2] = gyroData.z;


			tsDebug.bReleaseIMUData = true;
			IMU.u16SampleCount++;

			// Put into buffer for BLE data output
			IMU.u8AxTxData[u8AxDataLen++] = ( (  		filteredAccelData.x & 0xFF00 ) >> 8 );
			IMU.u8AxTxData[u8AxDataLen++] = ( (uint8_t)  filteredAccelData.x & 0x00FF 		);
			IMU.u8AyTxData[u8AyDataLen++] = ( (  		filteredAccelData.y & 0xFF00 ) >> 8 );
			IMU.u8AyTxData[u8AyDataLen++] = ( (uint8_t)  filteredAccelData.y & 0x00FF 		);
			IMU.u8AzTxData[u8AzDataLen++] = ( (  		filteredAccelData.z & 0xFF00 ) >> 8 );
			IMU.u8AzTxData[u8AzDataLen++] = ( (uint8_t)  filteredAccelData.z & 0x00FF 		);

			#ifdef GYRO_ENABLE
				// Put into buffer for BLE data output
				IMU.u8GxTxData[u8GxDataLen++] = ( (  		filteredGyroData.x & 0xFF00 ) >> 8 );
				IMU.u8GxTxData[u8GxDataLen++] = ( (uint8_t)  filteredGyroData.x & 0x00FF 		);
				IMU.u8GyTxData[u8GyDataLen++] = ( (  		filteredGyroData.y & 0xFF00 ) >> 8 );
				IMU.u8GyTxData[u8GyDataLen++] = ( (uint8_t)  filteredGyroData.y & 0x00FF 		);
				IMU.u8GzTxData[u8GzDataLen++] = ( (  		filteredGyroData.z & 0xFF00 ) >> 8 );
				IMU.u8GzTxData[u8GzDataLen++] = ( (uint8_t)  filteredGyroData.z & 0x00FF 		);
			#endif
			// Check if buffer is getting full
			if ( u8AxDataLen >=  IMU_Data_Length )
			{
				// Release the data into BLE notification
				tsStreamIMU.bReleaseAccelData = true;
				#ifdef GYRO_ENABLE
					tsStreamIMU.bReleaseGyroData = true;
				#endif
				// Copy data lengths
				IMU.u8AxTxLen = u8AxDataLen; 	IMU.u8AyTxLen = u8AyDataLen; 	IMU.u8AzTxLen = u8AzDataLen;
				IMU.u8GxTxLen = u8GxDataLen; 	IMU.u8GyTxLen = u8GyDataLen; 	IMU.u8GzTxLen = u8GzDataLen;

				// Reset the pointer
				u8AxDataLen = 0;
				u8AyDataLen = 0;
				u8AzDataLen = 0;
				u8GxDataLen = 0;
				u8GyDataLen = 0;
				u8GzDataLen = 0;
			}

			teIMU_State = IMU_STATE_SELECT_SLAVE;
			break;

		//----------------------------------------------------
		default:

			teIMU_State = IMU_STATE_ERROR;
			IMU_Error();
			break;
	}
}

//-----------------------------------------------------------------------------------------------------
HAL_StatusTypeDef IMU_ReadID(I2C_HandleTypeDef *hi2c, uint8_t *id)
{
    return HAL_I2C_Mem_Read(hi2c, IMU_ADDR, IMU_WHO_AM_I, 1, id, 1, I2C_DELAY);
}
//-----------------------------------------------------------------------------------------------------

HAL_StatusTypeDef IMU_ReadAccel(I2C_HandleTypeDef *hi2c, tsVector3D *pData)
{
    uint8_t rawData[6];

    if ( HAL_I2C_Mem_Read(hi2c, IMU_ADDR, IMU_OUTX_L_XL, 1, rawData, 6, I2C_DELAY) != HAL_OK )
    {
        return HAL_ERROR;
    }

    pData->x = (int16_t)((rawData[1] << 8) | rawData[0]);
    pData->y = (int16_t)((rawData[3] << 8) | rawData[2]);
    pData->z = (int16_t)((rawData[5] << 8) | rawData[4]);

    return HAL_OK;
}
//-----------------------------------------------------------------------------------------------------

HAL_StatusTypeDef IMU_ReadGyro(I2C_HandleTypeDef *hi2c, tsVector3D *pData)
{
    uint8_t rawData[6];

    if ( HAL_I2C_Mem_Read(hi2c, IMU_ADDR, IMU_OUTX_L_G, 1, rawData, 6, I2C_DELAY) != HAL_OK )
    {
        return HAL_ERROR;
    }

    pData->x = (int16_t)((rawData[1] << 8) | rawData[0]);
    pData->y = (int16_t)((rawData[3] << 8) | rawData[2]);
    pData->z = (int16_t)((rawData[5] << 8) | rawData[4]);

    return HAL_OK;
}
//--------------------------------------------------------------------------------------------

void IMU_Error ( void )
{
	DIAG (" IMU ERROR\r\n");
	//SET_RGB_COLOR (RED);
	//tsSystem.u8Error |= eBus_Error;
}
//--------------------------------------------------------------------------------------------

void IMU_StreamDataPack ( uint8_t sensor, uint8_t *x_pData, uint8_t *y_pData, uint8_t *z_pData, uint8_t *outputString   )
{
 	uint8_t index=0;

 	// Clear buffer
 	memset( tsStreamIMU.u8TxData, 0, sizeof(tsStreamIMU.u8TxData) );

 	outputString [index++] = 'S';
 	outputString [index++] = tsBle.Timestamp;				//'N'
 	outputString [index++] = ',';
 	outputString [index++] = 'C';
 	if ( sensor == Accelerometer)
 	{
 		outputString [index++] = '3';
 	}
 	else
 	{
 		outputString [index++] = '6';
 	}
 	outputString [index++] = ',';
 	outputString [index++] = 64;
 	outputString [index++] = ',';

 	//DIAG( "%d\r", tsStreamSCG.Counter);
 	memcpy ( outputString + index, x_pData, 64 );
 	index += 64;

 	outputString [index++] = ',';
 	outputString [index++] = 'C';
 	if ( sensor == Accelerometer)
 	{
 		outputString [index++] = '4';
 	}
 	else
 	{
 		outputString [index++] = '7';
 	}
 	outputString [index++] = ',';
 	outputString [index++] = 64;
 	outputString [index++] = ',';

 	memcpy ( outputString + index, y_pData, 64 );
 	index += 64;

 	outputString [index++] = ',';
 	outputString [index++] = 'C';
 	if ( sensor == 0)
 	{
 		outputString [index++] = '5';
 	}
 	else
 	{
 		outputString [index++] = '8';
 	}
 	outputString [index++] = ',';
 	outputString [index++] = 64;
 	outputString [index++] = ',';

 	memcpy ( outputString + index, z_pData, 64 );
}
//--------------------------------------------------------------------------------------------

void HighPassFilter_Init(tsHighPassFilter *filter, float cutoff_freq, float sampling_rate)
{
    float rc = 1.0 / (2.0 * M_PI * cutoff_freq);
    float dt = 1.0 / sampling_rate;
    filter->alpha = rc / (rc + dt);

    filter->prev_input.x = 0;
    filter->prev_input.y = 0;
    filter->prev_input.z = 0;

    filter->prev_output.x = 0;
    filter->prev_output.y = 0;
    filter->prev_output.z = 0;
}

//--------------------------------------------------------------------------------------------
void HighPassFilter_Apply(tsHighPassFilter *filter, tsVector3D *input,  tsVector3D *output)
{
	//tsVector3D output;
    output->x = filter->alpha * (filter->prev_output.x + input->x - filter->prev_input.x);
    output->y = filter->alpha * (filter->prev_output.y + input->y - filter->prev_input.y);
    output->z = filter->alpha * (filter->prev_output.z + input->z - filter->prev_input.z);

    filter->prev_input.x = input->x;
    filter->prev_input.y = input->y;
    filter->prev_input.z = input->z;

    filter->prev_output.x = output->x;
    filter->prev_output.y = output->y;
    filter->prev_output.z = output->z;

}

