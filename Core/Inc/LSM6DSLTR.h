/**
 ******************************************************************************
 * @file    LSM6DSLTR.h
 * @author  Eisa Aghchehli
 * @brief   This file provides a set of functions needed to manage the LSM6DSL
 *          accelero and gyro devices
 ******************************************************************************/

#ifndef LSM6DSLTR_H
#define LSM6DSLTR_H

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
#include "hardware.h"
#include "kernel.h"
#include "IIR_Filter.h"

#define	Accelerometer				0x00
#define	Gyroscope					0x01

//#define	IMU_ENABLE
#define	ACCEL_ENABLE
#define	GYRO_ENABLE
//#define		IMU_DEBUG_EN

#define	IMU_Data_Length				64

#define	I2C_DELAY					10

#define ACCEL_SENSITIVITY 			16384.0 	// Sensitivity for ±2g (change based on your config)
#define GYRO_SENSITIVITY 			131.0    	// Sensitivity for ±250°/s (change based on your config)

// Define the I2C address for the LSM6DSLTR
#define IMU_ADDR 					0x6A << 1 	// Shifted left because HAL expects the 7-bit address

// IMU register addresses
#define IMU_FUNC_CFG_ACCESS        	0x01
#define IMU_SENSOR_SYNC_TIME       	0x04
#define IMU_SENSOR_RES_RATIO       	0x05
#define IMU_FIFO_CTRL1             	0x06
#define IMU_FIFO_CTRL2             	0x07
#define IMU_FIFO_CTRL3             	0x08
#define IMU_FIFO_CTRL4             	0x09
#define IMU_FIFO_CTRL5             	0x0A
#define IMU_DRDY_PULSE_CFG_G       	0x0B
#define IMU_INT1_CTRL              	0x0D
#define IMU_INT2_CTRL              	0x0E
#define IMU_WHO_AM_I               	0x0F
#define IMU_CTRL1_XL               	0x10
#define IMU_CTRL2_G                	0x11
#define IMU_CTRL3_C                	0x12
#define IMU_CTRL4_C                	0x13
#define IMU_CTRL5_C                	0x14
#define IMU_CTRL6_C                	0x15
#define IMU_CTRL7_G                	0x16
#define IMU_CTRL8_XL               	0x17
#define IMU_CTRL9_XL               	0x18
#define IMU_CTRL10_C               	0x19
#define IMU_MASTER_CONFIG          	0x1A
#define IMU_WAKE_UP_SRC            	0x1B
#define IMU_TAP_SRC                	0x1C
#define IMU_D6D_SRC                	0x1D
#define IMU_STATUS_REG             	0x1E
#define IMU_OUT_TEMP_L             	0x20
#define IMU_OUT_TEMP_H             	0x21
#define IMU_OUTX_L_G               	0x22
#define IMU_OUTX_H_G               	0x23
#define IMU_OUTY_L_G               	0x24
#define IMU_OUTY_H_G               	0x25
#define IMU_OUTZ_L_G               	0x26
#define IMU_OUTZ_H_G               	0x27
#define IMU_OUTX_L_XL              	0x28
#define IMU_OUTX_H_XL              	0x29
#define IMU_OUTY_L_XL              	0x2A
#define IMU_OUTY_H_XL              	0x2B
#define IMU_OUTZ_L_XL              	0x2C
#define IMU_OUTZ_H_XL              	0x2D
#define IMU_SENSORHUB1_REG         	0x2E
#define IMU_SENSORHUB2_REG         	0x2F
#define IMU_SENSORHUB3_REG         	0x30
#define IMU_SENSORHUB4_REG         	0x31
#define IMU_SENSORHUB5_REG         	0x32
#define IMU_SENSORHUB6_REG         	0x33
#define IMU_SENSORHUB7_REG         	0x34
#define IMU_SENSORHUB8_REG         	0x35
#define IMU_SENSORHUB9_REG         	0x36
#define IMU_SENSORHUB10_REG        	0x37
#define IMU_SENSORHUB11_REG        	0x38
#define IMU_SENSORHUB12_REG        	0x39
#define IMU_FIFO_STATUS1           	0x3A
#define IMU_FIFO_STATUS2           	0x3B
#define IMU_FIFO_STATUS3           	0x3C
#define IMU_FIFO_STATUS4           	0x3D
#define IMU_FIFO_DATA_OUT_L        	0x3E
#define IMU_FIFO_DATA_OUT_H        	0x3F
#define IMU_TIMESTAMP0_REG         	0x40
#define IMU_TIMESTAMP1_REG         	0x41
#define IMU_TIMESTAMP2_REG         	0x42
#define IMU_STEP_TIMESTAMP_L       	0x49
#define IMU_STEP_TIMESTAMP_H       	0x4A
#define IMU_STEP_COUNTER_L         	0x4B
#define IMU_STEP_COUNTER_H         	0x4C
#define IMU_SENSORHUB13_REG        	0x4D
#define IMU_SENSORHUB14_REG        	0x4E
#define IMU_SENSORHUB15_REG        	0x4F
#define IMU_SENSORHUB16_REG        	0x50
#define IMU_SENSORHUB17_REG        	0x51
#define IMU_SENSORHUB18_REG        	0x52
#define IMU_FUNC_SRC1              		0x53
#define IMU_FUNC_SRC2              		0x54
#define IMU_WRIST_TILT_IA          		0x55
#define IMU_TAP_CFG                		0x58
#define IMU_TAP_THS_6D             		0x59
#define IMU_INT_DUR2               		0x5A
#define IMU_WAKE_UP_THS            		0x5B
#define IMU_WAKE_UP_DUR            		0x5C
#define IMU_FREE_FALL              		0x5D
#define IMU_MD1_CFG                		0x5E
#define IMU_MD2_CFG                		0x5F
#define IMU_MASTER_CMD_CODE        		0x60
#define IMU_SENS_SYNC_SPI_ERROR_CODE 	0x61
#define IMU_OUT_MAG_RAW_X_L        		0x66
#define IMU_OUT_MAG_RAW_X_H        		0x67
#define IMU_OUT_MAG_RAW_Y_L        		0x68
#define IMU_OUT_MAG_RAW_Y_H        		0x69
#define IMU_OUT_MAG_RAW_Z_L        		0x6A
#define IMU_OUT_MAG_RAW_Z_H        		0x6B
#define IMU_INT_OIS                		0x6F
#define IMU_CTRL1_OIS              		0x70
#define IMU_CTRL2_OIS              		0x71
#define IMU_CTRL3_OIS              		0x72
#define IMU_X_OFS_USR              		0x73
#define IMU_Y_OFS_USR              		0x74
#define IMU_Z_OFS_USR              		0x75


//------------------------------------------------------------------------------

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;

} tsVector3D;

typedef struct
{
    float alpha;
    tsVector3D prev_input;
    tsVector3D prev_output;

} tsHighPassFilter;

typedef struct
{
	int16_t 	u16AcceD[3];
	int16_t 	u16GyroD[3];

	uint8_t		u8AxTxLen;
	uint8_t		u8AyTxLen;
	uint8_t		u8AzTxLen;

	uint8_t		u8GxTxLen;
	uint8_t		u8GyTxLen;
	uint8_t		u8GzTxLen;

	uint8_t 	u8AxTxData[72];
	uint8_t 	u8AyTxData[72];
	uint8_t 	u8AzTxData[72];

	uint8_t 	u8GxTxData[72];
	uint8_t 	u8GyTxData[72];
	uint8_t 	u8GzTxData[72];

	volatile uint16_t 	u16SampleCount;
	volatile uint16_t 	u16SPS;

	bool		bInt1;
	bool		bInt2;
} tsIMU;

typedef enum
{
	IMU_STATE_INIT					= 0,
	IMU_STATE_IDLE,
	IMU_STATE_WAIT_INT,
	IMU_STATE_SELECT_SLAVE,
	IMU_STATE_REQ_ACCEL,
	IMU_STATE_REQ_GYRO,
	IMU_STATE_DESELECT_SLAVE,
	IMU_STATE_PROCESS_DATA,
	IMU_STATE_RELEASE_DATA,

	IMU_STATE_ERROR
}teIMU_STATE;

//------------------------------------------------------------------------------
//LSM6DSLTR pins control
//------------------------------------------------------------------------------

#define 	ENABLE_IMU_CS			GPIOA->BSRR = GPIO_BSRR_BS7;
#define 	DISABLE_IMU_CS			GPIOA->BSRR = GPIO_BSRR_BR7;

#define 	ADDR_IMU_6B				GPIOB->BSRR = GPIO_BSRR_BS2;
#define 	ADDR_IMU_6A				GPIOB->BSRR = GPIO_BSRR_BR2;

//------------------------------------------------------------------------------

extern	HAL_StatusTypeDef 	IMU_Init					( I2C_HandleTypeDef *hi2c );
extern	void				IMU_Control					( void );
		HAL_StatusTypeDef 	IMU_ReadID					( I2C_HandleTypeDef *hi2c, uint8_t *id );
		HAL_StatusTypeDef 	IMU_ReadAccel				( I2C_HandleTypeDef *hi2c, tsVector3D *pData );
		HAL_StatusTypeDef 	IMU_ReadGyro				( I2C_HandleTypeDef *hi2c, tsVector3D *pData );
extern	void 				IMU_StreamDataPack 			( uint8_t sensor, uint8_t *x_pData, uint8_t *y_pData, uint8_t *z_pData, uint8_t *outputString );
		void				IMU_Error					( void );

		void 				HighPassFilter_Init		( tsHighPassFilter *filter, float cutoff_freq, float sampling_rate );
		void 				HighPassFilter_Apply	( tsHighPassFilter *filter, tsVector3D *input, tsVector3D *output );
//------------------------------------------------------------------------------


#endif // LSM6DSLTR_H
