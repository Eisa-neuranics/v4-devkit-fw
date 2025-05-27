/*
 * ADS1293.h
 *
 *  Created on: Sep 3, 2023
 *      Author: nea79
 */

#ifndef ADS1293_H
#define ADS1293_H


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wbxx_hal.h"
#include "main.h"
#include "kernel.h"
#include "debug.h"
#include "hardware.h"
#include "IIR_Filter.h"
#include "LSM6DSLTR.h"
#include "AFE.h"
#include "timer.h"
#include "exti.h"
//------------------------------------------------------------------------------

extern 	void 	ADS1293_Init 				(void);
		void 	ADS1293_PWR					(uint8_t state);
extern 	uint8_t ADS1293_SPIReadReg			(uint8_t addr);
		void 	ADS1293_SPIWriteReg			(uint8_t addr, uint8_t value);
		void 	ADS1293_SPIAutoIncWriteReg	(uint8_t addr, uint8_t *buffer, uint8_t count);
		void 	ADS1293_SPIAutoIncReadReg	(uint8_t addr, uint8_t *buffer, uint8_t count);
extern 	void 	ADS1293_SPIStreamReadReg	(uint8_t *buffer, uint8_t count);


//----------------------MACROS--------------------------------


//#define 		ADS_DEBUG_EN




#define ENABLE_CS			GPIOA->BSRR = GPIO_BSRR_BR1;
#define DISABLE_CS			GPIOA->BSRR = GPIO_BSRR_BS1;

#define ENABLE_RSTB_ADS		GPIOA->BSRR = GPIO_BSRR_BR0;
#define DISABLE_RSTB_ADS	GPIOA->BSRR = GPIO_BSRR_BS0;

//-----------------------Typedef/enum------------------------
enum eR2_RATE
{
	R2_4 = 0x01,
	R2_5 = 0x02,
	R2_6 = 0x04,
	R2_8 = 0x08
};


enum eR3_RATE
{
	R3_4  = 0b00000001,
	R3_6  = 0b00000010,
	R3_8  = 0b00000100,
	R3_12 = 0b00001000,
	R3_16 = 0b00010000,
	R3_32 = 0b00100000,
	R3_64 = 0b01000000,
	R3_128= 0b10000000

};

//-----------------------Defenition--------------------------
#define ADS1293_ID									(0x01)

#define ADC_MAX										(0xC35000)				// 0xB964F0	0xC35000
#define ODR											(640)					// SPS
//#define BW											(175)					// Hz
#define ADC_REF										(2.4)					// V
#define ADC_GAIN									(3.5)					// x
/************************************************************
* TI ADS1293 REGISTER SET ADDRESSES
************************************************************/

#define ADS1293_CONFIG_REG                          (0x00)                   /* Main Configuration */

#define ADS1293_FLEX_CH1_CN_REG                     (0x01)                   /* Flex Routing Swich Control for Channel 1 */
#define ADS1293_FLEX_CH2_CN_REG                     (0x02)                   /* Flex Routing Swich Control for Channel 2 */
#define ADS1293_FLEX_CH3_CN_REG                     (0x03)                   /* Flex Routing Swich Control for Channel 3 */
#define ADS1293_FLEX_PACE_CN_REG                    (0x04)                   /* Flex Routing Swich Control for Pace Channel */
#define ADS1293_FLEX_VBAT_CN_REG                    (0x05)                   /* Flex Routing Swich Control for Battery Monitoriing */

#define ADS1293_LOD_CN_REG                          (0x06)                   /* Lead Off Detect Control */
#define ADS1293_LOD_EN_REG                          (0x07)                   /* Lead Off Detect Enable */
#define ADS1293_LOD_CURRENT_REG                     (0x08)                   /* Lead Off Detect Current */
#define ADS1293_LOD_AC_CN_REG                       (0x09)                   /* AC Lead Off Detect Current */

#define ADS1293_CMDET_EN_REG                        (0x0A)                   /* Common Mode Detect Enable */
#define ADS1293_CMDET_CN_REG                        (0x0B)                   /* Commond Mode Detect Control */
#define ADS1293_RLD_CN_REG                          (0x0C)                   /* Right Leg Drive Control */

#define ADS1293_WILSON_EN1_REG                      (0x0D)                   /* Wilson Reference Input one Selection */
#define ADS1293_WILSON_EN2_REG                      (0x0E)                   /* Wilson Reference Input two Selection */
#define ADS1293_WILSON_EN3_REG                      (0x0F)                   /* Wilson Reference Input three Selection */
#define ADS1293_WILSON_CN_REG                       (0x10)                   /* Wilson Reference Input Control */

#define ADS1293_REF_CN_REG                          (0x11)                   /* Internal Reference Voltage Control */

#define ADS1293_OSC_CN_REG                          (0x12)                   /* Clock Source and Output Clock Control */

#define ADS1293_AFE_RES_REG                         (0x13)                   /* Analog Front-End Frequency and Resolution */
#define ADS1293_AFE_SHDN_CN_REG                     (0x14)                   /* Analog Front-End Shutdown Control */
#define ADS1293_AFE_FAULT_CN_REG                    (0x15)                   /* Analog Front-End Fault Detection Control */
#define ADS1293_AFE_DITHER_EN_REG                   (0x16)                   /* Enable Dithering in Signma-Delta */
#define ADS1293_AFE_PACE_CN_REG                     (0x17)                   /* Analog Pace Channel Output Routing Control */

#define ADS1293_ERROR_LOD_REG                       (0x18)                   /* Lead Off Detect Error Status */
#define ADS1293_ERROR_STATUS_REG                    (0x19)                   /* Other Error Status */
#define ADS1293_ERROR_RANGE1_REG                    (0x1A)                   /* Channel 1 Amplifier Out of Range Status */
#define ADS1293_ERROR_RANGE2_REG                    (0x1B)                   /* Channel 1 Amplifier Out of Range Status */
#define ADS1293_ERROR_RANGE3_REG                    (0x1C)                   /* Channel 1 Amplifier Out of Range Status */
#define ADS1293_ERROR_SYNC_REG                      (0x1D)                   /* Synchronization Error */


#define ADS1293_R2_RATE_REG                         (0x21)                   /* R2 Decimation Rate */
#define ADS1293_R3_RATE1_REG                        (0x22)                   /* R3 Decimation Rate for Channel 1 */
#define ADS1293_R3_RATE2_REG                        (0x23)                   /* R3 Decimation Rate for Channel 2 */
#define ADS1293_R3_RATE3_REG                        (0x24)                   /* R3 Decimation Rate for Channel 3 */
#define ADS1293_P_DRATE_REG                         (0x25)                   /* 2x Pace Data Rate */
#define ADS1293_DIS_EFILTER_REG                     (0x26)                   /* ECG Filter Disable */
#define ADS1293_DRDYB_SRC_REG                       (0x27)                   /* Data Ready Pin Source */
#define ADS1293_SYNCOUTB_SRC_REG                    (0x28)                   /* Sync Out Pin Source */
#define ADS1293_MASK_DRDYB_REG                      (0x29)                   /* Optional Mask Control for DRDYB Output */
#define ADS1293_MASK_ERR_REG                        (0x2A)                   /* Mask Error on ALARMB Pin */

#define ADS1293_ALARM_FILTER_REG                    (0x2E)                   /* Digital Filter for Analog Alarm Signals */
#define ADS1293_CH_CNFG_REG                         (0x2F)                   /* Configure Channel for Loop Read Back Mode */

#define ADS1293_DATA_STATUS_REG                     (0x30)                   /* ECG and Pace Data Ready Status */
#define ADS1293_DATA_CH1_PACE_H_REG                 (0x31)                   /* Channel1 Pace Data High [15:8] */
#define ADS1293_DATA_CH1_PACE_L_REG                 (0x32)                   /* Channel1 Pace Data Low [7:0] */
#define ADS1293_DATA_CH2_PACE_H_REG                 (0x33)                   /* Channel2 Pace Data High [15:8] */
#define ADS1293_DATA_CH2_PACE_L_REG                 (0x34)                   /* Channel2 Pace Data Low [7:0] */
#define ADS1293_DATA_CH3_PACE_H_REG                 (0x35)                   /* Channel3 Pace Data High [15:8] */
#define ADS1293_DATA_CH3_PACE_L_REG                 (0x36)                   /* Channel3 Pace Data Low [7:0] */
#define ADS1293_DATA_CH1_ECG_H_REG                  (0x37)                   /* Channel1 ECG Data High [23:16] */
#define ADS1293_DATA_CH1_ECG_M_REG                  (0x38)                   /* Channel1 ECG Data Medium [15:8] */
#define ADS1293_DATA_CH1_ECG_L_REG					(0x39)                   /* Channel1 ECG Data Low [7:0] */
#define ADS1293_DATA_CH2_ECG_H_REG                	(0x3A)                   /* Channel2 ECG Data High [23:16] */
#define ADS1293_DATA_CH2_ECG_M_REG                 	(0x3B)                   /* Channel2 ECG Data Medium [15:8] */
#define ADS1293_DATA_CH2_ECG_L_REG                 	(0x3C)                   /* Channel2 ECG Data Low [7:0] */
#define ADS1293_DATA_CH3_ECG_H_REG                 	(0x3D)                   /* Channel3 ECG Data High [23:16] */
#define ADS1293_DATA_CH3_ECG_M_REG                 	(0x3E)                   /* Channel3 ECG Data Medium [15:8] */
#define ADS1293_DATA_CH3_ECG_L_REG                 	(0x3F)                   /* Channel3 ECG Data Low [7:0] */

#define ADS1293_REVID_REG                          	(0x40)                   /* Revision ID */
#define ADS1293_DATA_LOOP_REG                      	(0x50)                   /* Loop Read Back Address */


// Useful definitions
#define ADS1293_READ_BIT                           	(0x80)
#define ADS1293_WRITE_BIT                          	(0x7F)

#define ADS_STOP									(0x00)
#define	ADS_START									(0x01)
#define ADS_STANDBY									(0x02)
#define	ADS_PWR_DOWN								(0x04)

#define CH_Disable									(0x00)
#define Circuit_PWR_DWON							(0x01)











#endif /* ADS1293_H_ */
