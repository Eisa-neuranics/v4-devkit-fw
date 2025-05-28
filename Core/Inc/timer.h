/*******************************************************************************
 * @file    timer.h
 * @author  Eisa Aghchehli
 * @brief   Md DigitalEMG Master timer controller.
 * @Date	08/11/2023
  ******************************************************************************/

#ifndef __TIMER_H_
#define __TIMER_H_



#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
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
//#include "BLE.h"

// Defines--------------------------------------------------------------------*/




/* Exported types ------------------------------------------------------------*/







//-----------------------------------------------------------------------
// Define the states of the state machines
//-----------------------------------------------------------------------
typedef struct
{
    volatile uint16_t 	u16TimeValue;
    volatile uint8_t 	u8TimeFlag;

    volatile bool		bImuFlag;
    volatile bool		bAfeFlag;

    volatile uint16_t 	u16TimeStamp;
    volatile uint16_t  	u16Count;

    volatile uint16_t 	u16PwrCount;


} tsTIMER;


/* Exported constants --------------------------------------------------------*/


/* External variables --------------------------------------------------------*/



/* Exported functions prototypes ---------------------------------------------*/
extern 	void 	Timer_Init 						(void);
		void 	HAL_TIM_PeriodElapsedCallback	( TIM_HandleTypeDef* htim );


#endif /* __TIMER_H_ */
