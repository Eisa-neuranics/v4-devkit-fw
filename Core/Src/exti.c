/*
 * exti.c
 *
 *  Created on: Jun 2, 2024
 *      Author: eisaa
 */


//Includes ------------------------------------------------------------------
#include "LSM6DSLTR.h"
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
#include "exti.h"



//typedef -------------------------------------------------------------------

extern	tsDEBUG			Debug;
extern	tsADS			ADS;
extern	tsIMU			IMU;

//defines --------------------------------------------------------------------

//#define		SYS_DEBUG_EN

// variables -----------------------------------------------------------------


//functions ------------------------------------------------------------------

void Exti_Init	( void )
{
	DIAG ("\t\tExti. initialised\r\n");
	HAL_Delay (10);
}

//----------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	//----------------------------------------------
	if (GPIO_Pin == ADS_DRDY_Pin)
	{
		ADS.DRDY = true;
		__HAL_GPIO_EXTI_CLEAR_IT(ADS_DRDY_Pin);
	}

	//----------------------------------------------
	if (GPIO_Pin == IMU_INT1_Pin)
	{
		IMU.bInt1 = true;
		__HAL_GPIO_EXTI_CLEAR_IT(IMU_INT1_Pin);
		DIAG (" IMU_INT1 \r\n");
	}

	//----------------------------------------------
	if (GPIO_Pin == IMU_INT2_Pin)
	{
		IMU.bInt2 = true;
		__HAL_GPIO_EXTI_CLEAR_IT(IMU_INT2_Pin);
		DIAG (" IMU_INT2 \r\n");
	}

}


//Functions Definition -------------------------------------------------------
