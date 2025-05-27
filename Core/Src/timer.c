/*******************************************************************************
 * @file    timer.c
 * @author  Eisa Aghchehli
 * @brief   Md DigitalEMG Master timer controller.
 * @Date	08/11/2023
  ******************************************************************************/

//Includes ------------------------------------------------------------------
#include <stdint.h>
#include <string.h>         // memcmp()
#include <stddef.h>         // NULL definition
#include <stdio.h>          // printf()

#include "stm32wbxx_hal.h"
#include "main.h"
#include "kernel.h"
#include "debug.h"
#include "hardware.h"
#include "LSM6DSLTR.h"
#include "AFE.h"
#include "timer.h"
#include "exti.h"




//typedef -------------------------------------------------------------------

			tsTIMER				tsTimer;
			tsTIMER				tsTimer1;
extern 		TIM_HandleTypeDef 	htim2;
extern 		TIM_HandleTypeDef 	htim1;
extern		tsDEBUG				tsDebug;
			tsAFE				AFE;
extern		tsIMU				IMU;
extern  	tsBLE				tsBle;
//defines --------------------------------------------------------------------



// variables -----------------------------------------------------------------

uint16_t i=0;


//functions prototypes--------------------------------------------------------


//Functions Definition -------------------------------------------------------



//------------------------------------------------------------------------------------------//
//                                        Initialisation                                    //
//------------------------------------------------------------------------------------------//
void Timer_Init (void)
{
	DIAG ("\t\tTimer initialised\r\n");
	HAL_Delay (10);
	// Start TIM2 in basic mode
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim1);
}

//------------------------------------------------------------------------------------------//
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{

	// Base timer @ 1ms
	if (htim->Instance == TIM2)
	{

//		tsTimer.bImuFlag = true;
		// Clear the timer interrupt flag
		__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);		  // Clear the timer interrupt flag
		tsTimer.u8TimeFlag = true;
//		tsTimer.bImuFlag = true;
		htim2.Init.Period = Interrupt_ms;

		//DIAG ("%d\r",  i++);
	}

	// Calculating AFE sampling rate
	if (htim->Instance == TIM1)
	{
		tsTimer1.u16Count++;

		if ( tsTimer1.u16Count > 1999)
		{
			tsTimer1.u16Count = 0;
			AFE.u16SPS = AFE.u16SampleCount;
			AFE.u16SampleCount =0;
			tsDebug.u8PacketCount[0]= tsBle.u8PacketCount[0];
			tsDebug.u8PacketCount[1]= tsBle.u8PacketCount[1];
			tsBle.u8PacketCount[0] = 0;
			tsBle.u8PacketCount[1] = 0;

			IMU.u16SPS = IMU.u16SampleCount;
			IMU.u16SampleCount =0;
		}
	}

}

//------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------//
// The End!
//------------------------------------------------------------------------------------------//

//------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------



