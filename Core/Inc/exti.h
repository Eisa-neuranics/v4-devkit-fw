/*
 * exti.h
 *
 *  Created on: June 2, 2024
 *      Author: nea79
 */

#ifndef __EXTI_H_
#define __EXTI_H_


//Includes ------------------------------------------------------------------
#include "LSM6DSLTR.h"
#include <stdint.h>
#include <string.h>         // memcmp()
#include <stddef.h>         // NULL definition
#include <stdio.h>          // printf()

#include "stm32wbxx_hal.h"
#include "main.h"
#include "AFE.h"
#include "hardware.h"




// Defines--------------------------------------------------------------------*/




/* Exported types ------------------------------------------------------------*/







//-----------------------------------------------------------------------
// Define the states of the state machines
//-----------------------------------------------------------------------



/* Exported constants --------------------------------------------------------*/


/* External variables --------------------------------------------------------*/



/* Exported functions prototypes ---------------------------------------------*/

extern	void	Exti_Init	( void );


#endif /* __EXTI_H_ */
