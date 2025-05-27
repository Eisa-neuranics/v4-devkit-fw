/*
 * hardware.h
 *
 *  Created on: Oct 5, 2023
 *      Author: nea79
 */

#ifndef INC_HARDWARE_H_
#define INC_HARDWARE_H_


#define 	WB55_USB
#define		WBxx_BLE



typedef enum { false = 0, true = 1 } bool;

//------------------------------------------------------------------------------
//IMU pins control
//------------------------------------------------------------------------------

/* Exported macro ------------------------------------------------------------*/
#define SET_RGB_COLOR(color)          \
do {                                  \
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, ((color) & 0x01	) 	? GPIO_PIN_RESET : GPIO_PIN_SET); 		/* Red   */	\
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, ((color) & 0x02	) 	? GPIO_PIN_RESET : GPIO_PIN_SET); 		/* Green */ \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, ((color) & 0x04 	) 	? GPIO_PIN_RESET : GPIO_PIN_SET); 		/* Blue  */ \
} while(0)

#endif /* INC_HARDWARE_H_ */
