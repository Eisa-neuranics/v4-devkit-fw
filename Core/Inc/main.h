/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADS_RSTB_Pin GPIO_PIN_0
#define ADS_RSTB_GPIO_Port GPIOA
#define ADS_CS_Pin GPIO_PIN_1
#define ADS_CS_GPIO_Port GPIOA
#define IMU_INT1_Pin GPIO_PIN_4
#define IMU_INT1_GPIO_Port GPIOA
#define IMU_INT1_EXTI_IRQn EXTI4_IRQn
#define PWR_SW_Pin GPIO_PIN_5
#define PWR_SW_GPIO_Port GPIOA
#define PWR_SW_EXTI_IRQn EXTI9_5_IRQn
#define IMU_CS_Pin GPIO_PIN_6
#define IMU_CS_GPIO_Port GPIOA
#define IMU_ADDR_Pin GPIO_PIN_2
#define IMU_ADDR_GPIO_Port GPIOB
#define RGB_B_Pin GPIO_PIN_0
#define RGB_B_GPIO_Port GPIOB
#define RGB_G_Pin GPIO_PIN_1
#define RGB_G_GPIO_Port GPIOB
#define RGB_R_Pin GPIO_PIN_4
#define RGB_R_GPIO_Port GPIOE
#define PWR_ON_Pin GPIO_PIN_15
#define PWR_ON_GPIO_Port GPIOA
#define ADS_ALARMB_Pin GPIO_PIN_6
#define ADS_ALARMB_GPIO_Port GPIOB
#define ADS_DRDY_Pin GPIO_PIN_7
#define ADS_DRDY_GPIO_Port GPIOB
#define ADS_DRDY_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
