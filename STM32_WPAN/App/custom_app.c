/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include "ADS1293.h"
#include "kernel.h"
#include "debug.h"
#include "hardware.h"
#include "timer.h"
#include <Bluetooth.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* SENSOR_DATA */
  uint8_t               Accel_s_Notification_Status;
  uint8_t               Ecg_s_Notification_Status;
  uint8_t               All_sens_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

extern  tsBLE   	tsBle;
extern  tsCMD		tsCmd;
extern 	tsSTREAM	tsStreamAFE;
extern 	tsSTREAM	tsStreamIMU;
extern	tsDEBUG		tsDebug;


/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* SENSOR_DATA */
static void Custom_Accel_s_Update_Char(void);
static void Custom_Accel_s_Send_Notification(void);
static void Custom_Ecg_s_Update_Char(void);
static void Custom_Ecg_s_Send_Notification(void);
static void Custom_All_sens_Update_Char(void);
static void Custom_All_sens_Send_Notification(void);

/* USER CODE BEGIN PFP */


void ALL_SENs_Task (void)
{
	if ( tsBle.App.AfeSetTask )
	{
		tsBle.App.AfeSetTask = false;
		Custom_STM_App_Update_Char(CUSTOM_STM_ALL_SENS, (uint8_t*)(tsStreamAFE.u8TxData));
		tsBle.u8PacketCount[0]++;
	}
}
//--------------------------------------------------------------------------------------------

void ACCEL_Task(void)
{
	if ( tsBle.App.AccelSetTask )
	{
		tsBle.App.AccelSetTask = false;
		Custom_STM_App_Update_Char(CUSTOM_STM_ACCEL_S, (uint8_t*)(tsStreamIMU.u8TxData));
		tsBle.u8PacketCount[1]++;
	}
}



/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* SENSOR_DATA */
    case CUSTOM_STM_ACCEL_S_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ACCEL_S_NOTIFY_ENABLED_EVT */

    	tsBle.App.AfeNotification = true;
    	tsBle.App.AccelNotification = true;
    	DIAG (" BLE Debug ->\tAccel. Notification\t[ Enabled ]\n\r");

      /* USER CODE END CUSTOM_STM_ACCEL_S_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_ACCEL_S_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ACCEL_S_NOTIFY_DISABLED_EVT */


    	tsBle.App.AccelNotification = false;
    	DIAG (" BLE Debug ->\tAccel. Notification\t[ Disabled ]\n\r");

      /* USER CODE END CUSTOM_STM_ACCEL_S_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_ECG_S_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ECG_S_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_ECG_S_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_ECG_S_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ECG_S_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_ECG_S_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_CMD_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CMD_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_CMD_WRITE_EVT */
      break;

    case CUSTOM_STM_ALL_SENS_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ALL_SENS_NOTIFY_ENABLED_EVT */

    	tsBle.App.AfeNotification = true;
//    	tsBle.App.McgNotification = true;
    	DIAG (" BLE Debug ->\tMCG Notification\t[ Enabled ]\n\r");

      /* USER CODE END CUSTOM_STM_ALL_SENS_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_ALL_SENS_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ALL_SENS_NOTIFY_DISABLED_EVT */

    	tsBle.App.AfeNotification = false;
    	DIAG (" BLE Debug ->\tMCG Notification\t[ Disabled ]\n\r");

      /* USER CODE END CUSTOM_STM_ALL_SENS_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_DEVICE_INFO_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DEVICE_INFO_READ_EVT */

      /* USER CODE END CUSTOM_STM_DEVICE_INFO_READ_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* SENSOR_DATA */
void Custom_Accel_s_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Accel_s_UC_1*/

  /* USER CODE END Accel_s_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_ACCEL_S, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Accel_s_UC_Last*/

  /* USER CODE END Accel_s_UC_Last*/
  return;
}

void Custom_Accel_s_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Accel_s_NS_1*/

  DIAG ("Hmmm\n");
  /* USER CODE END Accel_s_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_ACCEL_S, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Accel_s_NS_Last*/

  /* USER CODE END Accel_s_NS_Last*/

  return;
}

void Custom_Ecg_s_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ecg_s_UC_1*/

  /* USER CODE END Ecg_s_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_ECG_S, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Ecg_s_UC_Last*/

  /* USER CODE END Ecg_s_UC_Last*/
  return;
}

void Custom_Ecg_s_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ecg_s_NS_1*/

  /* USER CODE END Ecg_s_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_ECG_S, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Ecg_s_NS_Last*/

  /* USER CODE END Ecg_s_NS_Last*/

  return;
}

void Custom_All_sens_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN All_sens_UC_1*/

  /* USER CODE END All_sens_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_ALL_SENS, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN All_sens_UC_Last*/

  /* USER CODE END All_sens_UC_Last*/
  return;
}

void Custom_All_sens_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN All_sens_NS_1*/

  /* USER CODE END All_sens_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_ALL_SENS, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN All_sens_NS_Last*/

  /* USER CODE END All_sens_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
