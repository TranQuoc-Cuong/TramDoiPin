/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ina219_manager.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum{
	pin_empty,
	pin_charge,
	pin_full,
	pin_over_heat,
	pin_error
} StatusChannel;

typedef enum {
	INSTALL_BATTERY,
	ALL_SLOTS_EMPTY,
	IDLE_WAIT_CMD,
	
	WAIT_BATTERY_REMOVAL,
	
	SWAP_WAIT_INSERTION,
	SWAP_WAIT_ESP_PROCESS,         
	SWAP_ERROR_REMOVAL,     
	SWAP_WAIT_REMOVAL,
	
	INSTALL_WAIT_ESP_PROCESS,          
	INSTALL_ERROR_REMOVAL     
} SystemState;

typedef	struct{
	INA219Data       data_pin;
	float            temperatures; 
	StatusChannel    status;
	uint8_t          charge_done_counter;
	uint32_t         charge_done_timestamp;
	uint8_t          check_percent_pin;
	uint32_t         last_check_percent;
} PinChannel;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
