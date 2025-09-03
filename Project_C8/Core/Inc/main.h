/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define uchar unsigned char 
#define uint unsigned int
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define relay_hot_Pin GPIO_PIN_0
#define relay_hot_GPIO_Port GPIOA
#define relay_fan_Pin GPIO_PIN_1
#define relay_fan_GPIO_Port GPIOA
#define voice_Pin GPIO_PIN_6
#define voice_GPIO_Port GPIOA
#define alarm_Pin GPIO_PIN_15
#define alarm_GPIO_Port GPIOA
#define K1_Pin GPIO_PIN_3
#define K1_GPIO_Port GPIOB
#define K2_Pin GPIO_PIN_4
#define K2_GPIO_Port GPIOB
#define K3_Pin GPIO_PIN_5
#define K3_GPIO_Port GPIOB
#define lullabuy_Pin GPIO_PIN_6
#define lullabuy_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define relay_hot(a) (a?HAL_GPIO_WritePin(relay_hot_GPIO_Port, relay_hot_Pin, GPIO_PIN_SET):HAL_GPIO_WritePin(relay_hot_GPIO_Port, relay_hot_Pin, GPIO_PIN_RESET)) 
#define relay_fan(a) (a?HAL_GPIO_WritePin(relay_fan_GPIO_Port, relay_fan_Pin, GPIO_PIN_SET):HAL_GPIO_WritePin(relay_fan_GPIO_Port, relay_fan_Pin, GPIO_PIN_RESET)) 
#define alarm(a) (a?HAL_GPIO_WritePin(alarm_GPIO_Port, alarm_Pin, GPIO_PIN_SET):HAL_GPIO_WritePin(alarm_GPIO_Port, alarm_Pin, GPIO_PIN_RESET)) 
#define lullabuy(a) (a?HAL_GPIO_WritePin(lullabuy_GPIO_Port, lullabuy_Pin, GPIO_PIN_SET):HAL_GPIO_WritePin(lullabuy_GPIO_Port, lullabuy_Pin, GPIO_PIN_RESET)) 

#define voice HAL_GPIO_ReadPin(voice_GPIO_Port,voice_Pin)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
