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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LR_Pin GPIO_PIN_7
#define LR_GPIO_Port GPIOE
#define LL_Pin GPIO_PIN_8
#define LL_GPIO_Port GPIOE
#define UL_Pin GPIO_PIN_9
#define UL_GPIO_Port GPIOE
#define UR_Pin GPIO_PIN_10
#define UR_GPIO_Port GPIOE
#define IN1_M1_Pin GPIO_PIN_8
#define IN1_M1_GPIO_Port GPIOD
#define IN2_M1_Pin GPIO_PIN_9
#define IN2_M1_GPIO_Port GPIOD
#define IN1_M2_Pin GPIO_PIN_10
#define IN1_M2_GPIO_Port GPIOD
#define IN2_M2_Pin GPIO_PIN_11
#define IN2_M2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
