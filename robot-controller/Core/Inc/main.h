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
#include "stm32f1xx_hal.h"

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

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LASER_Pin GPIO_PIN_13
#define LASER_GPIO_Port GPIOC
#define CS_BOT_Pin GPIO_PIN_14
#define CS_BOT_GPIO_Port GPIOC
#define CS_TOP_Pin GPIO_PIN_15
#define CS_TOP_GPIO_Port GPIOC
#define ISR_DUTY_Pin GPIO_PIN_1
#define ISR_DUTY_GPIO_Port GPIOA
#define PWM_BOT_A_Pin GPIO_PIN_7
#define PWM_BOT_A_GPIO_Port GPIOA
#define PWM_BOT_B_Pin GPIO_PIN_0
#define PWM_BOT_B_GPIO_Port GPIOB
#define IN_BOT_1_Pin GPIO_PIN_1
#define IN_BOT_1_GPIO_Port GPIOB
#define IN_BOT_2_Pin GPIO_PIN_2
#define IN_BOT_2_GPIO_Port GPIOB
#define PWM_TOP_B_Pin GPIO_PIN_6
#define PWM_TOP_B_GPIO_Port GPIOB
#define PWM_TOP_A_Pin GPIO_PIN_7
#define PWM_TOP_A_GPIO_Port GPIOB
#define IN_TOP_2_Pin GPIO_PIN_8
#define IN_TOP_2_GPIO_Port GPIOB
#define IN_TOP_1_Pin GPIO_PIN_9
#define IN_TOP_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
