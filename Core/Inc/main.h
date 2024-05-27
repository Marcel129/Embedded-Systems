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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SONAR_ECHO_Pin GPIO_PIN_5
#define SONAR_ECHO_GPIO_Port GPIOE
#define SONAR_ECHO_EXTI_IRQn EXTI9_5_IRQn
#define RGB_R_Pin GPIO_PIN_1
#define RGB_R_GPIO_Port GPIOA
#define RGB_G_Pin GPIO_PIN_2
#define RGB_G_GPIO_Port GPIOA
#define RGB_B_Pin GPIO_PIN_3
#define RGB_B_GPIO_Port GPIOA
#define SONAR_ENABLE_Pin GPIO_PIN_10
#define SONAR_ENABLE_GPIO_Port GPIOE
#define MOTOR_R_EN_Pin GPIO_PIN_6
#define MOTOR_R_EN_GPIO_Port GPIOC
#define MOTOR_R_PH_Pin GPIO_PIN_7
#define MOTOR_R_PH_GPIO_Port GPIOC
#define MOTOR_L_EN_Pin GPIO_PIN_8
#define MOTOR_L_EN_GPIO_Port GPIOC
#define MOTOR_L_PH_Pin GPIO_PIN_9
#define MOTOR_L_PH_GPIO_Port GPIOC
#define SONAR_TRIGGER_Pin GPIO_PIN_8
#define SONAR_TRIGGER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
