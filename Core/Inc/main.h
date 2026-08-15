/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define MOTOR_ENA_L_Pin GPIO_PIN_0
#define MOTOR_ENA_L_GPIO_Port GPIOA
#define MOTOR_ENB_R_Pin GPIO_PIN_1
#define MOTOR_ENB_R_GPIO_Port GPIOA
#define MOTOR_IN1_Pin GPIO_PIN_2
#define MOTOR_IN1_GPIO_Port GPIOA
#define MOTOR_IN2_Pin GPIO_PIN_3
#define MOTOR_IN2_GPIO_Port GPIOA
#define MOTOR_IN3_Pin GPIO_PIN_4
#define MOTOR_IN3_GPIO_Port GPIOA
#define MOTOR_IN4_Pin GPIO_PIN_5
#define MOTOR_IN4_GPIO_Port GPIOA
#define TRIG_LEFT_Pin GPIO_PIN_0
#define TRIG_LEFT_GPIO_Port GPIOB
#define TRIG_FRONT_Pin GPIO_PIN_1
#define TRIG_FRONT_GPIO_Port GPIOB
#define TRIG_RIGHT_Pin GPIO_PIN_10
#define TRIG_RIGHT_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOB
#define LED_STATUS_Pin GPIO_PIN_13
#define LED_STATUS_GPIO_Port GPIOB
#define ECHO_LEFT_Pin GPIO_PIN_6
#define ECHO_LEFT_GPIO_Port GPIOB
#define ECHO_FRONT_Pin GPIO_PIN_7
#define ECHO_FRONT_GPIO_Port GPIOB
#define ECHO_RIGHT_Pin GPIO_PIN_8
#define ECHO_RIGHT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
