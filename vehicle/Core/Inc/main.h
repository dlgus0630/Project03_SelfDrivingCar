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
/* CAN 진단용 전역 변수의 extern 선언 (실체는 main.c의 USER CODE PV 블록에 있음)
 * mcal_can.c, freertos.c 등 다른 파일에서 이 변수들에 값을 채우기 위해 필요하다.
 * 각 변수의 의미와 ESR 비트 배치 설명은 main.c 선언부 주석을 참고할 것. */
extern volatile uint8_t  g_vdiag_can_tec;
extern volatile uint8_t  g_vdiag_can_rec;
extern volatile uint8_t  g_vdiag_can_lec;
extern volatile uint8_t  g_vdiag_can_boff;
extern volatile uint8_t  g_vdiag_can_epvf;
extern volatile uint8_t  g_vdiag_can_ewgf;
extern volatile uint32_t g_vdiag_can_esr_raw;
extern volatile uint32_t g_vdiag_can_tx_ok;
extern volatile uint32_t g_vdiag_can_tx_fail;
extern volatile uint32_t g_vdiag_can_free_mb;
extern volatile uint32_t g_vdiag_can_recover_cnt;
extern volatile uint8_t  g_vdiag_can_unrecoverable;
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
