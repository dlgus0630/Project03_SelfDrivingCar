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
/* CAN 수신 프레임 1개를 담는 구조체
   (수신 인터럽트가 채워 넣고, CanRxTask 가 꺼내서 화면에 출력한다) */
typedef struct
{
  uint32_t Id;       /* 프레임 식별자(ID) */
  uint8_t  IsExt;    /* 0 = 표준 11비트 ID, 1 = 확장 29비트 ID */
  uint8_t  Dlc;      /* 데이터 길이 (0~8 바이트) */
  uint8_t  Data[8];  /* 페이로드 8바이트 */
} CanRxFrame_t;
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
/* CAN 원형 버퍼에서 프레임을 하나 꺼낸다.
   반환값 1 = 하나 꺼냈음, 0 = 버퍼가 비어 있음 */
uint8_t  CanRx_Pop(CanRxFrame_t *out);

/* 부팅 후 지금까지 받은 CAN 프레임 총 개수 */
uint32_t CanRx_GetTotal(void);

/* 원형 버퍼가 가득 차서 버려진 프레임 개수 */
uint32_t CanRx_GetOverflow(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
