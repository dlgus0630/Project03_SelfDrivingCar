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

/* ===== 디버거 관찰용 진단 변수 (실체는 main.c 에 있다) =====
   freertos.c 같은 다른 파일에서도 값을 채워 넣어야 해서
   여기에 extern 으로 "이런 변수가 어딘가에 있다"만 알려둔다.
   각 변수의 자세한 설명과 코드값 표는 main.c 의 선언부에 적어 두었다. */
extern volatile uint8_t  g_diag_i2c_found;        /* I2C 응답 장치 개수 */
extern volatile uint8_t  g_diag_i2c_addr[8];      /* 응답한 주소 목록 (최대 8개) */
extern volatile uint8_t  g_diag_imu_addr;         /* IMU 주소 (0x68 또는 0) */
extern volatile uint8_t  g_diag_who_am_i;         /* WHO_AM_I 값 (못 읽으면 0xFF) */
extern volatile uint8_t  g_diag_imu_kind;         /* 칩 판별 결과 코드 (0/1/2/3/4/9) */
extern volatile uint8_t  g_diag_scan_done;        /* I2C 스캔 완료 시 1 */
extern volatile uint32_t g_diag_can_total;        /* CAN 총 수신 개수 */
extern volatile uint32_t g_diag_can_last_id;      /* 마지막 프레임 ID */
extern volatile uint8_t  g_diag_can_last_dlc;     /* 마지막 프레임 데이터 길이 */
extern volatile uint8_t  g_diag_can_last_data[8]; /* 마지막 프레임 데이터 8바이트 */
extern volatile uint32_t g_diag_can_per_sec;      /* 직전 1초 동안 받은 개수 */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
