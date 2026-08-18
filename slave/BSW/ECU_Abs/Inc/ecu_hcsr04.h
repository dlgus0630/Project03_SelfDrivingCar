/*
 * ecu_hcsr04.h
 *
 *  Layer  : BSW / ECU_Abs (HW 구동부)
 *  Module : ECU_HCSR04
 *  Desc   : TIM4 Input Capture 기반 초음파 센서(좌/전/우) 3채널 거리 측정
 *
 *  HW 매핑
 *    ECHO_LEFT  -> PB6  TIM4_CH1
 *    ECHO_FRONT -> PB7  TIM4_CH2
 *    ECHO_RIGHT -> PB8  TIM4_CH3
 *    TRIG_LEFT  -> PB0
 *    TRIG_FRONT -> PB1
 *    TRIG_RIGHT -> PB10
 *
 *  주의: 여기 정의된 값은 센서의 "물리적 하드웨어 한계값"만 포함한다.
 *        위험/경고 등 "주행 판단 임계값"은 Application 계층
 *        (asw_autonomous.h)에서 관리한다. (Driver/App 책임 분리)
 *
 *  Naming Rule : ECU_HCSR04_<Verb><Object>()
 */

#ifndef __ECU_HCSR04_H
#define __ECU_HCSR04_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include "tim.h"

/* ------------------------------------------------------------------------
 * TRIG GPIO
 * ------------------------------------------------------------------------ */
#define ECU_HCSR04_TRIG_LEFT_PORT       GPIOB
#define ECU_HCSR04_TRIG_LEFT_PIN        GPIO_PIN_0
#define ECU_HCSR04_TRIG_FRONT_PORT      GPIOB
#define ECU_HCSR04_TRIG_FRONT_PIN       GPIO_PIN_1
#define ECU_HCSR04_TRIG_RIGHT_PORT      GPIOB
#define ECU_HCSR04_TRIG_RIGHT_PIN       GPIO_PIN_10

/* ------------------------------------------------------------------------
 * ECHO: TIM4 Input Capture (3채널 통합)
 * ------------------------------------------------------------------------ */
#define ECU_HCSR04_TIM_HANDLE           htim4

#define ECU_HCSR04_CH_LEFT              TIM_CHANNEL_1
#define ECU_HCSR04_CH_LEFT_IDX          HAL_TIM_ACTIVE_CHANNEL_1
#define ECU_HCSR04_IT_LEFT              TIM_IT_CC1

#define ECU_HCSR04_CH_FRONT             TIM_CHANNEL_2
#define ECU_HCSR04_CH_FRONT_IDX         HAL_TIM_ACTIVE_CHANNEL_2
#define ECU_HCSR04_IT_FRONT             TIM_IT_CC2

#define ECU_HCSR04_CH_RIGHT             TIM_CHANNEL_3
#define ECU_HCSR04_CH_RIGHT_IDX         HAL_TIM_ACTIVE_CHANNEL_3
#define ECU_HCSR04_IT_RIGHT             TIM_IT_CC3

/* ------------------------------------------------------------------------
 * 센서 물리적 한계값 (Driver 고유 스펙)
 * ------------------------------------------------------------------------ */
#define ECU_HCSR04_DIST_MAX_CM          400u    /* 센서 최대 유효 거리 */
#define ECU_HCSR04_ECHO_TIMEOUT_US      38000u  /* ECHO 응답 타임아웃 */

/* ------------------------------------------------------------------------
 * 센서 인덱스
 * ------------------------------------------------------------------------ */
typedef enum {
    ECU_HCSR04_SENSOR_LEFT  = 0,
    ECU_HCSR04_SENSOR_FRONT = 1,
    ECU_HCSR04_SENSOR_RIGHT = 2,
    ECU_HCSR04_SENSOR_NUM   = 3
} EcuHcsr04Index_t;

/* ------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------ */
void     ECU_HCSR04_Init(void);
void     ECU_HCSR04_TriggerSensor(EcuHcsr04Index_t sensor);
void     ECU_HCSR04_TriggerAllSensors(void);
uint32_t ECU_HCSR04_GetDistanceCm(EcuHcsr04Index_t sensor);
/* 아직 소비되지 않은 새 측정 결과가 있는지 조회만 한다(GetDistanceCm과 달리 소비하지
 * 않음). TrigTask가 고정 딜레이 대신 실제 에코가 돌아온 즉시 다음 센서로 넘어가게
 * 하는 데 사용한다 - 가까운 장애물일수록 왕복시간이 짧아 더 빨리 갱신된다. */
uint8_t  ECU_HCSR04_IsReady(EcuHcsr04Index_t sensor);

/* HAL_TIM_IC_CaptureCallback (main.c USER CODE BEGIN 4) 에서 반드시 호출 */
void     ECU_HCSR04_HandleCaptureIsr(TIM_HandleTypeDef *p_htim);

#endif /* __ECU_HCSR04_H */
