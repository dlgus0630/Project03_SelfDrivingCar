/*
 * ecu_l298n.h
 *
 *  Layer  : BSW / ECU_Abs (HW 구동부)
 *  Module : ECU_L298N
 *  Desc   : TIM2 PWM 기반 DC 모터(좌/우) 속도 및 방향 제어
 *
 *  HW 매핑
 *    TIM2_CH1 (PA0-WKUP) -> MOTOR_ENA_L (좌측 모터 PWM)
 *    TIM2_CH2 (PA1)      -> MOTOR_ENB_R (우측 모터 PWM)
 *    PA2 / PA3           -> 좌측 모터 방향 (IN1 / IN2)
 *    PA4 / PA5           -> 우측 모터 방향 (IN3 / IN4)
 *
 *  Naming Rule : ECU_L298N_<Verb><Object>()
 */

#ifndef __ECU_L298N_H
#define __ECU_L298N_H

#include "main.h"
#include "tim.h"

/* ------------------------------------------------------------------------
 * PWM / GPIO 하드웨어 정의
 * ------------------------------------------------------------------------ */
#define ECU_L298N_TIM_HANDLE        htim2
#define ECU_L298N_CH_LEFT           TIM_CHANNEL_1
#define ECU_L298N_CH_RIGHT          TIM_CHANNEL_2

#define ECU_L298N_MAX_DUTY          999u   /* TIM2 Period (1kHz PWM) */
#define ECU_L298N_DEFAULT_DUTY      700u   /* 기본 속도 70% */
#define ECU_L298N_TURN_DUTY         500u   /* 회전 속도 50% */
#define ECU_L298N_PIVOT_DUTY        500u   /* 제자리 피벗 회전 속도 50% */

#define ECU_L298N_IN1_PORT          GPIOA
#define ECU_L298N_IN1_PIN           GPIO_PIN_2
#define ECU_L298N_IN2_PORT          GPIOA
#define ECU_L298N_IN2_PIN           GPIO_PIN_3
#define ECU_L298N_IN3_PORT          GPIOA
#define ECU_L298N_IN3_PIN           GPIO_PIN_4
#define ECU_L298N_IN4_PORT          GPIOA
#define ECU_L298N_IN4_PIN           GPIO_PIN_5

/* ------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------ */
void ECU_L298N_Init(void);
void ECU_L298N_DriveForward(uint32_t duty);
void ECU_L298N_DriveBackward(uint32_t duty);
void ECU_L298N_TurnLeft(uint32_t duty);
void ECU_L298N_TurnRight(uint32_t duty);
void ECU_L298N_Stop(void);
void ECU_L298N_SetSpeed(uint32_t left_duty, uint32_t right_duty);

/* 좌/우 바퀴에 서로 다른 duty로 "전진" (통로 중앙 유지용 비례 조향).
 * 두 채널 모두 전진 방향이며 속도차만 두므로 급격한 회전은 발생하지 않음. */
void ECU_L298N_DriveForwardDifferential(uint16_t leftDuty, uint16_t rightDuty);

/* 좌/우 바퀴에 서로 다른 duty로 "후진" (전진용 차등 조향의 후진 버전).
 * 두 채널 모두 후진 방향이며 속도차만 두므로 급격한 회전은 발생하지 않음. */
void ECU_L298N_DriveBackwardDifferential(uint16_t leftDuty, uint16_t rightDuty);

/* 진짜 제자리 피벗 회전: 한쪽 바퀴는 전진, 반대쪽 바퀴는 후진으로 구동하여
 * 차체가 코너 쪽으로 전진하며 쓸리지 않고 그 자리에서 회전하도록 함. */
void ECU_L298N_PivotLeft(uint32_t duty);
void ECU_L298N_PivotRight(uint32_t duty);

/* 급제동(Active Brake): IN1=IN2(같은 레벨) + EN(PWM) 최대 duty로 L298N 내부에서
 * 모터 단자를 단락시켜 강제 감속시킨다. ECU_L298N_Stop()은 duty=0(EN LOW)이라
 * 그냥 전원만 끊는 자유회전(coast)이라 관성으로 계속 미끄러지므로, 벽 충돌
 * 직전처럼 즉시 감속이 필요한 상황에서는 이 함수를 사용한다. */
void ECU_L298N_Brake(void);

#endif /* __ECU_L298N_H */
