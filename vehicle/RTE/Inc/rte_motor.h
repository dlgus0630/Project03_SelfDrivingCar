/*
 * rte_motor.h
 *
 *  Layer  : RTE (Runtime Environment)
 *  Module : RTE_MOTOR
 *  Desc   : ASW <-> ECU_L298N 모터 드라이버 사이의 1:1 브릿지.
 *           ASW 계층은 HAL/드라이버를 직접 참조하지 않고 본 RTE API만 호출한다.
 *
 *  Naming Rule : RTE_Motor_<Verb><Object>()
 */
#ifndef __RTE_MOTOR_H
#define __RTE_MOTOR_H

#include "ecu_l298n.h"

/* ------------------------------------------------------------------------
 * Public API (ECU_L298N 1:1 pass-through)
 * ------------------------------------------------------------------------ */
void     RTE_Motor_Init(void);
void     RTE_Motor_SetSpeed(uint32_t left_duty, uint32_t right_duty);
void     RTE_Motor_DriveForward(uint32_t duty);
void     RTE_Motor_DriveBackward(uint32_t duty);
void     RTE_Motor_TurnLeft(uint32_t duty);
void     RTE_Motor_TurnRight(uint32_t duty);
void     RTE_Motor_Stop(void);
void     RTE_Motor_Brake(void);
void     RTE_Motor_DriveForwardDifferential(uint16_t leftDuty, uint16_t rightDuty);
void     RTE_Motor_DriveBackwardDifferential(uint16_t leftDuty, uint16_t rightDuty);
void     RTE_Motor_PivotLeft(uint32_t duty);
void     RTE_Motor_PivotRight(uint32_t duty);

#endif /* __RTE_MOTOR_H */
