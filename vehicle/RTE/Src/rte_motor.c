/*
 * rte_motor.c
 *  Layer  : RTE (Runtime Environment)
 *  Module : RTE_MOTOR
 *  Desc   : ECU_L298N 모터 드라이버로의 1:1 pass-through 구현.
 */
#include "rte_motor.h"

void RTE_Motor_Init(void)
{
    ECU_L298N_Init();
}

void RTE_Motor_SetSpeed(uint32_t left_duty, uint32_t right_duty)
{
    ECU_L298N_SetSpeed(left_duty, right_duty);
}

void RTE_Motor_DriveForward(uint32_t duty)
{
    ECU_L298N_DriveForward(duty);
}

void RTE_Motor_DriveBackward(uint32_t duty)
{
    ECU_L298N_DriveBackward(duty);
}

void RTE_Motor_TurnLeft(uint32_t duty)
{
    ECU_L298N_TurnLeft(duty);
}

void RTE_Motor_TurnRight(uint32_t duty)
{
    ECU_L298N_TurnRight(duty);
}

void RTE_Motor_Stop(void)
{
    ECU_L298N_Stop();
}

void RTE_Motor_Brake(void)
{
    ECU_L298N_Brake();
}

void RTE_Motor_DriveForwardDifferential(uint16_t leftDuty, uint16_t rightDuty)
{
    ECU_L298N_DriveForwardDifferential(leftDuty, rightDuty);
}

void RTE_Motor_DriveBackwardDifferential(uint16_t leftDuty, uint16_t rightDuty)
{
    ECU_L298N_DriveBackwardDifferential(leftDuty, rightDuty);
}

void RTE_Motor_PivotLeft(uint32_t duty)
{
    ECU_L298N_PivotLeft(duty);
}

void RTE_Motor_PivotRight(uint32_t duty)
{
    ECU_L298N_PivotRight(duty);
}
