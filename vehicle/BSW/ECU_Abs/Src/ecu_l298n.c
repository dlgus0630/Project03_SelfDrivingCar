/*
 * ecu_l298n.c
 *  Layer  : BSW / ECU_Abs
 *  Module : ECU_L298N
 */
#include "ecu_l298n.h"

/* ------------------------------------------------------------------------
 * 내부 헬퍼: GPIO 방향 설정 (s_ 없음 - 파일 스코프 static 함수)
 * ------------------------------------------------------------------------ */
static void L298n_SetLeftForward(void)
{
    HAL_GPIO_WritePin(ECU_L298N_IN1_PORT, ECU_L298N_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ECU_L298N_IN2_PORT, ECU_L298N_IN2_PIN, GPIO_PIN_RESET);
}

static void L298n_SetLeftBackward(void)
{
    HAL_GPIO_WritePin(ECU_L298N_IN1_PORT, ECU_L298N_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ECU_L298N_IN2_PORT, ECU_L298N_IN2_PIN, GPIO_PIN_SET);
}

static void L298n_SetLeftStop(void)
{
    HAL_GPIO_WritePin(ECU_L298N_IN1_PORT, ECU_L298N_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ECU_L298N_IN2_PORT, ECU_L298N_IN2_PIN, GPIO_PIN_RESET);
}

static void L298n_SetRightForward(void)
{
    HAL_GPIO_WritePin(ECU_L298N_IN3_PORT, ECU_L298N_IN3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ECU_L298N_IN4_PORT, ECU_L298N_IN4_PIN, GPIO_PIN_RESET);
}

static void L298n_SetRightBackward(void)
{
    HAL_GPIO_WritePin(ECU_L298N_IN3_PORT, ECU_L298N_IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ECU_L298N_IN4_PORT, ECU_L298N_IN4_PIN, GPIO_PIN_SET);
}

static void L298n_SetRightStop(void)
{
    HAL_GPIO_WritePin(ECU_L298N_IN3_PORT, ECU_L298N_IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ECU_L298N_IN4_PORT, ECU_L298N_IN4_PIN, GPIO_PIN_RESET);
}

/* ------------------------------------------------------------------------
 * 내부 헬퍼: PWM Duty 설정 (0 ~ ECU_L298N_MAX_DUTY)
 * ------------------------------------------------------------------------ */
static void L298n_SetLeftDuty(uint32_t duty)
{
    if (duty > ECU_L298N_MAX_DUTY) { duty = ECU_L298N_MAX_DUTY; }
    __HAL_TIM_SET_COMPARE(&ECU_L298N_TIM_HANDLE, ECU_L298N_CH_LEFT, duty);
}

static void L298n_SetRightDuty(uint32_t duty)
{
    if (duty > ECU_L298N_MAX_DUTY) { duty = ECU_L298N_MAX_DUTY; }
    __HAL_TIM_SET_COMPARE(&ECU_L298N_TIM_HANDLE, ECU_L298N_CH_RIGHT, duty);
}

/* ------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------ */
void ECU_L298N_Init(void)
{
    HAL_TIM_PWM_Start(&ECU_L298N_TIM_HANDLE, ECU_L298N_CH_LEFT);
    HAL_TIM_PWM_Start(&ECU_L298N_TIM_HANDLE, ECU_L298N_CH_RIGHT);
    ECU_L298N_Stop();
}

void ECU_L298N_DriveForward(uint32_t duty)
{
    L298n_SetLeftForward();
    L298n_SetRightForward();
    L298n_SetLeftDuty(duty);
    L298n_SetRightDuty(duty);
}

void ECU_L298N_DriveBackward(uint32_t duty)
{
    L298n_SetLeftBackward();
    L298n_SetRightBackward();
    L298n_SetLeftDuty(duty);
    L298n_SetRightDuty(duty);
}

/* 좌회전: 우측 모터만 전진 (제자리 피벗) */
void ECU_L298N_TurnLeft(uint32_t duty)
{
    L298n_SetLeftStop();
    L298n_SetRightForward();
    L298n_SetLeftDuty(0);
    L298n_SetRightDuty(duty);
}

/* 우회전: 좌측 모터만 전진 (제자리 피벗) */
void ECU_L298N_TurnRight(uint32_t duty)
{
    L298n_SetLeftForward();
    L298n_SetRightStop();
    L298n_SetLeftDuty(duty);
    L298n_SetRightDuty(0);
}

void ECU_L298N_Stop(void)
{
    L298n_SetLeftStop();
    L298n_SetRightStop();
    L298n_SetLeftDuty(0);
    L298n_SetRightDuty(0);
}

/* 급제동: 두 채널 모두 IN1=IN2(같은 레벨: HIGH)로 만들고 duty를 최대로 걸어
 * L298N의 "Fast Motor Stop" 동작(모터 단자 내부 단락)을 이끌어낸다.
 * Stop()과 달리 관성으로 미끄러지지 않고 즉시 감속된다. */
void ECU_L298N_Brake(void)
{
    HAL_GPIO_WritePin(ECU_L298N_IN1_PORT, ECU_L298N_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ECU_L298N_IN2_PORT, ECU_L298N_IN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ECU_L298N_IN3_PORT, ECU_L298N_IN3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ECU_L298N_IN4_PORT, ECU_L298N_IN4_PIN, GPIO_PIN_SET);
    L298n_SetLeftDuty(ECU_L298N_MAX_DUTY);
    L298n_SetRightDuty(ECU_L298N_MAX_DUTY);
}

void ECU_L298N_SetSpeed(uint32_t left_duty, uint32_t right_duty)
{
    if (left_duty  > ECU_L298N_MAX_DUTY) { left_duty  = ECU_L298N_MAX_DUTY; }
    if (right_duty > ECU_L298N_MAX_DUTY) { right_duty = ECU_L298N_MAX_DUTY; }
    L298n_SetLeftDuty(left_duty);
    L298n_SetRightDuty(right_duty);
}

/* 좌/우 바퀴 개별 duty로 "전진" (양쪽 모두 전진 방향, 속도차만 상이).
 * 통로 중앙을 유지하기 위한 비례 조향(센터링)에 사용. */
void ECU_L298N_DriveForwardDifferential(uint16_t leftDuty, uint16_t rightDuty)
{
    L298n_SetLeftForward();
    L298n_SetRightForward();
    L298n_SetLeftDuty(leftDuty);
    L298n_SetRightDuty(rightDuty);
}

/* 좌측 제자리 피벗: 좌측 바퀴 후진 + 우측 바퀴 전진 -> 차체 중심이 거의
 * 이동하지 않고 반시계 방향으로 회전 (편측 정지 회전과 달리 전방으로 쓸리지 않음). */
void ECU_L298N_PivotLeft(uint32_t duty)
{
    L298n_SetLeftBackward();
    L298n_SetRightForward();
    L298n_SetLeftDuty(duty);
    L298n_SetRightDuty(duty);
}

/* 우측 제자리 피벗: 좌측 바퀴 전진 + 우측 바퀴 후진 -> 시계 방향 제자리 회전. */
void ECU_L298N_PivotRight(uint32_t duty)
{
    L298n_SetLeftForward();
    L298n_SetRightBackward();
    L298n_SetLeftDuty(duty);
    L298n_SetRightDuty(duty);
}
