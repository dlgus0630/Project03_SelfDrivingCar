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
 * 내부 헬퍼: 좌/우 바퀴 개별 duty 차등 구동 (전진/후진 공통 경로)
 *
 * 방향 핀을 뒤집기 전에 duty를 먼저 0으로 내린다. 이전 명령의 높은 duty가
 * PWM 레지스터에 남은 채 방향 핀만 반대로 바뀌면(한 제어주기 50ms 안에서
 * 급전진->급후진 전환 시) 옛 속도 그대로 역방향으로 튀는 현상이 발생하기 때문.
 *
 * 전진/후진을 모두 이 하나의 헬퍼로 통과시키는 이유:
 *   - 글리치는 대칭이다. 후진->전진 전환도 전진->후진 전환과 똑같이 난폭하므로
 *     한쪽 방향에만 가드를 넣으면 반대쪽 스윙에서 그대로 터진다.
 *   - 진입점을 하나로 묶어두면 나중에 세 번째 차등 구동 변형을 추가하더라도
 *     가드를 빠뜨린 채로 만드는 것이 구조적으로 불가능해진다.
 * ------------------------------------------------------------------------ */
static void L298n_DriveDifferential(uint8_t reverse, uint16_t leftDuty, uint16_t rightDuty)
{
    L298n_SetLeftDuty(0);
    L298n_SetRightDuty(0);

    if (reverse != 0u)
    {
        L298n_SetLeftBackward();
        L298n_SetRightBackward();
    }
    else
    {
        L298n_SetLeftForward();
        L298n_SetRightForward();
    }

    L298n_SetLeftDuty(leftDuty);
    L298n_SetRightDuty(rightDuty);
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

/* 좌/우 바퀴 개별 duty로 "전진" (통로 중앙 유지 센터링 / 기울기 조향에 사용). */
void ECU_L298N_DriveForwardDifferential(uint16_t leftDuty, uint16_t rightDuty)
{
    L298n_DriveDifferential(0u, leftDuty, rightDuty);
}

/* 좌/우 바퀴 개별 duty로 "후진" (전진 차등 조향의 후진 버전, 기울기 조향에 사용). */
void ECU_L298N_DriveBackwardDifferential(uint16_t leftDuty, uint16_t rightDuty)
{
    L298n_DriveDifferential(1u, leftDuty, rightDuty);
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
