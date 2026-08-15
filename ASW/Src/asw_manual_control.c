/*
 * asw_manual_control.c
 *  Layer  : ASW
 *  Module : ASW_MANUAL_CONTROL
 *
 *  Node1(NUCLEO)이 블루투스로 수신한 조향/속도 명령을 CAN(0x110)으로
 *  전달받거나, 휴대폰 BT 앱의 단일문자 명령을 시리얼로 직접 받아 모터를 구동.
 *
 *  안전 정지(Fail-Safe) 기준은 명령 출처에 따라 다르다.
 *   - CAN    : 마스터가 명령을 주기적으로 재전송하므로 1000ms 미수신 시 정지.
 *   - SERIAL : BT 앱이 버튼을 누르는 동안 방향 문자를 반복 전송하지 않으므로
 *              같은 기준을 쓰면 정상 조작 중에도 멈춘다. 그래서 5000ms의
 *              더 긴 무응답 기준을 두어, 연결 끊김/앱 다운 시에만 정지시킨다.
 */
#include "asw_manual_control.h"
#include "rte_mode_manager.h"
#include "rte_motor.h"
#include <stdio.h>

typedef enum
{
    ASW_MANUAL_CMD_SOURCE_NONE = 0,
    ASW_MANUAL_CMD_SOURCE_CAN,
    ASW_MANUAL_CMD_SOURCE_SERIAL
} AswManualCmdSource_t;

static volatile uint8_t  s_lastCmd    = 'S';
static volatile uint32_t s_lastRxTick = 0;
static volatile AswManualCmdSource_t s_cmdSource = ASW_MANUAL_CMD_SOURCE_NONE;

void ASW_Manual_Init(void)
{
    s_lastCmd    = 'S';
    s_lastRxTick = osKernelSysTick();
    s_cmdSource  = ASW_MANUAL_CMD_SOURCE_NONE;
}

/*
 * ASW_Manual_ApplyCanCommand : CanRxTask가 수신 즉시 호출 (라우팅 지점)
 *   - MODE_CMD(0x100)  : 상태 머신 전환 (수동<->자동), 전환 시 안전 정지 선행
 *   - MANUAL_CMD(0x110): 최신 조향 명령 갱신 + 타임스탬프 기록
 */
void ASW_Manual_ApplyCanCommand(const McalCanMsg_t *p_msg)
{
    switch (p_msg->id)
    {
        case MCAL_CAN_ID_MASTER_MODE_CMD:
            RTE_Motor_Stop(); /* 모드 전환 시 항상 정지 먼저 (요구사항 2번) */
            if (p_msg->data[0] == 1u)
            {
                RTE_Mode_SetDriveMode(RTE_DRIVE_MODE_AUTO);
                printf("[MODE] MANUAL -> AUTO (CAN 0x100 rx)\r\n");
            }
            else
            {
                RTE_Mode_SetDriveMode(RTE_DRIVE_MODE_MANUAL);
                s_lastCmd   = 'S'; /* 수동 재진입 시 정지 상태로 시작 */
                s_cmdSource = ASW_MANUAL_CMD_SOURCE_NONE;
                printf("[MODE] AUTO -> MANUAL (CAN 0x100 rx)\r\n");
            }
            break;

        case MCAL_CAN_ID_MASTER_MANUAL_CMD:
            s_lastCmd    = p_msg->data[0]; /* 'F'/'B'/'L'/'R'/'S' */
            s_lastRxTick = osKernelSysTick();
            s_cmdSource  = ASW_MANUAL_CMD_SOURCE_CAN;
            break;

        default:
            break; /* 관심 없는 ID는 무시 */
    }
}

/*
 * ASW_Manual_ApplySerialChar : CtrlTask가 매 주기(50ms) 폴링해서 호출.
 *   ch==0이면 새 명령 없음(아무 것도 안 함).
 *   'A'(START)/'P'(PAUSE)는 현재 모드와 무관하게 항상 처리해서, 자율주행
 *   중에도 즉시 수동 전환할 수 있게 한다 (CAN의 MODE_CMD와 동일 철학).
 */
void ASW_Manual_ApplySerialChar(uint8_t ch)
{
    if (ch == 0u) { return; }

    if (ch == 'A')
    {
        RTE_Motor_Stop(); /* 모드 전환 시 항상 정지 먼저 */
        RTE_Mode_SetDriveMode(RTE_DRIVE_MODE_AUTO);
        printf("[MODE] -> AUTO (BT 'A')\r\n");
    }
    else if (ch == 'P')
    {
        RTE_Motor_Stop();
        RTE_Mode_SetDriveMode(RTE_DRIVE_MODE_MANUAL);
        s_lastCmd   = 'S'; /* 수동 재진입 시 정지 상태로 시작 */
        s_cmdSource = ASW_MANUAL_CMD_SOURCE_NONE;
        printf("[MODE] -> MANUAL (BT 'P')\r\n");
    }
    else
    {
        s_lastCmd    = ch;
        s_lastRxTick = osKernelSysTick();
        s_cmdSource  = ASW_MANUAL_CMD_SOURCE_SERIAL;
    }
}

/*
 * ASW_Manual_IsCommandTimeout : 명령 출처별로 서로 다른 무응답 기준을 적용한다.
 *   - CAN    : 마스터가 명령을 주기적으로 재전송하므로 1000ms로 짧게 검사한다.
 *   - SERIAL : 휴대폰 BT 앱은 버튼을 누르는 동안 방향 문자를 반복 전송하지 않고
 *              누를 때 1번, 뗄 때 'S' 1번만 보낸다. 그래서 CAN과 같은 짧은
 *              기준을 쓰면 계속 누르고 있어도 차가 멈춰버린다. 대신 5000ms의
 *              훨씬 긴 기준을 두어 "아무 명령도 안 들어온 지 5초"일 때만 정지한다.
 *              5초 넘게 계속 누르고 있으면 멈추므로 완벽한 해결책은 아니고,
 *              BT 연결이 끊기거나 앱이 죽었을 때 차가 영원히 달리는 최악의
 *              상황만 막기 위한 절충안이다.
 *   - NONE   : 아직 명령을 받은 적이 없으므로 검사하지 않는다(0 반환).
 */
uint8_t ASW_Manual_IsCommandTimeout(void)
{
    uint32_t limit_ms;

    if (s_cmdSource == ASW_MANUAL_CMD_SOURCE_CAN)
    {
        limit_ms = ASW_MANUAL_CMD_TIMEOUT_MS;
    }
    else if (s_cmdSource == ASW_MANUAL_CMD_SOURCE_SERIAL)
    {
        limit_ms = ASW_MANUAL_SERIAL_SILENCE_MS;
    }
    else
    {
        return 0u; /* 출처 없음(NONE) : 타임아웃 검사 대상 아님 */
    }

    return ((osKernelSysTick() - s_lastRxTick) > limit_ms) ? 1u : 0u;
}

/*
 * ASW_Manual_ProcessControl : CtrlTask 주기 호출 (RTE_DRIVE_MODE_MANUAL일 때만)
 */
void ASW_Manual_ProcessControl(void)
{
    if (ASW_Manual_IsCommandTimeout())
    {
        RTE_Motor_Stop(); /* Fail-Safe: 마스터 연결 끊김 */
        return;
    }

    switch (s_lastCmd)
    {
        case 'F': RTE_Motor_DriveForward(ASW_MANUAL_SPEED_DUTY);  break;
        case 'B': RTE_Motor_DriveBackward(ASW_MANUAL_SPEED_DUTY); break;
        case 'L': RTE_Motor_TurnLeft(ASW_MANUAL_SPEED_DUTY);      break;
        case 'R': RTE_Motor_TurnRight(ASW_MANUAL_SPEED_DUTY);     break;
        case 'S':
        default:  RTE_Motor_Stop();                          break;
    }
}
