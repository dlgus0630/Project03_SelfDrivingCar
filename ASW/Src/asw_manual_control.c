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
 * ASW_Manual_IsDriveCmdChar : 주행 명령으로 "인정"하는 문자인지 판별한다.
 *   여기 목록에 없는 문자는 전부 모르는 문자로 취급해서 주행 명령을 바꾸지 않는다.
 *   시중 RC카 블루투스 앱은 방향 문자 외에도 속도 단계('0'~'9'),
 *   전조등('W'/'w'), 경적('V'/'v') 같은 부가 문자를 함께 보내기 때문이다.
 */
static uint8_t ASW_Manual_IsDriveCmdChar(uint8_t ch)
{
    return (((ch == 'F') || (ch == 'B') || (ch == 'L') ||
             (ch == 'R') || (ch == 'S')) ? 1u : 0u);
}

/*
 * ASW_Manual_ApplySerialChar : CtrlTask가 매 주기(50ms) 폴링해서 호출.
 *   ch==0이면 새 명령 없음(아무 것도 안 함).
 *   'A'(START)/'P'(PAUSE)는 현재 모드와 무관하게 항상 처리해서, 자율주행
 *   중에도 즉시 수동 전환할 수 있게 한다 (CAN의 MODE_CMD와 동일 철학).
 *
 *  [핵심 설계 원칙] "링크 생존 확인"과 "주행 명령 변경"을 분리한다.
 *   - 바이트가 하나라도 들어왔다  = 블루투스 연결이 살아있다는 증거
 *                                 -> 무응답 타임스탬프를 갱신할 근거가 된다.
 *   - 주행 방향을 바꾼다          = 아는 명령 문자(F/B/L/R/S)일 때만 허용.
 *   - 모르는 문자                 = 완전히 무시. s_lastCmd를 절대 건드리지 않아
 *                                 진행 중이던 주행이 끊기지 않게 한다.
 *     (예전에는 모르는 문자가 s_lastCmd를 덮어써서 아래 switch의 default로
 *      떨어져 차가 즉시 멈추고 계속 멈춰 있는 문제가 있었다.)
 */
void ASW_Manual_ApplySerialChar(uint8_t ch)
{
    if (ch == 0u) { return; } /* 새로 들어온 문자가 없음 */

    if (ch == 'A')
    {
        RTE_Motor_Stop(); /* 모드 전환 시 항상 정지 먼저 */
        RTE_Mode_SetDriveMode(RTE_DRIVE_MODE_AUTO);

        /* 'A'도 실제로 수신된 바이트이므로 링크가 살아있다는 증거로 인정한다.
         * 다만 아직 방향 명령을 받은 적이 없는(NONE) 상태에서 타임아웃 감시를
         * 시작해버리면 안 되므로, 이미 SERIAL 출처로 감시 중일 때만 갱신한다.
         * (자동 모드에서는 ASW_Manual_ProcessControl이 호출되지 않아 당장은
         *  영향이 없지만, "타임스탬프는 SERIAL 출처일 때만 의미를 갖는다"는
         *  규칙을 코드 전체에서 똑같이 지키기 위해 이렇게 맞춰 둔다.) */
        if (s_cmdSource == ASW_MANUAL_CMD_SOURCE_SERIAL)
        {
            s_lastRxTick = osKernelSysTick();
        }

        /* [주의] 이 printf는 huart1으로 나가는데, huart1은 HC-06 블루투스와
         * 같은 USART1(9600bps)이다. 즉 이 로그가 그대로 휴대폰 앱으로 전송되며,
         * 한 줄 보내는 데 30ms 넘게 블로킹되어 50ms 제어주기를 잠식한다.
         * 모드 전환은 자주 일어나지 않아 당장 치명적이지는 않으나,
         * 주기적으로 찍히는 로그를 여기에 추가하면 제어가 밀리게 된다.
         * (제거 여부는 사용자 판단 사항이므로 일단 그대로 둔다.) */
        printf("[MODE] -> AUTO (BT 'A')\r\n");
    }
    else if (ch == 'P')
    {
        RTE_Motor_Stop();
        RTE_Mode_SetDriveMode(RTE_DRIVE_MODE_MANUAL);
        s_lastCmd   = 'S'; /* 수동 재진입 시 정지 상태로 시작 */
        s_cmdSource = ASW_MANUAL_CMD_SOURCE_NONE;

        /* 출처를 NONE으로 되돌리므로 ASW_Manual_IsCommandTimeout()이 곧바로 0을
         * 반환한다. 즉 타임아웃 감시 자체가 꺼진 상태라 타임스탬프는 아무 의미가
         * 없다. 그래서 여기서는 일부러 s_lastRxTick을 갱신하지 않는다.
         * (다음 방향 명령이 들어오는 순간 그 시점 기준으로 새로 갱신된다.) */

        /* [주의] 위 'A'와 동일 - printf가 블루투스와 같은 USART1으로 나간다. */
        printf("[MODE] -> MANUAL (BT 'P')\r\n");
    }
    else if (ASW_Manual_IsDriveCmdChar(ch) != 0u)
    {
        /* 아는 주행 명령 문자일 때만 실제 주행 명령을 바꾼다. */
        s_lastCmd    = ch;
        s_lastRxTick = osKernelSysTick();
        s_cmdSource  = ASW_MANUAL_CMD_SOURCE_SERIAL;
    }
    else
    {
        /* 모르는 문자(앱이 보내는 속도 단계 '0'~'9', 전조등 'W', 경적 'V' 등).
         * 주행 명령은 절대 건드리지 않고 무시한다. 다만 바이트가 도착했다는 것은
         * 블루투스 링크가 살아있다는 증거이므로 타임스탬프만 갱신해 준다.
         *
         * [중요] 반드시 이미 s_cmdSource == SERIAL인 경우에만 갱신한다.
         *  s_cmdSource를 여기서 SERIAL로 새로 설정해버리면, 아직 방향 명령을
         *  한 번도 받은 적 없는 부팅 직후(NONE) 상태에서 잡음 바이트 하나만으로
         *  타임아웃 감시가 시작되어 버린다. 그건 의도한 동작이 아니다. */
        if (s_cmdSource == ASW_MANUAL_CMD_SOURCE_SERIAL)
        {
            s_lastRxTick = osKernelSysTick();
        }
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
        /* [default의 의미 변경]
         * 이제 s_lastCmd에는 ASW_Manual_ApplySerialChar()에서 검증을 통과한
         * 명령 문자(F/B/L/R/S)만 저장되므로, default는 원래 도달하지 않는다.
         * 만약을 대비해 정지시키는 방어 코드로만 남겨 둔다.
         * (CAN 경로는 마스터가 보낸 값을 그대로 넣으므로, 마스터가 이상한 값을
         *  보냈을 때 안전하게 멈추는 최후의 보루 역할도 겸한다.) */
        default:  RTE_Motor_Stop();                          break;
    }
}
