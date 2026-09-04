/*
 * asw_manual_control.c
 *  Layer  : ASW
 *  Module : ASW_MANUAL_CONTROL
 *
 *  Node1(NUCLEO)이 블루투스로 수신한 조향/속도 명령을 CAN(0x110)으로
 *  전달받거나, 휴대폰 BT 앱의 단일문자 명령을 시리얼로 직접 받아 모터를 구동.
 *  여기에 더해 remote 노드가 CAN(0x120)으로 보내는 IMU 기울기(roll/pitch)를
 *  좌/우 차등 PWM으로 변환해 주행하는 세 번째 명령 출처도 지원한다.
 *
 *  안전 정지(Fail-Safe) 기준은 명령 출처에 따라 다르다.
 *   - CAN    : 마스터가 명령을 주기적으로 재전송하므로 1000ms 미수신 시 정지.
 *   - IMU    : 같은 remote 마스터가 주기 전송하는 CAN 출처라 CAN과 동일 기준.
 *   - SERIAL : BT 앱이 버튼을 누르는 동안 방향 문자를 반복 전송하지 않으므로
 *              같은 기준을 쓰면 정상 조작 중에도 멈춘다. 그래서 5000ms의
 *              더 긴 무응답 기준을 두어, 연결 끊김/앱 다운 시에만 정지시킨다.
 */
#include "asw_manual_control.h"
#include "rte_mode_manager.h"
#include "rte_motor.h"
#include <stdio.h>

/* ------------------------------------------------------------------------
 * IMU 기울기 -> 차등 PWM 변환 튜닝 상수
 *   헤더(asw_manual_control.h)는 ecu_l298n.h를 포함하지 않아 ECU_L298N_MAX_DUTY를
 *   볼 수 없다. 그래서 이 값들은 헤더가 아니라 구현 파일(.c)에만 둔다.
 *   (이 파일은 rte_motor.h -> ecu_l298n.h 경유로 최대 duty를 이미 알고 있다.)
 * ------------------------------------------------------------------------ */
#define ASW_IMU_DEADZONE_DEG   5.0f   /* 이 각도 미만은 손떨림으로 보고 무시 */
#define ASW_IMU_MAX_TILT_DEG   30.0f  /* 이 각도에서 최대 duty에 도달 */
#define ASW_IMU_MIN_DUTY       300u   /* 데드존을 막 벗어난 순간의 기동 duty */
#define ASW_IMU_MAX_DUTY       ECU_L298N_MAX_DUTY   /* 하드코딩 금지: L298N 헤더의 최대 duty와 항상 동기화 */

typedef enum
{
    ASW_MANUAL_CMD_SOURCE_NONE = 0,
    ASW_MANUAL_CMD_SOURCE_CAN,
    ASW_MANUAL_CMD_SOURCE_SERIAL,
    ASW_MANUAL_CMD_SOURCE_IMU
} AswManualCmdSource_t;

static volatile uint8_t  s_lastCmd    = 'S';
static volatile uint32_t s_lastRxTick = 0;
static volatile AswManualCmdSource_t s_cmdSource = ASW_MANUAL_CMD_SOURCE_NONE;

/* IMU 출처일 때 적용할 좌/우 duty와 진행 방향.
 * 계산된 duty는 "크기"만 담고 부호가 없으므로, 실제로 모터에 적용하는 쪽에서
 * 전진인지 후진인지 알 수 없다. 그래서 방향을 s_imuReverse로 따로 들고 간다. */
static volatile uint16_t s_imuLeftDuty  = 0;
static volatile uint16_t s_imuRightDuty = 0;
static volatile uint8_t  s_imuReverse   = 0; /* 1 = 후진 방향 */

void ASW_Manual_Init(void)
{
    s_lastCmd    = 'S';
    s_lastRxTick = osKernelSysTick();
    s_cmdSource  = ASW_MANUAL_CMD_SOURCE_NONE;

    s_imuLeftDuty  = 0;
    s_imuRightDuty = 0;
    s_imuReverse   = 0;
}

/*
 * ClampF / AbsF : 이 프로젝트는 math.h(libm)를 어디에서도 쓰지 않는다.
 *   fabsf/fminf/fmaxf를 쓰면 libm 의존성이 새로 생기므로, 필요한 만큼만
 *   직접 만들어 쓴다.
 */
static float ClampF(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static float AbsF(float v)
{
    return (v < 0.0f) ? -v : v;
}

/*
 * ComputeTiltDuty : IMU 기울기(도) -> 좌/우 duty 크기 + 진행 방향으로 변환한다.
 *   - pitch : 앞뒤 기울기. 크기가 속도, 부호가 진행 방향을 결정한다.
 *             (양수 = 앞으로 기울임 = 전진)
 *   - roll  : 좌우 기울기. 좌/우 duty에 서로 반대 부호로 더해 조향 편차를 만든다.
 *   두 축 모두 데드존 안이면 duty를 0으로 내려 보내고, 호출한 쪽이 정지시킨다.
 */
static void ComputeTiltDuty(float rollDeg, float pitchDeg,
                            uint16_t *outLeft, uint16_t *outRight,
                            uint8_t *outReverse)
{
    float clampedPitch;
    float clampedRoll;
    float baseDuty;
    float turnBias;

    if ((AbsF(pitchDeg) < ASW_IMU_DEADZONE_DEG) &&
        (AbsF(rollDeg)  < ASW_IMU_DEADZONE_DEG))
    {
        /* 데드존: 리모컨을 평평하게 들고 있는 상태. 정지 지시로 취급한다. */
        *outLeft    = 0u;
        *outRight   = 0u;
        *outReverse = 0u;
        return;
    }

    *outReverse = (pitchDeg < 0.0f) ? 1u : 0u; /* 뒤로 기울이면 후진 */

    /* 기본 속도 : 기울기 크기를 MIN_DUTY~MAX_DUTY 구간에 선형으로 대응시킨다.
     * 데드존을 막 벗어났을 때 0이 아니라 MIN_DUTY에서 시작해야 모터가 실제로 돈다. */
    clampedPitch = ClampF(pitchDeg, -ASW_IMU_MAX_TILT_DEG, ASW_IMU_MAX_TILT_DEG);
    baseDuty = (float)ASW_IMU_MIN_DUTY +
               (AbsF(clampedPitch) / ASW_IMU_MAX_TILT_DEG) *
               (float)(ASW_IMU_MAX_DUTY - ASW_IMU_MIN_DUTY);

    /* 조향 편차 : 좌우 기울기를 좌/우 duty에 +/-로 나눠 준다. */
    clampedRoll = ClampF(rollDeg, -ASW_IMU_MAX_TILT_DEG, ASW_IMU_MAX_TILT_DEG);
    turnBias = (clampedRoll / ASW_IMU_MAX_TILT_DEG) *
               (float)(ASW_IMU_MAX_DUTY - ASW_IMU_MIN_DUTY) * 0.5f;
               /* 절반 범위: 조향 편차만 주고 혼자 포화되지 않게 */

    *outLeft  = (uint16_t)ClampF(baseDuty + turnBias, 0.0f, (float)ASW_IMU_MAX_DUTY);
    *outRight = (uint16_t)ClampF(baseDuty - turnBias, 0.0f, (float)ASW_IMU_MAX_DUTY);
}

/*
 * ASW_Manual_ApplyImuAttitude : 0.1도 단위 정수 각도를 받아 IMU 명령 상태를 갱신한다.
 *   이 파일의 ASW_Manual_ApplyCanCommand()에서만 호출하므로 static으로 둔다.
 *   부동소수점 연산은 임계구역 밖에서 끝내고, 공유 변수 5개에 대한 "대입만"
 *   하나의 임계구역으로 묶는다. 다섯 값이 한 덩어리로 보여야 CtrlTask가
 *   섞인 조합(예: 새 duty + 옛 방향)을 관측하지 않는다.
 */
static void ASW_Manual_ApplyImuAttitude(int16_t rollTenths, int16_t pitchTenths)
{
    float    rollDeg  = (float)rollTenths  / 10.0f;
    float    pitchDeg = (float)pitchTenths / 10.0f;
    uint16_t leftDuty  = 0u;
    uint16_t rightDuty = 0u;
    uint8_t  reverse   = 0u;

    ComputeTiltDuty(rollDeg, pitchDeg, &leftDuty, &rightDuty, &reverse);

    taskENTER_CRITICAL();
    s_imuLeftDuty  = leftDuty;
    s_imuRightDuty = rightDuty;
    s_imuReverse   = reverse;
    s_lastRxTick   = osKernelSysTick();
    s_cmdSource    = ASW_MANUAL_CMD_SOURCE_IMU;
    taskEXIT_CRITICAL();
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
                /* 상태 3종(s_lastCmd/s_lastRxTick/s_cmdSource)은 CtrlTask와
                 * CanRxTask 양쪽에서 갱신되므로, 섞인 조합이 관측되지 않도록
                 * 대입만 임계구역으로 묶는다. */
                taskENTER_CRITICAL();
                s_lastCmd   = 'S'; /* 수동 재진입 시 정지 상태로 시작 */
                s_cmdSource = ASW_MANUAL_CMD_SOURCE_NONE;
                /* 남아 있던 IMU duty가 모드 전환 후에 되살아나지 않도록 같이 지운다. */
                s_imuLeftDuty  = 0;
                s_imuRightDuty = 0;
                s_imuReverse   = 0;
                taskEXIT_CRITICAL();
                printf("[MODE] AUTO -> MANUAL (CAN 0x100 rx)\r\n");
            }
            break;

        case MCAL_CAN_ID_MASTER_MANUAL_CMD:
            /* 3개 값이 한 덩어리로 보이도록 대입만 임계구역으로 묶는다.
             * (섞이면 CAN 명령에 SERIAL 출처가 붙어 5000ms 기준이 적용된다.) */
            taskENTER_CRITICAL();
            s_lastCmd    = p_msg->data[0]; /* 'F'/'B'/'L'/'R'/'S' */
            s_lastRxTick = osKernelSysTick();
            s_cmdSource  = ASW_MANUAL_CMD_SOURCE_CAN;
            taskEXIT_CRITICAL();
            break;

        /* remote 노드 IMU 자세(0x120) : 8바이트, 모두 MSB first(빅엔디안).
         *   data[0..1] : roll  (0.1도 단위, int16)
         *   data[2..3] : pitch (0.1도 단위, int16)
         *   data[4..5] : yaw   (이번 단계에서는 항상 0 -> 사용하지 않음)
         *   data[6]    : 상태 플래그. bit0 = 캘리브레이션 완료.
         *                제어를 이 비트로 막지는 않는다. 프레임이 왔다는 것
         *                자체를 유효한 명령으로 취급한다(사용하지 않음).
         *   data[7]    : 롤링 카운터(수신 확인용) -> 사용하지 않음 */
        case MCAL_CAN_ID_REMOTE_IMU_ATTITUDE:
        {
            int16_t rollTenths  = (int16_t)(((uint16_t)p_msg->data[0] << 8) | p_msg->data[1]);
            int16_t pitchTenths = (int16_t)(((uint16_t)p_msg->data[2] << 8) | p_msg->data[3]);
            ASW_Manual_ApplyImuAttitude(rollTenths, pitchTenths);
            break;
        }

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
 * RefreshSerialRxTickIfActive : 이미 SERIAL 출처로 감시 중일 때만 타임스탬프 갱신.
 *   s_cmdSource를 읽고 s_lastRxTick을 쓰므로, CanRxTask와의 경합을 막기 위해
 *   두 문장을 임계구역으로 묶는다.
 */
static void RefreshSerialRxTickIfActive(void)
{
    taskENTER_CRITICAL();
    if (s_cmdSource == ASW_MANUAL_CMD_SOURCE_SERIAL)
    {
        s_lastRxTick = osKernelSysTick();
    }
    taskEXIT_CRITICAL();
}

/*
 * ASW_Manual_ApplySerialChar : CtrlTask가 매 주기(50ms) 폴링해서 호출.
 *   ch==0이면 새 명령 없음(아무 것도 안 함).
 *   'A'(START)/'P'(PAUSE)는 현재 모드와 무관하게 항상 처리해서, 자율주행
 *   중에도 즉시 수동 전환할 수 있게 한다 (CAN의 MODE_CMD와 동일 철학).
 *
 *  [핵심 설계 원칙] "링크 생존 확인"과 "주행 명령 변경"을 분리한다.
 *   - 링크가 살아있다는 증거      = "아는 명령 문자"가 들어왔을 때만 인정한다.
 *                                 -> 그때만 무응답 타임스탬프를 갱신한다.
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
        RefreshSerialRxTickIfActive();

        /* 여기 있던 블로킹 로그(printf)는 제거했다. 블루투스와 같은 USART1
         * (9600bps)으로 나가면서 한 줄에 30ms 넘게 블로킹되어 50ms 제어주기를
         * 잠식했기 때문이다. */
    }
    else if (ch == 'P')
    {
        RTE_Motor_Stop();
        RTE_Mode_SetDriveMode(RTE_DRIVE_MODE_MANUAL);
        taskENTER_CRITICAL();
        s_lastCmd   = 'S'; /* 수동 재진입 시 정지 상태로 시작 */
        s_cmdSource = ASW_MANUAL_CMD_SOURCE_NONE;
        /* 남아 있던 IMU duty가 모드 전환 후에 되살아나지 않도록 같이 지운다. */
        s_imuLeftDuty  = 0;
        s_imuRightDuty = 0;
        s_imuReverse   = 0;
        taskEXIT_CRITICAL();

        /* 출처를 NONE으로 되돌리므로 ASW_Manual_IsCommandTimeout()이 곧바로 0을
         * 반환한다. 즉 타임아웃 감시 자체가 꺼진 상태라 타임스탬프는 아무 의미가
         * 없다. 그래서 여기서는 일부러 s_lastRxTick을 갱신하지 않는다.
         * (다음 방향 명령이 들어오는 순간 그 시점 기준으로 새로 갱신된다.) */

        /* 여기 있던 블로킹 로그(printf)도 위 'A'와 같은 이유로 제거했다. */
    }
    else if (ASW_Manual_IsDriveCmdChar(ch) != 0u)
    {
        /* 아는 주행 명령 문자일 때만 실제 주행 명령을 바꾼다.
         * 이 경로는 출처를 무조건 SERIAL로 설정해야 하므로 헬퍼를 쓰지 않고,
         * 3개 대입을 통째로 임계구역에 넣어 CanRxTask와 섞이지 않게 한다. */
        taskENTER_CRITICAL();
        s_lastCmd    = ch;
        s_lastRxTick = osKernelSysTick();
        s_cmdSource  = ASW_MANUAL_CMD_SOURCE_SERIAL;
        taskEXIT_CRITICAL();
    }
    else
    {
        /* 모르는 문자(앱이 보내는 속도 단계 '0'~'9', 전조등 'W', 경적 'V' 등).
         * 주행 명령은 절대 건드리지 않고 무시한다. s_lastCmd를 건드리지 않으므로
         * 진행 중이던 주행이 모르는 문자 하나 때문에 끊기지 않는다.
         *
         * [변경] 예전에는 여기서도 타임스탬프를 갱신했지만 이제 하지 않는다.
         *  모르는 바이트는 더 이상 "링크가 살아있다는 증거"로 인정하지 않는다.
         *  앱이 멈췄거나 RF 잡음으로 생긴 쓰레기 바이트가 계속 들어오면,
         *  실제 명령은 하나도 오지 않는데도 5000ms 무응답 정지가 무한정
         *  미뤄져 차가 옛 명령으로 계속 달릴 수 있기 때문이다.
         *  따라서 이 분기는 바이트를 그냥 버리는 것 외에 아무 일도 하지 않는다. */
    }
}

/*
 * ASW_Manual_IsCommandTimeout : 명령 출처별로 서로 다른 무응답 기준을 적용한다.
 *   - CAN    : 마스터가 명령을 주기적으로 재전송하므로 1000ms로 짧게 검사한다.
 *   - IMU    : CAN과 완전히 같은 기준(1000ms)을 쓴다. 아래 분기 참고.
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

    /* IMU도 같은 remote 마스터가 주기(100ms) 전송하는 CAN 출처이므로
     * 동일한 1000ms 생존 기준을 적용한다. */
    if ((s_cmdSource == ASW_MANUAL_CMD_SOURCE_CAN) ||
        (s_cmdSource == ASW_MANUAL_CMD_SOURCE_IMU))
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

    if (s_cmdSource == ASW_MANUAL_CMD_SOURCE_IMU)
    {
        uint16_t leftDuty;
        uint16_t rightDuty;
        uint8_t  reverse;

        /* CanRxTask가 세 값을 한 덩어리로 쓰므로, 읽을 때도 한 덩어리로 읽는다.
         * (중간에 갱신되면 새 duty에 옛 방향이 붙는 조합이 나올 수 있다.) */
        taskENTER_CRITICAL();
        leftDuty  = s_imuLeftDuty;
        rightDuty = s_imuRightDuty;
        reverse   = s_imuReverse;
        taskEXIT_CRITICAL();

        if ((leftDuty == 0u) && (rightDuty == 0u))
        {
            RTE_Motor_Stop(); /* 데드존(리모컨을 평평하게 든 상태) */
        }
        else if (reverse == 0u)
        {
            RTE_Motor_DriveForwardDifferential(leftDuty, rightDuty);
        }
        else
        {
            /* [후진 차등 조향 경로]
             * RTE_Motor_SetSpeed(= ECU_L298N_SetSpeed)는 PWM duty 레지스터만 쓰고
             * IN 방향 핀은 전혀 건드리지 않는다. 그런데 L298N에는 "후진 차등"
             * API가 없다(DriveForwardDifferential은 전진 전용).
             * 그래서 먼저 RTE_Motor_DriveBackward(0u)를 호출해 양쪽 IN 핀만
             * 후진 방향으로 세팅하고(duty를 0으로 주는 이유는, 잠깐이라도
             * 최대 속도로 튀는 글리치를 막기 위해서다), 바로 이어서
             * RTE_Motor_SetSpeed()로 좌/우 독립 duty를 얹는다.
             * 이렇게 하면 기존 공개 API만으로 후진에서도 차등 조향이 된다.
             *
             * [의존성 주의] 이 방식은 SetSpeed가 "duty 전용"이라는 사실에
             * 기대고 있다. 만약 SetSpeed가 나중에 방향 핀까지 건드리도록
             * 바뀌면 이 경로는 반드시 다시 검토해야 한다. */
            RTE_Motor_DriveBackward(0u);
            RTE_Motor_SetSpeed(leftDuty, rightDuty);
        }
        return; /* IMU 출처는 아래 문자 명령 switch를 타지 않는다. */
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
