/*
 * asw_manual_control.c
 *  Layer  : ASW
 *  Module : ASW_MANUAL_CONTROL
 *
 *  remote 노드가 블루투스로 수신한 조향/속도 명령을 CAN(0x110)으로
 *  전달받거나, 휴대폰 BT 앱의 단일문자 명령을 시리얼로 직접 받아 모터를 구동.
 *  여기에 더해 remote 노드가 CAN(0x120)으로 보내는 IMU 기울기(roll/pitch)를
 *  좌/우 차등 PWM으로 변환해 주행하는 세 번째 명령 출처도 지원한다.
 *
 *  안전 정지(Fail-Safe) 기준은 명령 출처에 따라 다르다.
 *   - CAN    : 마스터가 명령을 주기적으로 재전송하므로 1000ms 미수신 시 정지.
 *   - IMU    : 같은 remote 마스터가 주기 전송하는 CAN 출처라 CAN과 동일 기준.
 *              단, 리모컨을 다시 평평하게 놓았을 때의 정지는 이 타임아웃이 아니라
 *              "평평 = duty 0" 명령 자체가 곧바로 처리한다(다음 제어주기에 정지).
 *              타임아웃은 remote가 아예 사라진 경우만 담당한다.
 *   - SERIAL : BT 앱이 버튼을 누르는 동안 방향 문자를 반복 전송하지 않으므로
 *              같은 기준을 쓰면 정상 조작 중에도 멈춘다. 그래서 5000ms의
 *              더 긴 무응답 기준을 두어, 연결 끊김/앱 다운 시에만 정지시킨다.
 *
 *  [출처 경합 원칙] 세 출처는 "마지막에 명령을 준 쪽이 이긴다". 그래서 각
 *  수신 경로는 "링크가 살아있다"와 "실제 조작이 들어왔다"를 반드시 구분해야
 *  한다. remote의 IMU는 평평하게 놓여 있어도 100ms마다 프레임을 보내므로,
 *  프레임 도착만으로 출처를 가져가면 휴대폰 앱 조작을 영구히 빼앗는다.
 *  반대로 이미 IMU가 운전 중이라면 평평한 프레임은 "손을 뗐다 = 정지"라는
 *  분명한 조작이므로 그때는 반드시 반영해야 한다. 자세한 내용은 0x120 수신
 *  처리 주석 참조.
 */
#include "asw_manual_control.h"
#include "asw_imu_attitude.h"
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
/* 위 상수의 역수를 미리 계산해 둔다. 이 코어에는 FPU가 없어 나눗셈이
 * 소프트웨어 루틴(__aeabi_fdiv) 호출로 도는 비싼 연산인데, 나누는 값이
 * 컴파일 시점에 정해진 상수이므로 역수 곱셈으로 바꾸면 곱셈 한 번으로 끝난다.
 * (상수끼리의 나눗셈이라 역수 계산 자체는 컴파일러가 처리한다.) */
#define ASW_IMU_MAX_TILT_DEG_INV  (1.0f / ASW_IMU_MAX_TILT_DEG)
#define ASW_IMU_MIN_DUTY       300u   /* 데드존을 막 벗어난 순간의 기동 duty */
#define ASW_IMU_MAX_DUTY       ECU_L298N_MAX_DUTY   /* 하드코딩 금지: L298N 헤더의 최대 duty와 항상 동기화 */
#define ASW_IMU_DUTY_SPAN      ((float)(ASW_IMU_MAX_DUTY - ASW_IMU_MIN_DUTY)) /* 기울기 비율을 곱할 duty 가동 폭 */

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

/* IMU 출처일 때 마지막으로 "채택된" 기울기 주행 명령.
 * 각도(tenths)가 아니라 이미 duty로 변환을 끝낸 결과를 들고 간다. 변환을 수신
 * 시점(CanRxTask)으로 옮긴 이유는 ASW_Manual_ApplyCanCommand()의 0x120 처리
 * 주석 참조 - 데드존 판정 결과가 있어야 그 프레임을 명령으로 인정할지
 * 결정할 수 있기 때문이다. */
static volatile uint16_t s_imuLeftDuty  = 0u;
static volatile uint16_t s_imuRightDuty = 0u;
static volatile uint8_t  s_imuReverse   = 0u;

/*
 * EnterManualIdle : 수동 모드 재진입 시의 "정지 대기" 상태로 되돌린다.
 *   남아 있던 IMU 주행 명령(duty)까지 함께 지우는 이유는, 모드 전환 전의 낡은
 *   명령이 출처가 다시 IMU로 바뀌는 순간 되살아나 차가 갑자기 움직이는 것을
 *   막기 위해서다.
 *   여러 값을 함께 바꾸므로 CanRxTask가 섞인 조합을 관측하지 않도록
 *   대입 전체를 임계구역으로 묶는다.
 */
static void EnterManualIdle(void)
{
    taskENTER_CRITICAL();
    s_lastCmd       = 'S'; /* 수동 재진입 시 정지 상태로 시작 */
    s_cmdSource     = ASW_MANUAL_CMD_SOURCE_NONE;
    s_imuLeftDuty   = 0u;
    s_imuRightDuty  = 0u;
    s_imuReverse    = 0u;
    taskEXIT_CRITICAL();
}

void ASW_Manual_Init(void)
{
    EnterManualIdle();
    s_lastRxTick = osKernelSysTick();
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
    float absPitch = AbsF(pitchDeg); /* 데드존 판정과 기본 속도 계산에 모두 쓰므로 한 번만 구한다 */
    float clampedRoll;
    float baseDuty;
    float turnBias;

    if ((absPitch < ASW_IMU_DEADZONE_DEG) &&
        (AbsF(rollDeg) < ASW_IMU_DEADZONE_DEG))
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
    baseDuty = (float)ASW_IMU_MIN_DUTY +
               (ClampF(absPitch, 0.0f, ASW_IMU_MAX_TILT_DEG) * ASW_IMU_MAX_TILT_DEG_INV) *
               ASW_IMU_DUTY_SPAN;

    /* 조향 편차 : 좌우 기울기를 좌/우 duty에 +/-로 나눠 준다.
     * (pitch와 달리 roll은 좌/우 부호가 그대로 필요하므로 절댓값을 쓰지 않는다.) */
    clampedRoll = ClampF(rollDeg, -ASW_IMU_MAX_TILT_DEG, ASW_IMU_MAX_TILT_DEG);
    turnBias = (clampedRoll * ASW_IMU_MAX_TILT_DEG_INV) *
               ASW_IMU_DUTY_SPAN * 0.5f;
               /* 절반 범위: 조향 편차만 주고 혼자 포화되지 않게 */

    *outLeft  = (uint16_t)ClampF(baseDuty + turnBias, 0.0f, (float)ASW_IMU_MAX_DUTY);
    *outRight = (uint16_t)ClampF(baseDuty - turnBias, 0.0f, (float)ASW_IMU_MAX_DUTY);
}

/*
 * ASW_Manual_ApplyCanCommand : CanRxTask가 수신 즉시 호출 (라우팅 지점)
 *   - MODE_CMD(0x100)  : 상태 머신 전환 (수동<->자동), 전환 시 안전 정지 선행
 *   - MANUAL_CMD(0x110): 최신 조향 명령 갱신 + 타임스탬프 기록
 *   - IMU_ATTITUDE(0x120): 기울기를 duty로 변환해, 데드존을 벗어난 프레임이거나
 *                        이미 IMU가 활성 출처일 때만 명령으로 채택.
 *                        공유 자세 모듈 갱신은 항상 수행.
 */
void ASW_Manual_ApplyCanCommand(const McalCanMsg_t *p_msg)
{
    switch (p_msg->id)
    {
        case MCAL_CAN_ID_REMOTE_MODE_CMD:
            RTE_Motor_Stop(); /* 모드 전환 시 항상 정지 먼저 (요구사항 2번) */
            if (p_msg->data[0] == 1u)
            {
                RTE_Mode_SetDriveMode(RTE_DRIVE_MODE_AUTO);
                printf("[MODE] MANUAL -> AUTO (CAN 0x100 rx)\r\n");
            }
            else
            {
                RTE_Mode_SetDriveMode(RTE_DRIVE_MODE_MANUAL);
                EnterManualIdle();
                printf("[MODE] AUTO -> MANUAL (CAN 0x100 rx)\r\n");
            }
            break;

        case MCAL_CAN_ID_REMOTE_MANUAL_CMD:
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
         *   data[4..5] : yaw   (0.1도 단위, int16) -> 자율주행 PIVOT 과회전 가드가 사용.
         *   data[6]    : 상태 플래그. bit0 = 캘리브레이션 완료 플래그.
         *                이 비트로 수동 제어를 막지는 않는다. 대신 공유 자세 모듈의
         *                yaw 신뢰성 판정에 사용된다.
         *   data[7]    : 롤링 카운터(수신 확인용) -> 사용하지 않음 */
        case MCAL_CAN_ID_REMOTE_IMU_ATTITUDE:
        {
            uint16_t leftDuty  = 0u;
            uint16_t rightDuty = 0u;
            uint8_t  reverse   = 0u;
            uint8_t  dutyNonZero;      /* 아래 [게이트] 주석 참조 */
            uint8_t  imuAlreadyActive;

            int16_t rollTenths  = (int16_t)(((uint16_t)p_msg->data[0] << 8) | p_msg->data[1]);
            int16_t pitchTenths = (int16_t)(((uint16_t)p_msg->data[2] << 8) | p_msg->data[3]);

            /* yaw와 캘리브레이션 플래그는 수동 기울기 주행에는 쓰이지 않는다.
             * 공유 자세 모듈을 거쳐 자율주행 쪽이 읽어 가는 값이라 여기서 함께 푼다. */
            int16_t yawTenths = (int16_t)(((uint16_t)p_msg->data[4] << 8) | p_msg->data[5]);
            uint8_t remoteCalibrated = p_msg->data[6] & 0x01u;

            /* [핵심] 기울기 -> duty 변환을 여기(수신 시점)에서 먼저 끝낸다.
             *
             * 예전에는 "프레임이 왔다는 것 자체"를 명령으로 인정해 무조건
             * s_cmdSource를 IMU로 바꿨다. 그런데 remote는 부팅 직후부터 평평하게
             * 놓여 있어도 100ms마다 0x120을 계속 쏜다. 반면 휴대폰 BT 앱(SERIAL)은
             * 버튼을 누르고 뗄 때만 한 바이트를 보낸다. 그래서 "마지막에 쓴 쪽이
             * 이기는" 이 경합에서 10Hz로 떠드는 IMU가 거의 항상 이겨 버렸고,
             * remote를 옆에 켜 두기만 해도 휴대폰 조작이 먹히지 않았다.
             *
             * 해결 : "이 프레임이 의미 있는 조작인가"를 출처를 바꾸기 "전"에
             * 판정한다. 그 판정 기준은 이미 ComputeTiltDuty()가 갖고 있는
             * 데드존이므로, 새 기준을 만들지 않고 그 함수를 그대로 재사용해
             * 결과 duty가 0인지만 본다. (데드존 상수/로직을 여기에 복제하면
             * 두 곳이 따로 놀 위험이 생긴다.) 다만 그 판정만으로 프레임을
             * 버리면 반대편 버그가 생긴다 - 아래 게이트 주석 참조.
             *
             * [비용에 대한 정직한 설명]
             * 이 변환은 부동소수점이고 이 MCU에는 FPU가 없어 비싸다. 예전 주석은
             * 그래서 변환을 최우선순위 CanRxTask 밖(50ms CtrlTask)으로 미룬다고
             * 적혀 있었지만, 위 이유로 그 설계를 의도적으로 뒤집었다. 대신 손해만
             * 보는 거래는 아니다. 변환이 수신 프레임당 1회(10Hz)로 줄고 CtrlTask는
             * 이 경로에서 부동소수점 연산을 아예 하지 않게 되므로, 예전의
             * 20Hz(CtrlTask 50ms 주기)보다 초당 총 부동소수점 작업량은 오히려 준다. */
            ComputeTiltDuty(ASW_ImuAttitude_TenthsToDeg(rollTenths),
                            ASW_ImuAttitude_TenthsToDeg(pitchTenths),
                            &leftDuty, &rightDuty, &reverse);

            /* [게이트] 이 프레임을 공유 명령 상태에 반영할 것인가?
             *
             * 원칙 : "평평한 리모컨은 무시한다. 단, 이미 IMU가 운전 중이면
             *         평평함은 곧 정지 명령이므로 반영한다."
             *
             * 두 개의 서로 반대 방향 버그를 동시에 막기 위한 조건이다.
             *  - dutyNonZero    : 실제로 기울인 프레임. 언제나 명령으로 인정한다.
             *  - imuAlreadyActive : 지금 운전대를 쥐고 있는 쪽이 IMU라는 뜻.
             *                     이때의 평평한 프레임은 "손을 뗐다"는 분명한
             *                     의사표시이므로 duty 0을 그대로 저장해야 한다.
             *
             * 둘 다 아닐 때(=평평한데 활성 출처가 SERIAL/CAN/NONE)만 아무것도
             * 건드리지 않는다. 그 경우가 바로 "옆에 켜 둔 리모컨이 휴대폰 앱
             * 조작을 빼앗는" 상황이기 때문이다. 이때 s_lastRxTick까지 그대로
             * 두는 것도 중요하다. 이 타임스탬프는 "현재 출처"의 생존 시각이라,
             * 무관한 평평 IMU 프레임으로 갱신하면 SERIAL 링크가 죽어도 무응답
             * 정지가 무한정 미뤄진다.
             *
             * s_cmdSource를 임계구역 밖에서 읽는 것은 이 파일의 기존 관례와
             * 같다(ASW_Manual_IsCommandTimeout / ASW_Manual_ProcessControl도
             * 그렇게 읽는다). 값 하나짜리 enum이라 찢어질 것이 없고, 최악의
             * 경우라도 게이트 판정이 한 프레임 어긋날 뿐 다음 프레임(100ms)에서
             * 곧바로 자기 수정된다. */
            dutyNonZero      = ((leftDuty != 0u) || (rightDuty != 0u)) ? 1u : 0u;
            imuAlreadyActive = (s_cmdSource == ASW_MANUAL_CMD_SOURCE_IMU) ? 1u : 0u;

            if ((dutyNonZero != 0u) || (imuAlreadyActive != 0u))
            {
                /* 명령으로 인정 : 출처를 IMU로 가져오고 생존 타임스탬프도 갱신한다.
                 * duty가 0일 수도 있는데(위 imuAlreadyActive 경로), 그것이 곧
                 * 정지 명령이라 의도한 값이다. ASW_Manual_ProcessControl()이
                 * 좌/우 0을 보고 다음 제어주기(50ms)에 RTE_Motor_Stop()을 부른다.
                 * 4개 값이 한 덩어리로 보이도록 대입만 임계구역으로 묶는다.
                 * (좌/우 duty가 찢어져 읽히면 엉뚱한 조향이 한 주기 섞여 나간다.) */
                taskENTER_CRITICAL();
                s_imuLeftDuty  = leftDuty;
                s_imuRightDuty = rightDuty;
                s_imuReverse   = reverse;
                s_lastRxTick   = osKernelSysTick();
                s_cmdSource    = ASW_MANUAL_CMD_SOURCE_IMU;
                taskEXIT_CRITICAL();
            }
            else
            {
                /* 평평한 리모컨 + 다른 출처가 활성(또는 아직 아무 출처도 없음).
                 * 조작이 아니므로 s_cmdSource도, s_lastRxTick도, 저장된 duty도
                 * 전부 그대로 둔다. 지금 활성인 출처를 방해하지 않기 위해서다. */
            }

            /* 공유 자세 모듈 갱신은 일부러 임계구역 "밖"에서, 그리고 위 게이트
             * 판정 "밖"에서 무조건 호출한다.
             *  - 임계구역 밖인 이유 : 이 함수는 내부에서 평범한 저장만 하므로,
             *    최우선순위인 CanRxTask가 쥔 임계구역을 더 늘릴 이유가 없다.
             *  - 데드존과 무관하게 무조건인 이유 : 이쪽 소비자들에게는 평평한
             *    프레임도 전부 의미가 있다. 자율주행 PIVOT 과회전 가드는 매 프레임의
             *    yaw가 필요하고(기울기와 아무 상관 없다), ASW_ImuAttitude_IsFresh()의
             *    링크 생존 판정은 "프레임이 계속 도착하는가" 자체를 보기 때문에
             *    한 프레임이라도 빠뜨리면 살아있는 링크를 죽은 것으로 오판한다.
             *    즉 수동 제어의 "조작인가" 판정과 자세 모듈의 "도착했는가" 판정은
             *    서로 다른 질문이고, 위 if/else는 앞의 질문에만 관여한다. */
            /* IMU 값이 이 파일의 statics(duty)와 공유 모듈(각도) 두 곳에 나뉘어
             * 저장되는 것은 의도한 선택이다. 검증이 끝난 수동 기울기 주행 경로를
             * 새 모듈에서 읽도록 리팩터링하는 대안은 당장 얻는 것 없이 위험만
             * 늘리므로, 몇 바이트 중복이 더 싼 거래다. */
            ASW_ImuAttitude_Apply(rollTenths, pitchTenths, yawTenths, remoteCalibrated);
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
         *  규칙을 코드 전체에서 똑같이 지키기 위해 이렇게 맞춰 둔다.)
         * s_cmdSource를 읽고 s_lastRxTick을 쓰므로, CanRxTask와의 경합을 막기
         * 위해 두 문장을 임계구역으로 묶는다. */
        taskENTER_CRITICAL();
        if (s_cmdSource == ASW_MANUAL_CMD_SOURCE_SERIAL)
        {
            s_lastRxTick = osKernelSysTick();
        }
        taskEXIT_CRITICAL();

        /* 여기 있던 블로킹 로그(printf)는 제거했다. 블루투스와 같은 USART1
         * (9600bps)으로 나가면서 한 줄에 30ms 넘게 블로킹되어 50ms 제어주기를
         * 잠식했기 때문이다. */
    }
    else if (ch == 'P')
    {
        RTE_Motor_Stop();
        RTE_Mode_SetDriveMode(RTE_DRIVE_MODE_MANUAL);
        EnterManualIdle();

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
 *              IMU가 활성 출처인 동안에는 평평한 프레임도 명령으로 채택되므로
 *              (0x120 게이트 주석 참조) 타임스탬프가 100ms마다 계속 갱신된다.
 *              즉 여기서 타임아웃이 나는 것은 "리모컨을 평평하게 놓았을 때"가
 *              아니라 "remote가 죽거나 CAN이 끊겨 프레임 자체가 멈췄을 때"다.
 *              기울임을 푸는 정지는 타임아웃이 아니라 duty 0 명령이 다음
 *              제어주기(50ms)에 곧바로 처리한다.
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

    switch (s_cmdSource)
    {
        case ASW_MANUAL_CMD_SOURCE_IMU:
        {
            uint16_t leftDuty;
            uint16_t rightDuty;
            uint8_t  reverse;

            /* CanRxTask가 세 값을 한 덩어리로 쓰므로, 읽을 때도 한 덩어리로
             * 읽는다. (찢어져 읽히면 새 좌 duty에 옛 우 duty가 붙는다.)
             * 기울기 -> duty 변환은 이미 수신 시점에 끝나 있으므로 여기서는
             * 부동소수점 연산을 전혀 하지 않는다. 그 이유는 0x120 수신 처리의
             * 주석 참조. */
            taskENTER_CRITICAL();
            leftDuty  = s_imuLeftDuty;
            rightDuty = s_imuRightDuty;
            reverse   = s_imuReverse;
            taskEXIT_CRITICAL();

            if ((leftDuty == 0u) && (rightDuty == 0u))
            {
                /* 좌/우 duty가 모두 0 = "정지" 명령. 두 경로로 들어온다.
                 *  1) IMU 주행 중에 리모컨을 다시 평평하게 놓은 경우(정상 정지).
                 *     0x120 게이트가 이 duty 0을 그대로 저장해 주므로, 손을 뗀
                 *     뒤 한 주기(50ms) 안에 여기서 멈춘다. 1000ms 타임아웃을
                 *     기다리지 않는다.
                 *  2) 저장된 명령이 아직 없는 상태(모드 재진입 직후 등)의 방어. */
                RTE_Motor_Stop();
            }
            else if (reverse == 0u)
            {
                RTE_Motor_DriveForwardDifferential(leftDuty, rightDuty);
            }
            else
            {
                /* 후진에서도 좌/우 duty를 따로 주어 차등 조향을 유지한다. */
                RTE_Motor_DriveBackwardDifferential(leftDuty, rightDuty);
            }
            break;
        }

        default:
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
                 * (CAN 경로는 remote 노드가 보낸 값을 그대로 넣으므로, 이상한 값이
                 *  왔을 때 안전하게 멈추는 최후의 보루 역할도 겸한다.) */
                default:  RTE_Motor_Stop();                          break;
            }
            break;
    }
}
