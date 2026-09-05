/*
 * asw_imu_attitude.c
 *  Layer  : ASW
 *  Module : ASW_IMU_ATTITUDE
 *
 *  remote 노드가 CAN(0x120)으로 보내는 IMU 자세를 그대로 보관하고,
 *  읽는 쪽이 원하는 축만 도(degree) 단위로 꺼내 가게 하는 공유 저장소.
 *
 *  [임계구역이 하나도 없는 이유]
 *   각 필드는 정확히 한 곳(CanRxTask)에서만 쓰고, 모든 읽는 쪽은 한 번에
 *   한 필드만 읽는다 (자율주행 피벗 가드는 yaw만, 유효성은 유효성만).
 *   즉 여러 필드의 일관성을 요구하는 소비자가 없으므로 묶어서 지킬 것이
 *   애초에 없다. roll+pitch를 반드시 한 쌍으로 함께 소비해서(한쪽만 새 값이면
 *   엉뚱한 조향이 나간다) 임계구역이 꼭 필요한 asw_manual_control.c와는
 *   이 점이 다르다. 게다가 tenths를 int16으로 저장하므로 개별 쓰기 자체가
 *   Cortex-M3에서 원자적이라, 찢어진 값(torn write)을 읽을 여지도 없다.
 */
#include "asw_imu_attitude.h"

/* 마지막으로 수신한 "원본" 자세각(0.1도 단위). 도(degree)로 가공하지 않고
 * 받은 그대로 들고 간다. 변환 시점을 뒤로 미루는 이유는 게터 주석 참조.
 * volatile : 쓰는 쪽(CanRxTask)과 읽는 쪽(CtrlTask)이 서로 다른 태스크라
 *            컴파일러가 값을 레지스터에 캐싱해 두지 못하게 막는다. */
static volatile int16_t s_rollTenths  = 0;
static volatile int16_t s_pitchTenths = 0;
static volatile int16_t s_yawTenths   = 0;

/* remote가 캘리브레이션을 마친 자세를 한 번이라도 보냈는지의 단방향 래치.
 * 절대 0으로 되돌리지 않는다. remote는 부팅 시 한 번 캘리브레이션하고 그
 * 뒤로는 계속 캘리브레이션된 상태이므로, 이것은 "부팅 시 준비 완료" 래치이지
 * 살아있는 링크 상태 플래그가 아니다. 링크가 끊겼는지는 각 소비자가 자기
 * 기준(무응답 시간 등)으로 따로 판단할 몫이다. */
static volatile uint8_t s_valid = 0u;

/*
 * ASW_ImuAttitude_Apply : CanRxTask가 0x120 프레임을 파싱한 직후 호출.
 *   받은 tenths 값을 그대로 저장만 하고 아무 해석도 하지 않는다.
 *   (최우선순위 태스크의 수신 경로를 최대한 짧게 유지하기 위함)
 */
void ASW_ImuAttitude_Apply(int16_t rollTenths, int16_t pitchTenths, int16_t yawTenths, uint8_t remoteCalibrated)
{
    s_rollTenths  = rollTenths;
    s_pitchTenths = pitchTenths;
    s_yawTenths   = yawTenths;

    if (remoteCalibrated != 0u)
    {
        /* 단방향 래치 : 한 번 서면 끝. else로 0을 쓰는 분기를 두지 않는 것이
         * 이 모듈의 의도다. (해제 경로를 만들면 캘리브레이션 비트가 한 프레임
         * 흔들렸을 때 자율주행 가드가 통째로 꺼져 버린다.) */
        s_valid = 1u;
    }
}

/*
 * 게터들 : 저장해 둔 tenths를 이 시점에 도(degree)로 나눈다.
 *   CanRxTask는 최우선순위 태스크이고 이 MCU에는 FPU가 없어 부동소수점이
 *   소프트웨어로 도는 비싼 연산이다. 그래서 변환을 CAN 수신 경로에서 하지 않고,
 *   50ms 주기의 CtrlTask가 읽을 때 지불하게 한다. 수신 경로에는 int16 대입만
 *   남는데, int16 쓰기는 Cortex-M3에서 자연히 원자적이라 찢어진 값 문제까지
 *   덤으로 사라진다.
 */
float ASW_ImuAttitude_GetRollDeg(void)
{
    return (float)s_rollTenths / 10.0f;
}

float ASW_ImuAttitude_GetPitchDeg(void)
{
    return (float)s_pitchTenths / 10.0f;
}

float ASW_ImuAttitude_GetYawDeg(void)
{
    return (float)s_yawTenths / 10.0f;
}

/*
 * ASW_ImuAttitude_IsValid : 위 각도를 믿어도 되는지.
 *   0이면 remote가 아직 캘리브레이션된 자세를 한 번도 안 보낸 것이므로,
 *   소비자는 IMU를 쓰는 판단 자체를 건너뛰고 기존 로직으로 동작해야 한다.
 */
uint8_t ASW_ImuAttitude_IsValid(void)
{
    return s_valid;
}
