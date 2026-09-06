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
 *   이 점이 다르다. 게다가 tenths는 int16, 수신 시각은 정렬된 uint32라
 *   개별 쓰기 자체가 Cortex-M3에서 원자적이므로, 찢어진 값(torn write)을
 *   읽을 여지도 없다.
 */
#include "asw_imu_attitude.h"
#include "cmsis_os.h"   /* osKernelSysTick() : 마지막 수신 시각 기록용 */

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

/* 마지막으로 0x120 프레임을 받은 시각(ms tick). 위 s_valid가 답하지 못하는
 * "지금도 링크가 살아있는가"를 판정하기 위한 값이며, Apply()가 호출될 때마다
 * 캘리브레이션 플래그와 무관하게 무조건 갱신된다.
 * volatile : 쓰는 쪽(CanRxTask)과 읽는 쪽(소비자 태스크)이 서로 다르다. */
static volatile uint32_t s_lastRxTick = 0u;

/*
 * ASW_ImuAttitude_Apply : CanRxTask가 0x120 프레임을 파싱한 직후 호출.
 *   받은 tenths 값을 그대로 저장만 하고 아무 해석도 하지 않는다.
 *   (최우선순위 태스크의 수신 경로를 최대한 짧게 유지하기 위함)
 *   수신 시각(s_lastRxTick)은 프레임 내용과 무관하게 항상 남긴다. "프레임이
 *   도착했다"는 사실 자체가 링크 생존의 증거이기 때문이다.
 */
void ASW_ImuAttitude_Apply(int16_t rollTenths, int16_t pitchTenths, int16_t yawTenths, uint8_t remoteCalibrated)
{
    s_rollTenths  = rollTenths;
    s_pitchTenths = pitchTenths;
    s_yawTenths   = yawTenths;

    /* 조건 없이 갱신 : 캘리브레이션 전이든 후든, 기울기가 평평하든 아니든
     * 프레임이 왔다는 것 자체가 remote가 살아있다는 증거다. */
    s_lastRxTick = osKernelSysTick();

    if (remoteCalibrated != 0u)
    {
        /* 단방향 래치 : 한 번 서면 끝. else로 0을 쓰는 분기를 두지 않는 것이
         * 이 모듈의 의도다. (해제 경로를 만들면 캘리브레이션 비트가 한 프레임
         * 흔들렸을 때 자율주행 가드가 통째로 꺼져 버린다.) */
        s_valid = 1u;
    }
}

/*
 * 게터들 : 저장해 둔 tenths를 이 시점에 도(degree)로 바꿔 돌려준다.
 *   변환식 자체는 헤더의 ASW_ImuAttitude_TenthsToDeg()에 한 번만 적어 두고
 *   여기서는 그것만 부른다. (0.1도 전송 단위 -> 도 변환을 한 곳에 모아 두기
 *   위해서다. 자세한 이유는 헤더의 해당 헬퍼 주석 참조.)
 *   CanRxTask는 최우선순위 태스크이고 이 MCU에는 FPU가 없어 부동소수점이
 *   소프트웨어로 도는 비싼 연산이다. 그래서 변환을 CAN 수신 경로에서 하지 않고,
 *   이 게터를 읽는 소비자 태스크가 지불하게 한다. 수신 경로에는 정수 대입만
 *   남는데, 그 대입들은 Cortex-M3에서 자연히 원자적이라 찢어진 값 문제까지
 *   덤으로 사라진다.
 */
/* [현재 호출자 없음] 수동 기울기 주행은 asw_manual_control.c가 자체 저장소를
 * 따로 들고 가는 쪽을 의도적으로 택했기 때문에(검증 끝난 경로를 건드리지 않기
 * 위한 문서화된 절충), roll은 지금 아무도 읽지 않는다. 그래도 이 모듈이
 * roll/pitch/yaw 세 축의 공용 보관소라는 설계를 유지하기 위해, 저장 필드와
 * 짝을 맞춘 게터로 남겨 둔다(죽어 있지만 올바른 표면). */
float ASW_ImuAttitude_GetRollDeg(void)
{
    return ASW_ImuAttitude_TenthsToDeg(s_rollTenths);
}

/* [현재 호출자 없음] GetRollDeg()와 같은 이유로 남겨 둔다. 삭제하면 세 축 중
 * yaw만 꺼낼 수 있는 비대칭 모듈이 되어, 나중에 roll/pitch 소비자가 생겼을 때
 * 다시 만들어야 한다. */
float ASW_ImuAttitude_GetPitchDeg(void)
{
    return ASW_ImuAttitude_TenthsToDeg(s_pitchTenths);
}

float ASW_ImuAttitude_GetYawDeg(void)
{
    return ASW_ImuAttitude_TenthsToDeg(s_yawTenths);
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

/*
 * ASW_ImuAttitude_IsFresh : 마지막 0x120 프레임을 받은 지 maxAgeMs 이내인가.
 *   IsValid()가 "부팅 시 한 번이라도 준비됐는지"를 보는 단방향 래치인 반면,
 *   이쪽은 "지금도 값이 들어오고 있는지"를 보는 실시간 생존 판정이다.
 *   remote 전원이 꺼지거나 CAN이 끊겨도 IsValid()는 1로 남으므로, 소비자는
 *   두 함수를 반드시 함께 확인해야 한다. (IsValid() && IsFresh(...))
 *
 *   부호 없는 뺄셈이라 tick이 32비트 한 바퀴를 돌아도(약 49.7일) 경과 시간이
 *   올바르게 나온다. asw_manual_control.c의 ASW_Manual_IsCommandTimeout()이
 *   자기 저장소에 대해 쓰는 것과 완전히 같은 방식이며, 다만 그쪽은 "만료됐나"를
 *   묻고 이쪽은 "아직 살아있나"를 묻는 반대 극성이라는 점만 다르다.
 *   maxAgeMs는 소비자가 정한다. remote는 100ms 주기로 보내므로, 몇 프레임의
 *   유실을 허용할지에 따라 값을 고르면 된다.
 */
uint8_t ASW_ImuAttitude_IsFresh(uint32_t maxAgeMs)
{
    return ((osKernelSysTick() - s_lastRxTick) <= maxAgeMs) ? 1u : 0u;
}
