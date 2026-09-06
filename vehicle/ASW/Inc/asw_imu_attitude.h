/*
 * asw_imu_attitude.h
 *
 *  Layer  : ASW (Application Software)
 *  Module : ASW_IMU_ATTITUDE
 *  Desc   : remote 노드가 CAN 0x120으로 보내는 IMU 자세(roll/pitch/yaw)를
 *           여러 소비자가 공유해 읽을 수 있게 보관하는 저장소.
 *           수신 경로(CanRxTask)는 받은 값을 여기에 넣기만 하고, 해석은
 *           읽는 쪽이 각자 필요한 만큼만 한다.
 *           현재 소비자는 자율주행 PIVOT 과회전 가드(yaw).
 *           (수동 모드의 기울기 조향은 자체 저장소를 따로 쓰므로 여기에
 *            의존하지 않는다 - 서로 다른 모듈의 수명주기를 섞지 않기 위함)
 *
 *  Naming Rule : ASW_ImuAttitude_<Verb><Object>()
 */

#ifndef __ASW_IMU_ATTITUDE_H
#define __ASW_IMU_ATTITUDE_H

#include <stdint.h>

/* ------------------------------------------------------------------------
 * 0.1도 단위(tenths) -> 도(degree) 변환 헬퍼
 *
 *  CAN 프레임의 각도는 전부 0.1도 단위 정수로 실려 온다. 이 "전송 단위 ->
 *  도" 변환식을 파일마다 (float)x / 10.0f 로 흩어 쓰면 나중에 remote가
 *  전송 배율을 바꿨을 때 고쳐야 할 곳을 놓치기 쉽다. 그래서 변환은 이 한
 *  곳에만 쓰고, 두 소비자(.c)가 모두 보도록 헤더에 static inline으로 둔다.
 *  (인라인이라 함수 호출 비용도 없고, 별도 .c를 만들 필요도 없다.)
 * ------------------------------------------------------------------------ */
static inline float ASW_ImuAttitude_TenthsToDeg(int16_t tenths)
{
    return (float)tenths / 10.0f;
}

/* ------------------------------------------------------------------------
 * Public API
 *
 *  Apply()  : CAN 수신 경로에서 호출. 각도는 CAN 프레임에 실려 온 그대로,
 *             즉 0.1도 단위 정수(tenths)로 넘긴다. 여기서 도(degree)로
 *             바꾸지 않는 이유는 구현 파일 주석 참조.
 *             호출될 때마다 수신 시각도 함께 기록한다(IsFresh() 참조).
 *  Get*()   : 소비자(CtrlTask)가 호출. 이 시점에 도(degree)로 변환된다.
 *             GetYawDeg()만 현재 소비자가 있고(자율주행 PIVOT 과회전 가드),
 *             GetRollDeg()/GetPitchDeg()는 지금 호출자가 없다. 자세한 이유는
 *             구현 파일의 각 함수 주석 참조 - 의도적으로 남겨 둔 표면이다.
 *  IsValid(): remote가 캘리브레이션을 마친 자세를 한 번이라도 보냈는지의
 *             "부팅 시 준비 완료" 단방향 래치. 한 번 서면 절대 내려가지
 *             않으므로, 링크가 지금 살아있는지는 알려주지 못한다.
 *  IsFresh(): 마지막 0x120 프레임을 받은 지 maxAgeMs 이내인지. 이쪽이
 *             "지금 링크가 살아있는가"를 보는 실시간 판정이다.
 *
 *  [두 판정을 반드시 함께 쓸 것]
 *   IsValid()는 과거에 한 번이라도 준비가 됐는지(래치), IsFresh()는 지금도
 *   값이 들어오고 있는지(생존)를 본다. 서로 다른 질문이라 한쪽만으로는
 *   부족하다. remote 전원이 꺼져도 IsValid()는 1로 남아 있으므로, 각도를
 *   실제로 신뢰하려면 (IsValid() && IsFresh(...)) 두 조건을 모두 확인해야 한다.
 * ------------------------------------------------------------------------ */
void    ASW_ImuAttitude_Apply(int16_t rollTenths, int16_t pitchTenths, int16_t yawTenths, uint8_t remoteCalibrated);
float   ASW_ImuAttitude_GetRollDeg(void);
float   ASW_ImuAttitude_GetPitchDeg(void);
float   ASW_ImuAttitude_GetYawDeg(void);
uint8_t ASW_ImuAttitude_IsValid(void);
uint8_t ASW_ImuAttitude_IsFresh(uint32_t maxAgeMs);

#endif /* __ASW_IMU_ATTITUDE_H */
