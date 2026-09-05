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
 * Public API
 *
 *  Apply()  : CAN 수신 경로에서 호출. 각도는 CAN 프레임에 실려 온 그대로,
 *             즉 0.1도 단위 정수(tenths)로 넘긴다. 여기서 도(degree)로
 *             바꾸지 않는 이유는 구현 파일 주석 참조.
 *  Get*()   : 소비자(CtrlTask)가 호출. 이 시점에 도(degree)로 변환된다.
 *  IsValid(): remote가 캘리브레이션을 마친 자세를 한 번이라도 보냈는지.
 *             각도를 신뢰해도 되는지 판단하는 용도로만 쓴다.
 * ------------------------------------------------------------------------ */
void    ASW_ImuAttitude_Apply(int16_t rollTenths, int16_t pitchTenths, int16_t yawTenths, uint8_t remoteCalibrated);
float   ASW_ImuAttitude_GetRollDeg(void);
float   ASW_ImuAttitude_GetPitchDeg(void);
float   ASW_ImuAttitude_GetYawDeg(void);
uint8_t ASW_ImuAttitude_IsValid(void);

#endif /* __ASW_IMU_ATTITUDE_H */
