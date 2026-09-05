/*
 * mcal_can.h
 *
 *  Layer  : BSW / MCAL (통신 / OS 인터페이스)
 *  Module : MCAL_CAN
 *  Desc   : 2-Node 공유 CAN 버스(Node1 NUCLEO 마스터, Node2 STM32(본 노드))
 *           송수신을 RTOS Mail Queue로 브릿지하는 래퍼.
 *           CAN ISR(HAL_CAN_RxFifo0MsgPendingCallback)에서는 최소한의
 *           작업(Read + MailPut)만 수행하고, 실제 처리는 CanRxTask에서
 *           담당한다.
 *
 *  주의: CMSIS-RTOS v1의 osMessageQDef는 32bit 값(포인터/정수) 1개만
 *        전달 가능하므로, 구조체(McalCanMsg_t) 전체를 안전하게 전달하기
 *        위해 osMailQDef(Mail Queue)를 사용한다. (ISR-safe alloc/put)
 *
 *  Naming Rule : MCAL_CAN_<Verb><Object>()
 */

#ifndef __MCAL_CAN_H
#define __MCAL_CAN_H

#include "main.h"
#include "can.h"
#include "cmsis_os.h"

/* ------------------------------------------------------------------------
 * CAN ID 정의 (Communication Matrix 확정본)
 *   0x100 ~ 0x1FF : remote 노드 (Master)       -> 전체 Broadcast
 *   0x200 ~ 0x2FF : vehicle 노드 (본 노드)     -> 상태/응답
 *   0x3F0         : vehicle 노드 전용 생존 신호(Heartbeat) 송신 ID
 * ------------------------------------------------------------------------ */
#define MCAL_CAN_ID_REMOTE_MODE_CMD      0x100u  /* 수동/자동 모드 전환 */
#define MCAL_CAN_ID_REMOTE_MANUAL_CMD    0x110u  /* 수동 조향/속도 명령 */
#define MCAL_CAN_ID_REMOTE_IMU_ATTITUDE  0x120u  /* remote 노드 IMU roll/pitch (0.1도 단위) */
#define MCAL_CAN_ID_VEHICLE_STATUS       0x200u  /* 본 노드 상태/거리값 응답 */
#define MCAL_CAN_ID_VEHICLE_HEARTBEAT    0x3F0u  /* 본 노드 생존 신호 */

#define MCAL_CAN_RX_QUEUE_LEN            8u
#define MCAL_CAN_DLC_MAX                 8u

#define MCAL_CAN_TIMEOUT_MASTER_MS       1000u   /* Fail-Safe 판단 기준 */

/* ------------------------------------------------------------------------
 * CAN 메시지 표준 포맷 (Mail Queue 아이템)
 * ------------------------------------------------------------------------ */
typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[MCAL_CAN_DLC_MAX];
} McalCanMsg_t;

/* SLAVE_STATUS 상태 코드 */
typedef enum {
    MCAL_CAN_STATE_MANUAL = 0,
    MCAL_CAN_STATE_AUTO   = 1,
    MCAL_CAN_STATE_FAULT  = 2
} McalCanSlaveState_t;

/* ------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------ */
void     MCAL_CAN_Init(void);
void     MCAL_CAN_ConfigFilter(void);
osStatus MCAL_CAN_Transmit(uint32_t id, const uint8_t *p_data, uint8_t dlc);
osStatus MCAL_CAN_Receive(McalCanMsg_t *p_msg, uint32_t timeout_ms);
void     MCAL_CAN_BroadcastSlaveStatus(McalCanSlaveState_t state,
                                    uint32_t dist_left,
                                    uint32_t dist_front,
                                    uint32_t dist_right);

/* HEARTBEAT(0x3F0) 생존 신호 송신 헬퍼. CanTxTask에서 약 1초 주기로 호출.
 * healthBits는 1바이트 비트필드이며, 각 비트의 의미는 아래와 같다.
 *   bit0 : CAN 셀이 버스오프 복구에 실패해 막혀 있음 (g_vdiag_can_stuck)
 *   bit1 : CAN 오류 수동(Error Passive) 상태  (g_vdiag_can_epvf)
 *   bit2 : CAN 오류 경고(Error Warning) 상태  (g_vdiag_can_ewgf)
 *   bit3 : CAN 수신 인터럽트가 켜져 있지 않음 (g_vdiag_can_notify_ok == 0)
 *   bit4 : 제어 루프(CtrlTask)가 멈춤 - 생존 카운터가 지난 주기 이후 안 늘어남
 *   bit5~7 : 예약(항상 0)
 * 0이면 "모든 항목 정상"을 뜻한다. */
void     MCAL_CAN_BroadcastHeartbeat(uint8_t healthBits);

/* HAL_CAN_RxFifo0MsgPendingCallback (ISR Context) 에서 호출 */
void     MCAL_CAN_HandleRxFifoIsr(CAN_HandleTypeDef *p_hcan);

extern osMailQId g_mcalCanRxMailQueueHandle;

#endif /* __MCAL_CAN_H */
