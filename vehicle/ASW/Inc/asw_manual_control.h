/*
 * asw_manual_control.h
 *
 *  Layer  : ASW (Application Software)
 *  Module : ASW_MANUAL_CONTROL
 *  Desc   : remote 노드가 블루투스로 수신한 조향/속도 명령을
 *           CAN을 통해 전달받아 모터를 구동하는 수동 주행 로직.
 *           기존 bluetooth.c(USART1 직접 수신) 로직을 CAN 기반으로 대체.
 *           휴대폰 블루투스 앱(USART1 직접 수신)의 단일문자 명령도 함께 처리한다.
 *
 *  Naming Rule : ASW_Manual_<Verb><Object>()
 */

#ifndef __ASW_MANUAL_CONTROL_H
#define __ASW_MANUAL_CONTROL_H

#include "mcal_can.h"

#define ASW_MANUAL_SPEED_DUTY        600u   /* 수동주행 기본 속도 Duty */
#define ASW_MANUAL_CMD_TIMEOUT_MS    1000u  /* CAN 명령 미수신 안전정지 기준 */
#define ASW_MANUAL_SERIAL_SILENCE_MS 5000u  /* BT 무응답(연결끊김/앱다운) 시 강제정지 기준 */

/* ------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------ */
void ASW_Manual_Init(void);
void ASW_Manual_ProcessControl(void);
void ASW_Manual_ApplyCanCommand(const McalCanMsg_t *p_msg);
void ASW_Manual_ApplySerialChar(uint8_t ch);
uint8_t ASW_Manual_IsCommandTimeout(void);

#endif /* __ASW_MANUAL_CONTROL_H */
