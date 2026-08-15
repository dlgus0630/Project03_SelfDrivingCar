/*
 * mcal_bt_serial.h
 *
 *  Layer  : BSW / MCAL (통신 / OS 인터페이스)
 *  Module : MCAL_BT_SERIAL
 *  Desc   : 휴대폰 블루투스 앱(HC-06, USART1)이 보내는 1바이트 명령을
 *           인터럽트로 수신해 보관한다. mcal_can과 동일하게 ISR에서는
 *           최소 작업만 하고, 실제 처리는 CtrlTask가 폴링해서 담당한다.
 *
 *  Naming Rule : MCAL_BtSerial_<Verb><Object>()
 */
#ifndef __MCAL_BT_SERIAL_H
#define __MCAL_BT_SERIAL_H

#include "main.h"

void    MCAL_BtSerial_Init(void);

/* 소비되지 않은 새 명령이 있으면 그 문자를, 없으면 0을 반환하고 내부 플래그를
 * 리셋한다(소비형 읽기 - 같은 명령을 두 번 반환하지 않음) */
uint8_t MCAL_BtSerial_GetChar(void);

/* HAL_UART_RxCpltCallback (main.c USER CODE BEGIN 4) 에서 반드시 호출 */
void    MCAL_BtSerial_HandleRxCpltIsr(UART_HandleTypeDef *p_huart);

/* HAL_UART_ErrorCallback (main.c USER CODE BEGIN 4) 에서 반드시 호출.
 * 오버런(ORE) 등 수신 에러가 나면 HAL이 수신을 중단해 버려 이후 바이트가
 * 들어오지 않는다. 이 함수가 에러 플래그를 지우고 수신을 재등록해 복구한다. */
void    MCAL_BtSerial_HandleErrorIsr(UART_HandleTypeDef *p_huart);

/* ---- 임시 진단용 인터페이스 - 원인 확인 후 제거 ---- */

/* 정상 수신 ISR이 호출된 총 횟수 */
uint32_t MCAL_BtSerial_GetRxCount(void);

/* 에러(오버런 등) ISR이 호출된 총 횟수 */
uint32_t MCAL_BtSerial_GetErrCount(void);

/* 마지막으로 받은 문자를 "소비하지 않고" 그대로 읽는다(엿보기).
 * MCAL_BtSerial_GetChar()와 달리 내부 플래그를 건드리지 않으므로,
 * 진단 로그가 호출해도 제어 로직이 명령을 놓치지 않는다. */
uint8_t MCAL_BtSerial_PeekLastChar(void);

#endif /* __MCAL_BT_SERIAL_H */
