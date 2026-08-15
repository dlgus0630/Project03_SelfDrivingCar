/*
 * mcal_bt_serial.c
 *  Layer  : BSW / MCAL
 *  Module : MCAL_BT_SERIAL
 *
 *  USART1(HC-06)로 들어오는 휴대폰 앱 명령을 1바이트씩 인터럽트로 받는다.
 *  지금은 별도 큐 없이 "최신 값 하나만 보관"하는 방식인데, 이는 버튼을 누르는
 *  속도가 제어주기(50ms)보다 느리다는 전제에 기대고 있다. 그러나 이 전제는
 *  아직 검증되지 않았다 - 휴대폰 앱이 방향키를 누르고 있는 동안 같은 문자를
 *  50ms보다 훨씬 빠르게 반복 전송하면 수신 오버런(ORE)이나 명령 유실이
 *  생길 수 있다. 현재 아래 진단용 카운터로 실제 수신/에러 횟수를 확인하는
 *  중이며, 결과를 보고 큐 도입 여부를 결정한다.
 */
#include "mcal_bt_serial.h"
#include "usart.h"

static volatile uint8_t s_rxByte;
static volatile uint8_t s_lastChar = 0;
static volatile uint8_t s_hasNew   = 0;

/* 임시 진단용 로그 - 원인 확인 후 제거.
 * ISR 호출 횟수만 세는 카운터다. ISR 안에서는 printf 같은 블로킹 UART 송신을
 * 절대 하지 않는다(인터럽트가 지연되어 오히려 수신을 더 놓치게 된다). */
static volatile uint32_t s_rxCount  = 0;
static volatile uint32_t s_errCount = 0;

void MCAL_BtSerial_Init(void)
{
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rxByte, 1);
}

uint8_t MCAL_BtSerial_GetChar(void)
{
    uint8_t ch = 0u;

    if (s_hasNew)
    {
        ch       = s_lastChar;
        s_hasNew = 0u;
    }
    return ch;
}

/* 임시 진단용 - 원인 확인 후 제거.
 * 값을 "소비하지 않고" 마지막으로 받은 문자만 그대로 돌려준다. s_hasNew 플래그를
 * 전혀 건드리지 않으므로, 진단 로그가 이 함수를 호출해도 제어 로직이 쓰는
 * MCAL_BtSerial_GetChar()가 명령을 놓치는 일은 없다. */
uint8_t MCAL_BtSerial_PeekLastChar(void)
{
    return s_lastChar;
}

/* 임시 진단용 - 원인 확인 후 제거. 정상 수신 ISR이 몇 번 돌았는지 반환 */
uint32_t MCAL_BtSerial_GetRxCount(void)
{
    return s_rxCount;
}

/* 임시 진단용 - 원인 확인 후 제거. 에러(오버런 등) ISR이 몇 번 돌았는지 반환 */
uint32_t MCAL_BtSerial_GetErrCount(void)
{
    return s_errCount;
}

void MCAL_BtSerial_HandleRxCpltIsr(UART_HandleTypeDef *p_huart)
{
    if (p_huart->Instance != USART1) { return; }

    s_lastChar = s_rxByte;
    s_hasNew   = 1u;

    s_rxCount++; /* 임시 진단용 카운터 - 원인 확인 후 제거 */

    /* 인터럽트 수신은 1바이트마다 자동으로 꺼지므로 재등록 필수.
     * 다만 이 재등록이 항상 HAL_OK를 반환한다는 보장은 없다. 예를 들어 HAL이
     * 아직 수신 상태를 정리하지 못했으면 HAL_BUSY가 나올 수 있다. 그런 경우에도
     * 여기서 따로 처리하지 않는 이유는, 수신이 실제로 멈추면 에러 인터럽트가
     * 떠서 MCAL_BtSerial_HandleErrorIsr()가 플래그를 지우고 다시 재등록해
     * 복구해 주기 때문이다. */
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rxByte, 1);
}

void MCAL_BtSerial_HandleErrorIsr(UART_HandleTypeDef *p_huart)
{
    if (p_huart->Instance != USART1) { return; }

    s_errCount++; /* 임시 진단용 카운터 - 원인 확인 후 제거 */

    /* 수신 에러 플래그(ORE 오버런 / NE 노이즈 / FE 프레이밍 / PE 패리티)를 지운다.
     * STM32F1에서는 이 네 플래그를 지우는 매크로가 전부 동일하며, 내부적으로
     * "SR 레지스터를 읽고 -> DR 레지스터를 읽는" 시퀀스다. 즉 아래 한 번의 호출로
     * 네 플래그가 모두 지워지므로 매크로를 네 번 부를 필요가 없다.
     * (오히려 여러 번 부르면 그 사이 새로 도착한 바이트를 DR에서 읽어 버려
     *  명령을 잃을 수 있다.)
     * 이 플래그를 지우지 않으면 HAL이 수신을 중단한 채로 남아 이후 바이트가
     * 영영 들어오지 않는다 - 방향키를 계속 누르고 있는데도 차가 스스로 멈추는
     * 증상의 원인으로 의심되는 지점이다. */
    __HAL_UART_CLEAR_OREFLAG(p_huart);

    /* HAL 내부 에러코드도 초기화해 다음 수신이 정상 상태에서 시작되게 한다 */
    p_huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* 수신 재등록. 만약 HAL이 이미 수신을 살려둔 상태(RxState가 BUSY_RX)라면
     * 이 호출은 HAL_BUSY를 반환하는데, 그건 "수신이 이미 살아 있다"는 뜻이므로
     * 문제가 되지 않는다(무시해도 안전하다). */
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rxByte, 1);
}
