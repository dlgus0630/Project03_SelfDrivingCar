/*
 * mcal_can.c
 *  Layer  : BSW / MCAL
 *  Module : MCAL_CAN
 */
#include "mcal_can.h"

/* Private variables (s_ prefix) ------------------------------------------ */
static CAN_FilterTypeDef  s_canFilterConfig;
static volatile uint8_t   s_heartbeatCounter = 0;

/* Public (extern, g_ prefix) ---------------------------------------------- */
osMailQId g_mcalCanRxMailQueueHandle = NULL;

/* Mail Queue 정의: MCAL_CAN_RX_QUEUE_LEN개의 McalCanMsg_t 아이템 풀 생성 */
osMailQDef(canRxMailQueue, MCAL_CAN_RX_QUEUE_LEN, McalCanMsg_t);

/* ------------------------------------------------------------------------
 * MCAL_CAN_Init : Mail Queue 생성 + 필터 설정 + 인터럽트 시작
 * ------------------------------------------------------------------------ */
void MCAL_CAN_Init(void)
{
    g_mcalCanRxMailQueueHandle = osMailCreate(osMailQ(canRxMailQueue), NULL);
    if (g_mcalCanRxMailQueueHandle == NULL)
    {
        Error_Handler();
    }

    MCAL_CAN_ConfigFilter();

    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ------------------------------------------------------------------------
 * MCAL_CAN_ConfigFilter
 *   Node2가 수신할 ID 대역: 0x100~0x1FF(Master)
 *   Mask: 0x700 (Bit 10:8 검사) -> StdId[10:8] 패턴이 001 인 것만 통과.
 *   ※ 2-Node 구성이므로 본 노드가 수신할 대역은 마스터 대역 하나뿐이며,
 *     FilterBank 0 한 개만 사용한다. (0x200/0x3F0은 본 노드가 송신하는
 *     ID이므로 수신 필터에 등록하지 않는다.)
 * ------------------------------------------------------------------------ */
void MCAL_CAN_ConfigFilter(void)
{
    /* FilterBank 0 : 0x100 ~ 0x1FF (Master 대역) */
    s_canFilterConfig.FilterBank           = 0;
    s_canFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    s_canFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    s_canFilterConfig.FilterIdHigh         = (0x100u << 5);
    s_canFilterConfig.FilterIdLow          = 0x0000;
    s_canFilterConfig.FilterMaskIdHigh     = (0x700u << 5); /* 상위 3bit(0x1xx) 일치 검사 */
    s_canFilterConfig.FilterMaskIdLow      = 0x0000;
    s_canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    s_canFilterConfig.FilterActivation     = ENABLE;
    s_canFilterConfig.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan, &s_canFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ------------------------------------------------------------------------
 * MCAL_CAN_Transmit : 임의 ID 단건 송신 (Task Context 전용)
 * ------------------------------------------------------------------------ */
osStatus MCAL_CAN_Transmit(uint32_t id, const uint8_t *p_data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint32_t freeLevel;

    if (dlc > MCAL_CAN_DLC_MAX)
    {
        dlc = MCAL_CAN_DLC_MAX;
    }

    txHeader.StdId = id;
    txHeader.IDE   = CAN_ID_STD;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = dlc;

    /* [진단] 비어 있는 송신 메일박스 수(0~3)를 전역에 기록한다.
     * 기존에도 호출하던 함수의 결과를 지역변수에 한 번만 담아 쓰는 것이므로
     * 판정 조건과 동작은 기존과 완전히 동일하다.
     * 이 값이 계속 0으로 붙어 있으면 메일박스 3개가 전부 "송신 대기"로 막혀
     * 있다는 뜻이고, 곧 아무도 ACK를 주지 않아 프레임이 버스 밖으로 나가지
     * 못하고 있다는 신호다. */
    freeLevel = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    g_vdiag_can_free_mb = freeLevel;

    if (freeLevel == 0u)
    {
        /* [진단] 메일박스가 꽉 차서 송신 요청 자체를 넣지 못한 경우도
         * "송신 요청 실패"로 함께 집계한다. 그래야 Live Expressions에서
         * tx_ok가 멈춘 이유가 "태스크가 안 도는 것"인지 "메일박스가 막힌 것"
         * 인지 구분된다. 반환값과 동작은 기존과 동일하다. */
        g_vdiag_can_tx_fail++;
        return osErrorResource; /* 3개 Mailbox 모두 사용 중 (버스 혼잡) */
    }

    if (HAL_CAN_AddTxMessage(&hcan, &txHeader, (uint8_t *)p_data, &txMailbox) != HAL_OK)
    {
        g_vdiag_can_tx_fail++;  /* [진단] 송신 요청 실패 */
        return osErrorOS;
    }

    g_vdiag_can_tx_ok++;        /* [진단] 송신 요청(메일박스 적재) 성공 */
    return osOK;
}

/* ------------------------------------------------------------------------
 * MCAL_CAN_BroadcastSlaveStatus : SLAVE_STATUS(0x200) 주기 송신 헬퍼
 *   CanTxTask에서 100ms 주기로 호출
 * ------------------------------------------------------------------------ */
void MCAL_CAN_BroadcastSlaveStatus(McalCanSlaveState_t state,
                                uint32_t dist_left,
                                uint32_t dist_front,
                                uint32_t dist_right)
{
    uint8_t payload[8] = {0};

    payload[0] = (uint8_t)state;
    payload[1] = (dist_left  > 255u) ? 255u : (uint8_t)dist_left;
    payload[2] = (dist_front > 255u) ? 255u : (uint8_t)dist_front;
    payload[3] = (dist_right > 255u) ? 255u : (uint8_t)dist_right;
    payload[4] = s_heartbeatCounter++;   /* Rolling Counter */
    payload[5] = 0u;                     /* Fault Flags (추후 확장) */

    (void)MCAL_CAN_Transmit(MCAL_CAN_ID_SLAVE_STATUS, payload, 8u);
}

/* ------------------------------------------------------------------------
 * MCAL_CAN_Receive : CanRxTask에서 Mail Queue Pop (Blocking)
 * ------------------------------------------------------------------------ */
osStatus MCAL_CAN_Receive(McalCanMsg_t *p_msg, uint32_t timeout_ms)
{
    osEvent evt = osMailGet(g_mcalCanRxMailQueueHandle, timeout_ms);

    if (evt.status == osEventMail)
    {
        McalCanMsg_t *p_received = (McalCanMsg_t *)evt.value.p;
        *p_msg = *p_received;
        osMailFree(g_mcalCanRxMailQueueHandle, p_received);
        return osOK;
    }
    return osErrorOS; /* Timeout 또는 큐 비어있음 */
}

/* ------------------------------------------------------------------------
 * MCAL_CAN_HandleRxFifoIsr : ISR Context
 *   - main.c의 HAL_CAN_RxFifo0MsgPendingCallback에서 위임 호출됨
 *   - ISR에서는 최소 작업만: Mail Alloc(0 timeout) -> Read -> Put
 *   - Alloc 실패(풀 가득 참) 시 메시지는 드롭됨 (Overrun 정책)
 * ------------------------------------------------------------------------ */
void MCAL_CAN_HandleRxFifoIsr(CAN_HandleTypeDef *p_hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    McalCanMsg_t *p_mailMsg;

    p_mailMsg = osMailAlloc(g_mcalCanRxMailQueueHandle, 0);
    if (p_mailMsg == NULL)
    {
        return; /* 풀 가득 참: 이번 메시지는 드롭 (통계 필요 시 카운터 추가) */
    }

    if (HAL_CAN_GetRxMessage(p_hcan, CAN_RX_FIFO0, &rxHeader, p_mailMsg->data) != HAL_OK)
    {
        osMailFree(g_mcalCanRxMailQueueHandle, p_mailMsg);
        return;
    }

    p_mailMsg->id  = rxHeader.StdId;
    p_mailMsg->dlc = (uint8_t)rxHeader.DLC;

    osMailPut(g_mcalCanRxMailQueueHandle, p_mailMsg); /* ISR-safe */
}
