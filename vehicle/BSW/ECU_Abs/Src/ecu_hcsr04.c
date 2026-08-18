/*
 * ecu_hcsr04.c
 *  Layer  : BSW / ECU_Abs
 *  Module : ECU_HCSR04
 *
 *  TIM4 Input Capture 기반 3채널(좌/전/우) 초음파 거리 측정.
 *  Rising/Falling Edge를 폴러리티 스위칭으로 잡아 펄스폭(us)을 계산하고,
 *  TIM4 APB1 클럭 72MHz / (Prescaler+1=72) = 1MHz -> 1 count = 1us.
 *  거리(cm) = pulse_width(us) / 58
 */
#include "ecu_hcsr04.h"
#include "cmsis_os.h"   /* ECU_HCSR04_TriggerAllSensors의 osDelay 사용 (implicit decl 방지) */

/* ------------------------------------------------------------------------
 * 내부 상태 구조체 (s_ prefix)
 * ------------------------------------------------------------------------ */
typedef struct {
    volatile uint32_t rise_tick;    /* Rising edge 캡처값 (us) */
    volatile uint32_t pulse_width;  /* 펄스 폭 (us) */
    volatile uint8_t  edge;         /* 0=Rising 대기, 1=Falling 대기 */
    volatile uint8_t  measuring;    /* 측정 진행 중 플래그 */
    volatile uint8_t  ready;        /* 측정 완료, 읽기 대기 */
} EcuHcsr04State_t;

static EcuHcsr04State_t s_usState[ECU_HCSR04_SENSOR_NUM];

/* 마지막으로 성공 측정한 거리(cm)를 센서별로 래치.
 * 제어주기(CtrlTask 50ms)가 측정주기(센서당 약 195ms)보다 빠르므로,
 * 새 측정이 아직 없을 때 '측정없음=최대거리(400cm=장애물없음)'를 돌려주면
 * 매 주기 3/4은 전방을 '비어있음'으로 오판해 충돌 위험이 있다.
 * 따라서 새 값이 없으면 마지막 유효값을 유지(hold)한다. (task 컨텍스트 전용) */
static uint32_t s_lastDist[ECU_HCSR04_SENSOR_NUM];

/* 디버그 진단용: TIM4 IC ISR이 실제로 도달하는지 Live Expressions로 확인.
 * g_hcsr04_riseCount만 늘고 fallCount가 안 늘면 ECHO가 안 돌아옴(펄스 미수신),
 * 둘 다 0이면 애초에 ISR 자체가 안 불림(배선/전원/TRIG 문제)로 구분 가능. */
volatile uint32_t g_hcsr04_riseCount[ECU_HCSR04_SENSOR_NUM] = {0};
volatile uint32_t g_hcsr04_fallCount[ECU_HCSR04_SENSOR_NUM] = {0};

static const uint32_t s_usChannel[ECU_HCSR04_SENSOR_NUM] = {
    ECU_HCSR04_CH_LEFT, ECU_HCSR04_CH_FRONT, ECU_HCSR04_CH_RIGHT
};

static const HAL_TIM_ActiveChannel s_usChannelIdx[ECU_HCSR04_SENSOR_NUM] = {
    ECU_HCSR04_CH_LEFT_IDX, ECU_HCSR04_CH_FRONT_IDX, ECU_HCSR04_CH_RIGHT_IDX
};

static const uint32_t s_usInterrupt[ECU_HCSR04_SENSOR_NUM] = {
    ECU_HCSR04_IT_LEFT, ECU_HCSR04_IT_FRONT, ECU_HCSR04_IT_RIGHT
};

static GPIO_TypeDef * const s_usTrigPort[ECU_HCSR04_SENSOR_NUM] = {
    ECU_HCSR04_TRIG_LEFT_PORT, ECU_HCSR04_TRIG_FRONT_PORT, ECU_HCSR04_TRIG_RIGHT_PORT
};

static const uint16_t s_usTrigPin[ECU_HCSR04_SENSOR_NUM] = {
    ECU_HCSR04_TRIG_LEFT_PIN, ECU_HCSR04_TRIG_FRONT_PIN, ECU_HCSR04_TRIG_RIGHT_PIN
};

/* ------------------------------------------------------------------------
 * 내부: IC 채널 폴러리티 설정 + 인터럽트 재활성화
 *   ISR 내부에서도 호출됨: HAL_TIM_IC_ConfigChannel이 채널을 잠시
 *   비활성화 후 재설정하므로, 매번 인터럽트 재-enable이 필수.
 * ------------------------------------------------------------------------ */
static void Hcsr04_SetPolarity(EcuHcsr04Index_t idx, uint32_t polarity)
{
    TIM_IC_InitTypeDef ic = {0};
    ic.ICPolarity  = polarity;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(&ECU_HCSR04_TIM_HANDLE, &ic, s_usChannel[idx]);
    __HAL_TIM_ENABLE_IT(&ECU_HCSR04_TIM_HANDLE, s_usInterrupt[idx]);
}

/* ------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------ */
void ECU_HCSR04_Init(void)
{
    for (int i = 0; i < ECU_HCSR04_SENSOR_NUM; i++)
    {
        s_usState[i].rise_tick   = 0;
        s_usState[i].pulse_width = 0;
        s_usState[i].edge        = 0;
        s_usState[i].measuring   = 0;
        s_usState[i].ready       = 0;
        s_lastDist[i]            = ECU_HCSR04_DIST_MAX_CM;

        Hcsr04_SetPolarity((EcuHcsr04Index_t)i, TIM_ICPOLARITY_RISING);
        HAL_TIM_IC_Start_IT(&ECU_HCSR04_TIM_HANDLE, s_usChannel[i]);
    }
}

/*
 * ECU_HCSR04_TriggerSensor : 특정 센서에 10us TRIG 펄스 발생
 *   순서: 상태 초기화 -> Polarity Rising 복귀 -> measuring On -> TRIG HIGH
 *   (measuring을 TRIG HIGH보다 먼저 세워야 Rising Edge 유실 방지)
 */
void ECU_HCSR04_TriggerSensor(EcuHcsr04Index_t sensor)
{
    s_usState[sensor].edge      = 0;
    s_usState[sensor].measuring = 0;
    s_usState[sensor].ready     = 0;
    s_usState[sensor].rise_tick = 0;

    Hcsr04_SetPolarity(sensor, TIM_ICPOLARITY_RISING);

    s_usState[sensor].measuring = 1;

    HAL_GPIO_WritePin(s_usTrigPort[sensor], s_usTrigPin[sensor], GPIO_PIN_SET);

    uint32_t cnt = 720; /* 72MHz 기준 약 10us NOP 대기 */
    while (cnt--) { __NOP(); }

    HAL_GPIO_WritePin(s_usTrigPort[sensor], s_usTrigPin[sensor], GPIO_PIN_RESET);
}

/*
 * ECU_HCSR04_TriggerAllSensors : 3개 센서 순차 트리거
 *   FreeRTOS Task Context에서만 호출할 것 (osDelay 사용, HAL_Delay 금지)
 */
void ECU_HCSR04_TriggerAllSensors(void)
{
    for (int i = 0; i < ECU_HCSR04_SENSOR_NUM; i++)
    {
        ECU_HCSR04_TriggerSensor((EcuHcsr04Index_t)i);
        osDelay(60); /* 센서 간 간섭 방지 (최소 50ms 권장) */
    }
}

/*
 * ECU_HCSR04_GetDistanceCm : 거리 반환 (cm)
 *   새 측정(ready=1)이 있으면 그 값으로 래치를 갱신하고,
 *   없으면 마지막 유효 거리를 유지(hold)한다.
 *   -> 제어주기(50ms)가 측정주기(약 195ms)보다 빨라도 stale 상태에서
 *      '장애물 없음(400cm)'으로 오판하지 않는다. (안전 우선)
 *   타임아웃/이상값(에코 미수신 등)일 때만 최대거리로 갱신한다.
 */
uint32_t ECU_HCSR04_GetDistanceCm(EcuHcsr04Index_t sensor)
{
    if (s_usState[sensor].ready)
    {
        s_usState[sensor].ready = 0;

        uint32_t pulse = s_usState[sensor].pulse_width;

        if (pulse == 0u || pulse > ECU_HCSR04_ECHO_TIMEOUT_US)
        {
            /* 에코 미수신/이상값 -> 실제로 전방이 비었다는 뜻이므로 최대거리로 래치 */
            s_lastDist[sensor] = ECU_HCSR04_DIST_MAX_CM;
        }
        else
        {
            s_lastDist[sensor] = pulse / 58u;
        }
    }

    return s_lastDist[sensor];
}

uint8_t ECU_HCSR04_IsReady(EcuHcsr04Index_t sensor)
{
    return s_usState[sensor].ready;
}

/*
 * ECU_HCSR04_HandleCaptureIsr : TIM4 IC 캡처 콜백
 *   main.c의 HAL_TIM_IC_CaptureCallback (USER CODE BEGIN 4) 에서
 *   반드시 위임 호출되어야 함.
 */
void ECU_HCSR04_HandleCaptureIsr(TIM_HandleTypeDef *p_htim)
{
    if (p_htim->Instance != TIM4) { return; }

    for (int i = 0; i < ECU_HCSR04_SENSOR_NUM; i++)
    {
        if (p_htim->Channel != s_usChannelIdx[i]) { continue; }
        if (!s_usState[i].measuring)              { continue; }

        uint32_t capture = HAL_TIM_ReadCapturedValue(p_htim, s_usChannel[i]);

        if (s_usState[i].edge == 0)
        {
            /* Rising edge: 시작 시각 저장 */
            s_usState[i].rise_tick = capture;
            s_usState[i].edge      = 1;
            g_hcsr04_riseCount[i]++;
            Hcsr04_SetPolarity((EcuHcsr04Index_t)i, TIM_ICPOLARITY_FALLING);
        }
        else
        {
            /* Falling edge: 펄스 폭 계산 (TIM4 16bit 오버플로우 처리 포함) */
            uint32_t rise = s_usState[i].rise_tick;
            uint32_t fall = capture;

            if (fall >= rise)
            {
                s_usState[i].pulse_width = fall - rise;
            }
            else
            {
                s_usState[i].pulse_width = (65535u - rise) + fall + 1u;
            }

            s_usState[i].rise_tick = 0;
            s_usState[i].edge      = 0;
            s_usState[i].measuring = 0;
            s_usState[i].ready     = 1;
            g_hcsr04_fallCount[i]++;

            Hcsr04_SetPolarity((EcuHcsr04Index_t)i, TIM_ICPOLARITY_RISING);
        }
        break;
    }
}
