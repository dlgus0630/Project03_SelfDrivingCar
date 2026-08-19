/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  *
  * [수정 사항]
  * 1. AUTOSAR-lite 3계층 적용: ASW(asw_*) / RTE(rte_*) / BSW(mcal_*, ecu_*)
  * 2. CanRxTask/CanTxTask 신규 추가 (3-Node CAN 통신)
  * 3. 상태머신(수동/자동) 전환은 rte_mode_manager의 Mutex 보호 전역으로 관리
  * 4. 거리 공유값 volatile 전역 유지 (CtrlTask 갱신 -> CanTxTask 참조)
  * 5. HAL_Delay 전면 배제, osDelay로 태스크 동기화
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "rte_motor.h"
#include "rte_sensor.h"
#include "mcal_can.h"
#include "rte_mode_manager.h"
#include "asw_manual_control.h"
#include "asw_autonomous.h"
#include "ecu_hcsr04.h"   /* ECU_HCSR04_SENSOR_* / ECU_HCSR04_DIST_MAX_CM 직접 참조 (허용 예외) */
#include "mcal_bt_serial.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 공유 거리값 (volatile: CtrlTask가 갱신, CanTxTask가 참조/브로드캐스트) */
volatile uint32_t g_dist_left  = ECU_HCSR04_DIST_MAX_CM;
volatile uint32_t g_dist_front = ECU_HCSR04_DIST_MAX_CM;
volatile uint32_t g_dist_right = ECU_HCSR04_DIST_MAX_CM;
/* USER CODE END Variables */
osThreadId TrigTaskHandle;
osThreadId CtrlTaskHandle;
osThreadId CanRxTaskHandle;
osThreadId CanTxTaskHandle;
osMutexId DriveModeMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartCanRxTask(void const * argument);
void StartCanTxTask(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartTrigTask(void const * argument);
void StartCtrlTask(void const * argument);
extern void StartCanRxTask(void const * argument);
extern void StartCanTxTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* CAN / TaskManager 관련 RTOS 객체는 osKernelStart() 이전,
     각 Task 진입점 첫 줄에서 초기화하지 않고 여기서 선행 생성한다.
     (Mutex/MailQueue는 Task 생성보다 먼저 준비되어 있어야 ISR/Task 경합 없음) */
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of DriveModeMutex */
  osMutexDef(DriveModeMutex);
  DriveModeMutexHandle = osMutexCreate(osMutex(DriveModeMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of TrigTask */
  osThreadDef(TrigTask, StartTrigTask, osPriorityNormal, 0, 128);
  TrigTaskHandle = osThreadCreate(osThread(TrigTask), NULL);

  /* definition and creation of CtrlTask */
  osThreadDef(CtrlTask, StartCtrlTask, osPriorityAboveNormal, 0, 256);
  CtrlTaskHandle = osThreadCreate(osThread(CtrlTask), NULL);

  /* definition and creation of CanRxTask */
  osThreadDef(CanRxTask, StartCanRxTask, osPriorityHigh, 0, 256);
  CanRxTaskHandle = osThreadCreate(osThread(CanRxTask), NULL);

  /* definition and creation of CanTxTask */
  osThreadDef(CanTxTask, StartCanTxTask, osPriorityLow, 0, 128);
  CanTxTaskHandle = osThreadCreate(osThread(CanTxTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* CanRxTask/CanTxTask는 위 CubeMX 생성 블록에서 이미 생성되므로 여기서
   * 중복 생성하지 않는다 (osThreadDef 중복 정의로 빌드 에러 발생했었음) */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartTrigTask */
/**
* @brief 3개 초음파 센서를 순차 트리거하는 태스크.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTrigTask */
void StartTrigTask(void const * argument)
{
  /* USER CODE BEGIN StartTrigTask */
  osDelay(200); /* 시스템 안정화 대기 */

  for(;;)
  {
    /* 고정 65ms 대기 대신, 실제 에코가 돌아온 즉시(IsReady) 다음 센서로 넘어간다.
     * 가까운 벽일수록 초음파 왕복시간이 짧아지므로(예: 20cm면 약 1ms), 위험할수록
     * 오히려 갱신이 더 빨라진다. 에코가 안 돌아오는 경우(장애물 없음)에 대비해
     * 65ms를 상한으로 유지해 원래의 타임아웃 안전장치는 그대로 둔다. */
    RTE_Sensor_TriggerSensor(ECU_HCSR04_SENSOR_LEFT);
    for (uint32_t waited = 0u; waited < 65u && !RTE_Sensor_IsReady(ECU_HCSR04_SENSOR_LEFT); waited++)
    {
        osDelay(1);
    }

    RTE_Sensor_TriggerSensor(ECU_HCSR04_SENSOR_FRONT);
    for (uint32_t waited = 0u; waited < 65u && !RTE_Sensor_IsReady(ECU_HCSR04_SENSOR_FRONT); waited++)
    {
        osDelay(1);
    }

    RTE_Sensor_TriggerSensor(ECU_HCSR04_SENSOR_RIGHT);
    for (uint32_t waited = 0u; waited < 65u && !RTE_Sensor_IsReady(ECU_HCSR04_SENSOR_RIGHT); waited++)
    {
        osDelay(1);
    }
  }
  /* USER CODE END StartTrigTask */
}

/* USER CODE BEGIN Header_StartCtrlTask */
/**
* @brief 거리값 갱신 + 현재 주행모드에 따라 수동/자율 로직으로 분기 실행.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCtrlTask */
void StartCtrlTask(void const * argument)
{
  /* USER CODE BEGIN StartCtrlTask */
  static uint32_t debugLogCounter = 0; /* 4회에 1번(200ms)만 UART 로그 출력 */

  osDelay(500); /* TrigTask 첫 측정 완료까지 대기 */

  for(;;)
  {
    uint32_t left  = RTE_Sensor_GetDistance(ECU_HCSR04_SENSOR_LEFT);
    uint32_t front = RTE_Sensor_GetDistance(ECU_HCSR04_SENSOR_FRONT);
    uint32_t right = RTE_Sensor_GetDistance(ECU_HCSR04_SENSOR_RIGHT);

    /* CanTxTask가 참조할 수 있도록 공유 전역 갱신 */
    g_dist_left  = left;
    g_dist_front = front;
    g_dist_right = right;

    if ((debugLogCounter++ % 4u) == 0u)
    {
        printf("[US] L=%3lucm F=%3lucm R=%3lucm\r\n",
               (unsigned long)left, (unsigned long)front, (unsigned long)right);

        /* 블루투스 수신 진단은 printf로 찍지 않는다.
         * printf 출력이 나가는 huart1은 HC-06 블루투스와 같은 USART1이라,
         * 로그가 디버그 창이 아니라 휴대폰 앱으로 전송된다. 게다가 9600bps에서는
         * 한 줄(약 30바이트)에 30ms 넘게 블로킹 송신을 하므로 제어주기(50ms)를
         * 심하게 잡아먹는다.
         * 대신 mcal_bt_serial.c의 진단용 카운터(s_rxCount / s_errCount)를
         * STM32CubeIDE의 Live Expressions 창에서 직접 관찰한다. */
    }

    /* 휴대폰 앱 블루투스 명령(모드 전환 포함)은 현재 모드와 무관하게 항상 확인한다 -
     * 자율주행 중에도 'P'를 받으면 바로 수동 전환돼야 하기 때문 */
    ASW_Manual_ApplySerialChar(MCAL_BtSerial_GetChar());

    if (RTE_Mode_GetDriveMode() == RTE_DRIVE_MODE_MANUAL)
    {
        ASW_Manual_ProcessControl();
    }
    else
    {
        ASW_Autonomous_ProcessControl(left, front, right);
    }

    osDelay(50); /* 50ms 제어 주기 */
  }
  /* USER CODE END StartCtrlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE BEGIN Header_StartCanRxTask */
/**
* @brief CAN Mail Queue에서 수신 메시지를 꺼내 ASW_Manual_ApplyCanCommand로 즉시 라우팅.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanRxTask */
void StartCanRxTask(void const * argument)
{
  /* USER CODE BEGIN StartCanRxTask */
  McalCanMsg_t msg;

  for(;;)
  {
    /* Mail Queue가 빌 때까지 블로킹 대기 - ISR(MCAL_CAN_HandleRxFifoIsr)이
     * Put하는 즉시 이 Task가 깨어나 라우팅한다 */
    if (MCAL_CAN_Receive(&msg, osWaitForever) == osOK)
    {
        ASW_Manual_ApplyCanCommand(&msg);
    }
  }
  /* USER CODE END StartCanRxTask */
}

/* USER CODE BEGIN Header_StartCanTxTask */
/**
* @brief 100ms 주기로 현재 주행모드와 거리값을 SLAVE_STATUS(0x200)로 브로드캐스트.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTxTask */
void StartCanTxTask(void const * argument)
{
  /* USER CODE BEGIN StartCanTxTask */
  /* 버스오프 복구 재시도 간격을 제한하기 위한 주기 카운터.
   * 이 태스크는 100ms 주기이므로 10주기 = 약 1초에 한 번만 복구를 시도한다.
   *
   * 간격을 두는 이유:
   *  1) HAL_CAN_Stop()은 CAN을 초기화 모드로 넣으면서 대기 중이던 송신
   *     메일박스를 전부 취소해 버린다. 100ms마다 Stop/Start를 반복하면
   *     정상 복구된 뒤에도 송신이 계속 끊겨 오히려 통신이 더 나빠진다.
   *  2) CAN 규격상 버스오프에서 빠져나오려면 "연속 11비트 리세시브"를
   *     128번 관측해야 한다. 상대 노드가 아직 안 켜져 있으면 복구 직후
   *     곧바로 다시 버스오프로 떨어지므로, 무한정 재시도해봐야 소용없다.
   *     1초 간격이면 상대 노드가 살아나는 즉시 늦어도 1초 안에 다시 붙는다.
   *  3) 복구 시도 횟수가 1초에 1씩만 늘어나므로, Live Expressions에서
   *     g_vdiag_can_recover_cnt 값을 "버스오프였던 시간(초)"으로 바로
   *     읽을 수 있다. */
  uint32_t busOffRetryTick = 0u;

  for(;;)
  {
    uint32_t esr;

    McalCanSlaveState_t state = (RTE_Mode_GetDriveMode() == RTE_DRIVE_MODE_AUTO)
                               ? MCAL_CAN_STATE_AUTO
                               : MCAL_CAN_STATE_MANUAL;

    MCAL_CAN_BroadcastSlaveStatus(state, g_dist_left, g_dist_front, g_dist_right);

    /* ---- [진단] CAN 오류상태 레지스터(ESR) 주기 갱신 ----
     * 비트 배치는 RM0008(STM32F1 참조매뉴얼) 기준이며, 아래 CAN_ESR_*_Pos /
     * CAN_ESR_*_Msk 매크로는 CMSIS 헤더(stm32f103xb.h)가 제공하는 공식 값이다.
     *   REC = 31:24 / TEC = 23:16 / LEC = 6:4 / BOFF = 비트2 / EPVF = 비트1 /
     *   EWGF = 비트0
     * ESR은 읽기 전용 상태 레지스터이므로 여기서 읽어도 통신에 영향이 없다. */
    esr = hcan.Instance->ESR;

    g_vdiag_can_esr_raw = esr;
    g_vdiag_can_rec  = (uint8_t)((esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos);
    g_vdiag_can_tec  = (uint8_t)((esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos);
    g_vdiag_can_lec  = (uint8_t)((esr & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos);
    g_vdiag_can_boff = (uint8_t)(((esr & CAN_ESR_BOFF) != 0u) ? 1u : 0u);
    g_vdiag_can_epvf = (uint8_t)(((esr & CAN_ESR_EPVF) != 0u) ? 1u : 0u);
    g_vdiag_can_ewgf = (uint8_t)(((esr & CAN_ESR_EWGF) != 0u) ? 1u : 0u);

    /* ---- 버스오프 자동 복구 (진단용 임시 코드가 아니라 정식 기능) ----
     * 이 보드는 CubeMX 설정에서 AutoBusOff가 DISABLE(꺼짐)로 되어 있다.
     * AutoBusOff가 켜져 있으면 버스오프에 빠졌을 때 하드웨어가 알아서
     * 복구 절차를 밟지만, 꺼져 있으면 CAN 컨트롤러는 버스오프 상태에
     * 그대로 눌러앉아 영원히 한 프레임도 내보내지 않는다.
     * 즉 상대 노드가 꺼져 있는 동안 혼자 송신하다 한 번 버스오프로
     * 떨어지면, 나중에 상대 노드를 켜도 이쪽은 영영 조용한 채로 남는다.
     * 그래서 소프트웨어가 버스오프를 직접 감지해 CAN을 껐다 켜서
     * 되살려 주어야 한다. (.ioc 설정은 건드리지 않는 것이 요구사항이므로
     * 이 소프트웨어 복구 로직을 정식으로 남겨 둔다.) */
    if (g_vdiag_can_boff != 0u)
    {
      if (busOffRetryTick == 0u)
      {
        g_vdiag_can_recover_cnt++;   /* 복구 시도 횟수 기록 */

        /* Stop -> Start 로 CAN 셀을 초기화 모드에 넣었다 빼면서
         * 버스오프 복구 시퀀스를 다시 시작시킨다. */
        (void)HAL_CAN_Stop(&hcan);
        (void)HAL_CAN_Start(&hcan);
      }

      busOffRetryTick++;
      if (busOffRetryTick >= 10u)   /* 100ms * 10 = 약 1초마다 재시도 */
      {
        busOffRetryTick = 0u;
      }
    }
    else
    {
      /* 정상 상태로 돌아왔으면 다음 버스오프 때 곧바로 복구할 수 있도록
       * 재시도 카운터를 초기화해 둔다. */
      busOffRetryTick = 0u;
    }

    osDelay(100);
  }
  /* USER CODE END StartCanTxTask */
}

/* USER CODE END Application */

