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
    /* 제어 루프가 살아서 한 주기를 돌고 있음을 증명 (CanTxTask가 HEARTBEAT에 반영) */
    g_ctrl_loop_alive_counter++;

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

/* ---- 버스오프 자동 복구 (진단용 임시 코드가 아니라 정식 기능) ----
 * 이 보드는 CubeMX 설정에서 AutoBusOff가 DISABLE(꺼짐)로 되어 있다.
 * AutoBusOff가 켜져 있으면 버스오프에 빠졌을 때 하드웨어가 알아서 복구
 * 절차를 밟지만, 꺼져 있으면 CAN 컨트롤러는 버스오프 상태에 그대로
 * 눌러앉아 영원히 한 프레임도 내보내지 않는다. 즉 상대 노드가 꺼져 있는
 * 동안 혼자 송신하다 한 번 버스오프로 떨어지면, 나중에 상대 노드를 켜도
 * 이쪽은 영영 조용한 채로 남는다. 그래서 소프트웨어가 버스오프를 직접
 * 감지해 CAN을 껐다 켜서 되살려 주어야 한다.
 * (.ioc 설정은 건드리지 않는 것이 요구사항이므로 이 소프트웨어 복구
 *  로직을 정식으로 남겨 둔다.)
 *
 * 재시도는 절대 포기하지 않는다. 지금 복구가 막혀 있으면
 * g_vdiag_can_stuck을 1로 올려 두되(영구 래치가 아닌 현재 상태 플래그),
 * 다음 주기에도 계속 시도하며 복구에 성공하는 순간 다시 0으로 내린다.
 *
 * @retval 0 : 버스오프가 아니거나 이번 주기에 할 일 없음(재시도 간격 대기)
 * @retval 1 : 이번 호출에서 복구 성공
 * @retval 2 : 시도했으나 실패 (g_vdiag_can_stuck = 1)
 */
/* keep this function's logic in sync with remote/Core/Src/freertos.c's equivalent recovery helper
 * the busOffRetryTick throttle exists only because StartCanTxTask calls this every 100ms while remote's
 * StartDefaultTask already paces at ~1s via osDelay(1000); apart from that the logic must stay identical. */
static uint8_t Can_TryBusOffRecover(void)
{
  /* 버스오프 복구 재시도 간격을 제한하기 위한 주기 카운터.
   * CanTxTask가 100ms 주기로 이 함수를 호출하므로 10주기 = 약 1초에 한 번만
   * 실제 복구를 시도한다. (static이라 호출 사이에 값이 유지된다)
   *
   * 간격을 두는 이유:
   *  1) HAL_CAN_Stop()은 CAN을 초기화 모드로 넣으면서 대기 중이던 송신
   *     메일박스를 전부 취소해 버린다. 100ms마다 Stop/Start를 반복하면
   *     정상 복구된 뒤에도 송신이 계속 끊겨 오히려 통신이 더 나빠진다.
   *  2) CAN 규격상 버스오프에서 빠져나오려면 "연속 11비트 리세시브"를
   *     128번 관측해야 한다. 상대 노드가 아직 안 켜져 있으면 복구 직후
   *     곧바로 다시 버스오프로 떨어지므로, 더 자주 시도해봐야 소용없다.
   *     1초 간격이면 상대 노드가 살아나는 즉시 늦어도 1초 안에 다시 붙는다.
   *  3) 복구 성공이 1초에 최대 1번만 집계되므로, Live Expressions에서
   *     g_vdiag_can_recover_cnt를 "지금까지 되살아난 횟수"로 읽을 수 있다.
   *     (버스가 붙었다 끊겼다를 반복하는 동안에는 초당 1씩 늘어난다.
   *      반대로 복구가 계속 실패하는 동안에는 값이 멈춰 있으므로, 이 값을
   *      '버스오프였던 시간'으로 읽어서는 안 된다.) */
  static uint32_t busOffRetryTick = 0u;

  HAL_StatusTypeDef stopStatus;
  HAL_StatusTypeDef startStatus;
  HAL_StatusTypeDef notifyStatus;
  uint8_t attemptNow;

  if (g_vdiag_can_boff == 0u)
  {
    /* 정상 상태로 돌아왔으면 다음 버스오프 때 곧바로 복구할 수 있도록
     * 재시도 카운터를 초기화해 둔다. */
    busOffRetryTick = 0u;
    return 0u;
  }

  attemptNow = (busOffRetryTick == 0u) ? 1u : 0u;

  busOffRetryTick++;
  if (busOffRetryTick >= 10u)   /* 100ms * 10 = 약 1초마다 재시도 */
  {
    busOffRetryTick = 0u;
  }

  if (attemptNow == 0u)
  {
    return 0u;   /* 재시도 간격에 걸림: 이번 주기는 아무것도 건드리지 않는다 */
  }

  /* 상태 조건을 "먼저" 확인한다. 실제로 시도할 수 있는 전이일 때만 수신
   * 인터럽트를 껐다 켠다. HAL_CAN_Stop()은 State가 LISTENING일 때만 성공하고,
   * 다른 상태(특히 이전 시도가 타임아웃되어 굳어버린 HAL_CAN_STATE_ERROR)에서는
   * 내부 가드에 막혀 아무 일도 하지 않고 HAL_ERROR만 돌려준다. 그런 상황에서
   * 인터럽트만 껐다 켜는 것은 얻는 것 없이 수신만 흔드는 짓이므로,
   * 시도조차 못 하는 경우에는 인터럽트를 전혀 건드리지 않고 바로 빠져나온다. */
  if (hcan.State != HAL_CAN_STATE_LISTENING)
  {
    /* 인터럽트는 위 설명대로 전혀 건드리지 않지만, 진단 플래그는 갱신해 둔다.
     * 이 함수의 실패가 아니라 바깥의 다른 원인으로 CAN 상태가 망가진 경우,
     * 플래그는 마지막에 기록된 값(대개 "정상")에 영원히 멈춰 있게 되고
     * 그러면 HEARTBEAT bit3이 계속 거짓 정보를 내보낸다. State가 LISTENING
     * 조차 아니라면 수신이 정상 상태라고 볼 근거가 없으므로 0으로 내린다. */
    g_vdiag_can_stuck = 1u;
    g_vdiag_can_notify_ok = 0u;
    return 2u;
  }

  /* Stop/Start 전에 수신 인터럽트를 먼저 꺼 둔다.
   * 이 과정에서 CAN 셀은 초기화 모드로 들어갔다 나오는데, 그 도중에
   * HAL_CAN_RxFifo0MsgPendingCallback()이 끼어들면 재설정 중인 같은 hcan
   * 핸들을 동시에 건드리게 된다.
   * HAL_CAN_Stop()은 인터럽트 설정을 건드리지 않으므로 직접 꺼야 한다. */
  (void)HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* 반환값을 따지지 않고 무조건 플래그를 내린다. "수신 인터럽트를 꺼 달라"고
   * 요청한 순간부터는 그 호출이 성공했든 실패했든 수신이 정상 상태라고 볼 수
   * 없기 때문이다. 어차피 함수 끝에서 ActivateNotification 결과로 다시 세팅된다. */
  g_vdiag_can_notify_ok = 0u;

  /* Stop -> Start 로 CAN 셀을 초기화 모드에 넣었다 빼면서 버스오프 복구
   * 시퀀스를 다시 시작시킨다. */
  stopStatus = HAL_CAN_Stop(&hcan);

  /* HAL_CAN_Start()는 State가 READY일 때만 성공한다.
   * HAL_CAN_Stop()은 성공 경로에서 반드시 State를 HAL_CAN_STATE_READY로 내려놓고
   * HAL_OK를 반환하므로(벤더 HAL 소스 확인), stopStatus == HAL_OK 하나만으로
   * READY가 보장된다. 예전에 함께 보던 (hcan.State == HAL_CAN_STATE_READY)는
   * 항상 참인 중복 조건이라 제거했다. */
  if (stopStatus == HAL_OK)
  {
    startStatus = HAL_CAN_Start(&hcan);
  }
  else
  {
    startStatus = HAL_ERROR;
  }

  /* 성공/실패 어느 쪽이든, 우리가 껐던 수신 인터럽트는 반드시 다시 켜서
   * 영구히 꺼진 채로 남지 않게 한다. 셀이 HAL_CAN_STATE_ERROR로 굳었다면
   * 이 호출도 같은 내부 가드에 막혀 HAL_ERROR를 돌려주는데, 그 결과를
   * 버리지 않고 진단 플래그에 그대로 반영해 둔다. */
  notifyStatus = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  g_vdiag_can_notify_ok = (uint8_t)((notifyStatus == HAL_OK) ? 1u : 0u);

  if ((stopStatus == HAL_OK) && (startStatus == HAL_OK))
  {
    /* 시퀀스 전체가 성공했을 때만 복구로 인정하고 센다. */
    g_vdiag_can_stuck = 0u;
    g_vdiag_can_recover_cnt++;
    return 1u;
  }

  /* 이번 시도는 실패. 다음 주기에도 계속 재시도한다(포기하지 않는다). */
  g_vdiag_can_stuck = 1u;
  return 2u;
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
  /* HEARTBEAT(0x3F0) 송신 주기 카운터.
   * 이 태스크는 100ms 주기이므로 10주기 = 약 1초에 한 번만 생존 신호를 보낸다.
   * 상태(0x200)는 100ms마다 나가지만, 생존 신호까지 같은 주기로 내보내면
   * 송신 메일박스(3개)와 버스 대역만 낭비된다. 마스터 입장에서 "이 노드가
   * 살아 있는가"는 1초 해상도면 충분하다. */
  uint32_t heartbeatTick = 0u;

  /* 직전 HEARTBEAT 때 읽어 둔 제어 루프 생존 카운터 값.
   * 이 태스크는 절대 반환하지 않으므로 지역변수만으로도 주기 사이에 값이
   * 유지된다(static 불필요). 다음 HEARTBEAT 때 값이 그대로면 CtrlTask가
   * 멈춘 것으로 판정한다. */
  uint32_t lastCtrlAliveSnapshot = 0u;

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

    /* ---- 버스오프 자동 복구 ----
     * 실제 복구 로직과 재시도 간격 제한은 Can_TryBusOffRecover()가 전담한다.
     * 결과는 g_vdiag_can_stuck / g_vdiag_can_recover_cnt 전역에 반영되고,
     * 바로 아래 HEARTBEAT가 그 값을 그대로 실어 보내므로 반환값은 쓰지 않는다. */
    (void)Can_TryBusOffRecover();

    /* ---- HEARTBEAT(0x3F0) 약 1초 주기 송신 ----
     * DLC 1바이트짜리 최소 프레임으로 "이 노드가 아직 제대로 돌고 있다"를 알린다.
     * payload[0]은 비트필드이며 각 비트의 의미는 아래와 같다 (0 = 전부 정상).
     *   bit0 : 버스오프 복구가 막혀 있음        (g_vdiag_can_stuck)
     *   bit1 : CAN 오류 수동(Error Passive)     (g_vdiag_can_epvf)
     *   bit2 : CAN 오류 경고(Error Warning)     (g_vdiag_can_ewgf)
     *   bit3 : CAN 수신 인터럽트가 꺼져 있음    (g_vdiag_can_notify_ok == 0)
     *   bit4 : 제어 루프(CtrlTask)가 멈춤
     *   bit5~7 : 예약(항상 0)
     *
     * bit4가 핵심이다. 이 태스크(CanTxTask)는 우선순위가 가장 낮아서, 이 프레임이
     * 나갔다는 사실만으로는 "CanTxTask가 스케줄됐다"는 것밖에 증명하지 못한다.
     * 실제로 센서를 읽고 위험하면 모터를 세우는 것은 50ms 주기의 CtrlTask이므로,
     * CtrlTask가 매 주기 올리는 생존 카운터가 지난 HEARTBEAT 이후로 늘었는지를
     * 함께 확인한다. 카운터가 그대로면 제어 루프가 멈춘 것이고, 그때 모터는
     * 마지막 명령 듀티로 계속 돌고 있으므로 마스터가 반드시 알아야 한다.
     *
     * 위 버스오프 복구 블록이 끝난 뒤에 보내는 이유는, 이번 주기에 복구를
     * 시도하다 실패해 g_vdiag_can_stuck이 막 1이 된 경우까지 같은 주기 안에서
     * 곧바로 반영해 내보내기 위해서다.
     *
     * 물론 CAN 셀이 정말 죽었다면 이 프레임 자체가 버스로 나가지 못한다.
     * 그때는 마스터가 "비트가 1이라고 알려오는 것"이 아니라 "생존 신호가 끊긴 것"
     * 으로 고장을 판정하게 된다. 이 비트필드는 셀이 아직 송신은 되는 상태에서
     * 어디가 어떻게 나쁜지를 구분해 주는 용도다. */
    heartbeatTick++;
    if (heartbeatTick >= 10u)   /* 100ms * 10 = 약 1초 */
    {
      uint32_t ctrlAliveNow = g_ctrl_loop_alive_counter;
      uint8_t  healthBits   = 0u;

      if (g_vdiag_can_stuck != 0u)
      {
        healthBits |= 0x01u;
      }
      if (g_vdiag_can_epvf != 0u)
      {
        healthBits |= 0x02u;
      }
      if (g_vdiag_can_ewgf != 0u)
      {
        healthBits |= 0x04u;
      }
      if (g_vdiag_can_notify_ok == 0u)
      {
        healthBits |= 0x08u;
      }
      if (ctrlAliveNow == lastCtrlAliveSnapshot)
      {
        /* 지난 HEARTBEAT 이후 제어 루프가 한 주기도 못 돌았다는 뜻.
         * (부팅 직후에도 CtrlTask가 첫 HEARTBEAT보다 먼저 돌기 시작하므로
         *  카운터는 이미 0이 아니다. 만에 하나 아직 0이라면 그것 역시
         *  "제어 루프가 아직 안 돌고 있다"는 사실 그대로의 보고다.) */
        healthBits |= 0x10u;
      }

      lastCtrlAliveSnapshot = ctrlAliveNow;

      MCAL_CAN_BroadcastHeartbeat(healthBits);

      heartbeatTick = 0u;
    }

    osDelay(100);
  }
  /* USER CODE END StartCanTxTask */
}

/* USER CODE END Application */

