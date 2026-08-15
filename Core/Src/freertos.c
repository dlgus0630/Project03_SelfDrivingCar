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
  for(;;)
  {
    McalCanSlaveState_t state = (RTE_Mode_GetDriveMode() == RTE_DRIVE_MODE_AUTO)
                               ? MCAL_CAN_STATE_AUTO
                               : MCAL_CAN_STATE_MANUAL;

    MCAL_CAN_BroadcastSlaveStatus(state, g_dist_left, g_dist_front, g_dist_right);

    osDelay(100);
  }
  /* USER CODE END StartCanTxTask */
}

/* USER CODE END Application */

