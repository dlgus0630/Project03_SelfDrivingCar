/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "i2c.h"
/* CAN 손잡이(hcan)와 CAN 관련 HAL 함수들을 쓰기 위해 필요하다 */
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* I2C 버스를 훑어볼 주소 범위 (7비트 주소 체계에서 쓸 수 있는 전 구간) */
#define I2C_SCAN_ADDR_MIN        1u
#define I2C_SCAN_ADDR_MAX        127u

/* 9축 IMU 센서의 주소와, 어떤 칩인지 알려주는 레지스터 번호 */
#define MPU_ADDR_7BIT            0x68u
#define MPU_REG_WHO_AM_I         0x75u

/* 같은 모듈에 함께 붙어 나오는 경우가 많은 기압 센서 주소 */
#define BMP280_ADDR_7BIT         0x76u

/* CAN 프레임을 화면에 찍는 최소 간격(밀리초).
   상대 노드가 100ms 마다 보내므로 전부 찍으면 화면이 순식간에 넘쳐서
   정작 봐야 할 내용을 놓친다. 500ms 간격 = 초당 2개만 찍는다. */
#define CAN_PRINT_INTERVAL_MS    500u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId CanRxTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* 모든 ID 를 통과시키는 수신 필터를 설정한다.
   루프백 시험 전과 정상 모드 복귀 때 두 번 쓰이므로 함수로 묶어 두었다.
   반환값 1 = 성공, 0 = 실패 */
static uint8_t Can_SetAcceptAllFilter(void);

/* CAN 컨트롤러 자체가 멀쩡한지 바깥 배선과 무관하게 확인하는 자기진단.
   결과는 g_diag_can_loopback 과 g_diag_can_restore_ok 에 담긴다. */
static void Can_LoopbackSelfTest(void);

/* CAN_ESR(오류 상태 레지스터)를 읽어 진단 변수들을 갱신한다. */
static void Can_UpdateErrorDiag(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartCanRxTask(void const * argument);

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
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of CanRxTask */
  osThreadDef(CanRxTask, StartCanRxTask, osPriorityIdle, 0, 256);
  CanRxTaskHandle = osThreadCreate(osThread(CanRxTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint8_t      addr;
  uint8_t      found_count = 0u;   /* 응답한 장치 개수 */
  uint8_t      mpu_found   = 0u;   /* 0x68 이 응답했는지 여부 */
  uint8_t      who_am_i    = 0u;
  const char  *chip_name;
  uint32_t     total;
  uint32_t     prev_total  = 0u;   /* 1초 전의 총 수신 개수 */

  /* ================= CAN 루프백 자기진단 (부팅 후 딱 한 번만) =================
     [왜 여기인가]
       1) 이 자리는 FreeRTOS 스케줄러가 이미 돌기 시작한 뒤라서
          osDelay() 를 쓸 수 있다. main() 안이었다면 스케줄러가 아직 안 돌아
          osDelay() 가 먹히지 않아 HAL_Delay() 를 써야 했을 것이다.
       2) I2C 스캔보다 앞에 두었다. 스캔은 127개 주소를 하나씩 두드리느라
          1초 넘게 걸리는데, 그 뒤로 미루면 진단 결과가 그만큼 늦게 나온다.
       3) 루프백 시험을 하는 동안에는 바깥에서 들어오는 진짜 프레임을 받지 못한다.
          그러니 이 "못 받는 구간"은 부팅 직후로 최대한 당겨서 짧게 끝내는 편이 낫다. */
  Can_LoopbackSelfTest();

  /* ================= I2C 버스 스캔 (부팅 후 딱 한 번만) ================= */
  printf("[I2C] 버스 스캔을 시작합니다 (주소 0x01 ~ 0x7F)\r\n");

  for (addr = I2C_SCAN_ADDR_MIN; addr <= I2C_SCAN_ADDR_MAX; addr++)
  {
    /* HAL 함수는 8비트 형태의 주소를 받는다.
       7비트 주소의 오른쪽 한 칸은 읽기/쓰기 구분용 자리이므로,
       7비트 주소를 왼쪽으로 한 칸 밀어서(addr << 1) 넘겨야 한다.
       이걸 빼먹으면 실제로는 절반 주소로 두드리게 되어 아무것도 못 찾는다. */
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 2, 10) == HAL_OK)
    {
      found_count++;

      /* 디버거(Live Expressions)로 볼 진단 변수에도 같은 내용을 담아 둔다.
         주소 목록 배열은 8칸뿐이므로, 그 안에 들어갈 때만 기록한다(경계 검사).
         9개째부터는 개수만 늘어나고 목록에는 담기지 않는다. */
      if (found_count <= 8u)
      {
        g_diag_i2c_addr[found_count - 1u] = addr;
      }
      g_diag_i2c_found = found_count;

      if (addr == MPU_ADDR_7BIT)
      {
        mpu_found = 1u;
        g_diag_imu_addr = MPU_ADDR_7BIT;   /* IMU 를 이 주소에서 찾았다 */
        printf("[I2C]   0x%02X 응답  <- 9축 IMU 로 보입니다\r\n", (unsigned int)addr);
      }
      else if (addr == BMP280_ADDR_7BIT)
      {
        printf("[I2C]   0x%02X 응답  <- 기압 센서(BMP280) 로 보입니다\r\n", (unsigned int)addr);
      }
      else
      {
        printf("[I2C]   0x%02X 응답\r\n", (unsigned int)addr);
      }
    }
  }

  printf("[I2C] 스캔 완료: 모두 %u개 장치가 응답했습니다\r\n", (unsigned int)found_count);

  if (found_count == 0u)
  {
    /* 하나도 못 찾았을 때는 어디를 봐야 하는지 순서대로 알려준다 */
    printf("[I2C] 응답한 장치가 하나도 없습니다. 아래를 확인해 주세요.\r\n");
    printf("[I2C]   1) SCL 은 PB6, SDA 는 PB7 입니다. 두 선이 서로 바뀌지 않았는지 보세요.\r\n");
    printf("[I2C]   2) 센서 모듈의 VCC(3.3V) 와 GND 가 제대로 꽂혀 있는지 보세요.\r\n");
    printf("[I2C]   3) MPU-9250 모듈의 NCS 핀은 I2C 로 쓸 때 3.3V 에 연결해야 합니다.\r\n");
    printf("[I2C]      (NCS 가 GND 나 빈 상태면 SPI 모드로 잡혀 I2C 로는 응답하지 않습니다)\r\n");
    printf("[I2C]   4) AD0 핀이 GND 면 주소가 0x68, 3.3V 면 0x69 가 됩니다.\r\n");
    printf("[I2C]   5) 신호선을 3.3V 로 끌어올리는 풀업 저항이 있는지 보세요.\r\n");
  }
  else if (mpu_found != 0u)
  {
    /* 0x68 이 응답했으니 어떤 칩인지 WHO_AM_I 레지스터를 읽어 확인한다.
       이 레지스터에는 칩마다 고유한 값이 들어 있어 모델 구분이 가능하다. */
    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(MPU_ADDR_7BIT << 1),
                         MPU_REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                         &who_am_i, 1, 100) == HAL_OK)
    {
      /* 읽은 값 그대로를 디버거로 볼 수 있게 남긴다 */
      g_diag_who_am_i = who_am_i;

      /* 화면에 찍을 이름과, 디버거로 볼 판별 코드를 함께 정한다.
         (판별 코드의 의미는 main.h / main.c 의 선언부에 적어 두었다) */
      switch (who_am_i)
      {
        case 0x71u: chip_name = "MPU-9250 (9축, 나침반 있음)";
                    g_diag_imu_kind = 1u;   /* 1 = MPU-9250 */
                    break;
        case 0x73u: chip_name = "MPU-9255 (9축, 나침반 있음)";
                    g_diag_imu_kind = 2u;   /* 2 = MPU-9255 */
                    break;
        case 0x70u: chip_name = "MPU-6500 (6축, 나침반 없음)";
                    g_diag_imu_kind = 3u;   /* 3 = MPU-6500 */
                    break;
        case 0x68u: chip_name = "MPU-6050 (6축, 나침반 없음)";
                    g_diag_imu_kind = 4u;   /* 4 = MPU-6050 */
                    break;
        default:    chip_name = "알 수 없는 칩";
                    g_diag_imu_kind = 9u;   /* 9 = 알 수 없는 칩 */
                    break;
      }
      printf("[IMU] WHO_AM_I(0x75) = 0x%02X -> %s\r\n",
             (unsigned int)who_am_i, chip_name);
    }
    else
    {
      /* 읽기에 실패했다는 사실 자체를 디버거로 구분할 수 있게 표시한다.
         0xFF 는 "읽지 못했다"는 뜻으로 쓰는 값이다. */
      g_diag_who_am_i = 0xFFu;
      g_diag_imu_kind = 0u;   /* 0 = 못 찾음 */
      printf("[IMU] 0x68 은 응답했지만 WHO_AM_I 레지스터를 읽지 못했습니다\r\n");
    }
  }
  else
  {
    printf("[IMU] 0x68 주소에서 응답이 없습니다. AD0 핀이 3.3V 라면 주소는 0x69 입니다\r\n");
  }

  /* 여기까지 왔다면 I2C 스캔과 칩 판별이 모두 끝난 것이다.
     디버거에서 이 값이 1 이 되어야 위의 진단 변수들을 믿고 읽을 수 있다.
     계속 0 이라면 스캔 도중에 멈춰 있다는 뜻이다. */
  g_diag_scan_done = 1u;

  printf("----------------------------------------\r\n");
  printf("[알림] 이제부터 1초마다 CAN 수신 현황을 알려드립니다\r\n");

  /* ================= 1초마다 CAN 수신 현황 요약 ================= */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);

    /* ---------- CAN 오류 상태를 매초 새로 읽어 둔다 ---------- */
    Can_UpdateErrorDiag();

    /* ---------- 버스오프 자동 복구 ----------
       이 보드는 AutoBusOff 가 꺼져 있어서(CubeMX 에서 AutoBusOff = DISABLE),
       버스오프에 빠져도 하드웨어가 알아서 빠져나오지 않는다.
       그래서 소프트웨어가 직접 "초기화 요청"을 걸었다가 풀어 주어야 한다.
       HAL_CAN_Stop() 이 초기화 요청을 걸고 HAL_CAN_Start() 가 그것을 푸는데,
       이 과정을 거쳐야 컨트롤러가 버스가 조용해지기를 기다렸다가 다시 합류한다. */
    if (g_diag_can_boff != 0u)
    {
      g_diag_can_recover_cnt++;
      printf("[CAN] 버스오프 감지 - 복구를 시도합니다 (누적 %lu회)\r\n",
             (unsigned long)g_diag_can_recover_cnt);

      if (HAL_CAN_Stop(&hcan) != HAL_OK)
      {
        g_diag_can_started = 0u;
      }
      else if (HAL_CAN_Start(&hcan) != HAL_OK)
      {
        g_diag_can_started = 0u;
      }
      else
      {
        g_diag_can_started = 1u;
        /* Stop/Start 로는 수신 인터럽트 설정이 지워지지 않지만,
           복구한 뒤에는 확실히 켜져 있어야 하므로 한 번 더 켜 둔다.
           이미 켜져 있는 것을 또 켜는 것이라 해가 없다. */
        if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK)
        {
          g_diag_can_notify_ok = 1u;
        }
        else
        {
          g_diag_can_notify_ok = 0u;
        }
      }
    }

    total = CanRx_GetTotal();

    /* 직전 1초 동안 늘어난 개수를 디버거로 볼 수 있게 담아 둔다.
       이 값이 0 이면 상대 노드가 안 보내고 있거나 배선 문제이고,
       정상이라면 상대가 100ms 주기로 보내므로 10 안팎이 나와야 한다. */
    g_diag_can_per_sec = total - prev_total;

    /* 총 개수와, 직전 1초 동안 늘어난 개수를 같이 보여준다.
       늘어난 개수가 0 이면 상대 노드가 안 보내고 있거나 배선 문제다.
       정상이라면 100ms 주기이므로 1초에 약 10개씩 늘어야 한다. */
    printf("[CAN 요약] 총 %lu개 수신 (최근 1초 +%lu개), 버퍼가 꽉 차 버린 것 %lu개\r\n",
           (unsigned long)total,
           (unsigned long)(total - prev_total),
           (unsigned long)CanRx_GetOverflow());

    prev_total = total;
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartCanRxTask */
/**
* @brief Function implementing the CanRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanRxTask */
void StartCanRxTask(void const * argument)
{
  /* USER CODE BEGIN StartCanRxTask */
  CanRxFrame_t frame;
  uint8_t      i;
  uint32_t     now;
  uint32_t     last_print_tick;

  /* [참고] 이 태스크는 지금 우선순위가 osPriorityIdle(-3) 로 잡혀 있다.
     이는 시스템에서 가장 낮은 등급이라, 다른 태스크가 바쁘면
     이 태스크는 계속 뒤로 밀려 CAN 출력이 끊기거나 늦게 나올 수 있다.
     지금은 진단용이고 출력이 조금 늦어도 상관없어서 그대로 두지만,
     나중에 실제 주행 제어 로직을 이 태스크에 붙일 때에는
     CubeMX 에서 osPriorityNormal 이상으로 반드시 올려주는 것이 좋다.
     (우선순위 설정은 CubeMX 가 만드는 자리라 코드에서 고치지 않았다) */

  /* 처음 들어온 프레임은 기다리지 않고 바로 찍히도록 시작값을 맞춰 둔다 */
  last_print_tick = osKernelSysTick() - CAN_PRINT_INTERVAL_MS;

  /* Infinite loop */
  for(;;)
  {
    /* 버퍼에 쌓인 프레임을 남김없이 꺼내 비운다.
       꺼내지 않고 두면 버퍼가 금방 가득 차서 새 프레임을 놓치게 된다. */
    while (CanRx_Pop(&frame) != 0u)
    {
      now = osKernelSysTick();

      /* 출력 속도 제한.
         상대가 100ms 마다 보내므로 받은 것을 전부 찍으면 화면이 넘쳐
         정작 봐야 할 내용을 놓친다. 그래서 500ms 에 한 개만 찍고
         나머지는 화면에 안 찍고 버린다.
         단, 몇 개를 받았는지 세는 일은 수신 인터럽트에서 계속 하고 있으므로
         여기서 안 찍어도 개수는 하나도 빠지지 않는다.
         (115200bps 라 여유는 있지만 그래도 제한을 두는 편이 안전하다) */
      if ((uint32_t)(now - last_print_tick) >= CAN_PRINT_INTERVAL_MS)
      {
        last_print_tick = now;

        if (frame.IsExt != 0u)
        {
          printf("[CAN] ID=0x%08lX(확장) DLC=%u data=",
                 (unsigned long)frame.Id, (unsigned int)frame.Dlc);
        }
        else
        {
          printf("[CAN] ID=0x%03lX DLC=%u data=",
                 (unsigned long)frame.Id, (unsigned int)frame.Dlc);
        }

        for (i = 0u; i < frame.Dlc; i++)
        {
          printf("%02X", (unsigned int)frame.Data[i]);
          if ((uint8_t)(i + 1u) < frame.Dlc)
          {
            printf(" ");   /* 바이트 사이만 띄우고 줄 끝에는 공백을 남기지 않는다 */
          }
        }
        printf("\r\n");
      }
    }

    /* 버퍼가 비었으면 잠깐 쉰다.
       FreeRTOS 환경이므로 HAL_Delay() 가 아니라 osDelay() 를 써야 한다.
       osDelay() 는 쉬는 동안 다른 태스크에 자리를 넘겨주지만,
       HAL_Delay() 는 자리를 넘기지 않고 그 자리에서 계속 붙잡고 있는다. */
    osDelay(10);
  }
  /* USER CODE END StartCanRxTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief  모든 ID 를 통과시키는 수신 필터를 설정한다.
  *         마스크를 0 으로 두면 ID 를 비교하는 자리가 하나도 없어져
  *         결과적으로 어떤 ID 든 전부 통과한다.
  *         지금은 원인을 찾는 것이 목적이라, 버스에 실제로 어떤 ID 가
  *         흘러다니는지 하나도 빠짐없이 봐야 하므로 일부러 전부 통과시킨다.
  * @retval 1 = 성공, 0 = 실패
  */
static uint8_t Can_SetAcceptAllFilter(void)
{
  CAN_FilterTypeDef can_filter = {0};

  can_filter.FilterBank           = 0;
  can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh         = 0x0000;
  can_filter.FilterIdLow          = 0x0000;
  can_filter.FilterMaskIdHigh     = 0x0000;   /* 마스크 0 = 전부 통과 */
  can_filter.FilterMaskIdLow      = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterActivation     = ENABLE;
  can_filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK)
  {
    g_diag_can_filter_ok = 0u;
    return 0u;
  }

  g_diag_can_filter_ok = 1u;
  return 1u;
}

/**
  * @brief  CAN_ESR(오류 상태 레지스터)를 읽어 진단 변수들을 갱신한다.
  *         이 레지스터를 통째로 읽어오는 HAL 함수는 따로 없어서
  *         hcan.Instance->ESR 로 직접 읽는다.
  */
static void Can_UpdateErrorDiag(void)
{
  uint32_t esr;

  esr = hcan.Instance->ESR;
  g_diag_can_esr_raw = esr;

  /* RM0008(STM32F1 참조매뉴얼)의 CAN_ESR 비트 배치대로 잘라낸다.
       비트 0     = EWGF (오류 경고)
       비트 1     = EPVF (오류 수동)
       비트 2     = BOFF (버스오프)
       비트 6~4   = LEC  (마지막 오류 코드)
       비트 23~16 = TEC  (송신 오류 카운터)
       비트 31~24 = REC  (수신 오류 카운터)
     [주의] TEC 가 아래쪽(16비트), REC 가 위쪽(24비트)이다. 순서를 바꿔 쓰면
     "보내다 난 오류"와 "받다가 난 오류"를 통째로 뒤바꿔 읽게 되어
     원인을 정반대로 짚게 되므로 반드시 이대로 두어야 한다. */
  g_diag_can_ewgf = (uint8_t)( esr        & 0x1u);
  g_diag_can_epvf = (uint8_t)((esr >> 1)  & 0x1u);
  g_diag_can_boff = (uint8_t)((esr >> 2)  & 0x1u);
  g_diag_can_lec  = (uint8_t)((esr >> 4)  & 0x7u);
  g_diag_can_tec  = (uint8_t)((esr >> 16) & 0xFFu);
  g_diag_can_rec  = (uint8_t)((esr >> 24) & 0xFFu);

  /* HAL 계층이 따로 모아 둔 오류값도 같이 남겨 둔다 */
  g_diag_can_hal_err = HAL_CAN_GetError(&hcan);
}

/**
  * @brief  CAN 컨트롤러 자체가 멀쩡한지 확인하는 루프백 자기진단.
  *
  *         루프백 모드는 내보낸 신호를 칩 안에서 곧바로 자기 수신부로
  *         되돌려 넣는 시험 모드다. 이때 수신 핀(PA11)에서 들어오는 실제 신호는
  *         아예 무시하므로, 트랜시버가 안 붙어 있든 배선이 끊겼든 상관이 없다.
  *         그래서 이 시험이 성공하면 클럭·비트타이밍(통신 속도)·필터·인터럽트 같은
  *         MCU 안쪽 설정이 전부 정상이라는 뜻이 되고,
  *         남은 원인은 바깥(트랜시버·배선·종단저항·상대 노드)뿐으로 좁혀진다.
  */
static void Can_LoopbackSelfTest(void)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  CAN_RxHeaderTypeDef rx_header = {0};
  uint8_t             tx_data[2];
  uint8_t             rx_data[8] = {0u};
  uint32_t            tx_mailbox = 0u;
  uint32_t            waited_ms  = 0u;
  uint8_t             got_frame  = 0u;

  tx_data[0] = 0xABu;
  tx_data[1] = 0xCDu;

  printf("[CAN 자가진단] 루프백 시험을 시작합니다 (바깥 배선과 무관한 내부 시험)\r\n");

  /* ---------- 0단계 : 수신 인터럽트를 잠시 꺼 둔다 ----------
     [이 단계가 없으면 시험이 통째로 헛돈다]
     수신 인터럽트가 켜져 있으면, 되돌아온 시험용 프레임을 인터럽트가 먼저
     낚아채서 수신함(FIFO)을 비워 버린다. 그러면 아래에서 아무리 수신함을
     들여다봐도 늘 비어 있어서, 멀쩡한데도 "수신 타임아웃(3)"이라는
     엉뚱한 결과가 나온다.
     그리고 인터럽트가 가져가 버리면 시험용 프레임이 실제 수신 개수
     (g_diag_can_total)에 섞여 들어가 통계까지 더럽힌다.
     HAL_CAN_Stop() 이나 HAL_CAN_Init() 은 인터럽트 설정을 건드리지 않으므로,
     이렇게 직접 꺼 주어야 한다. */
  (void)HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  g_diag_can_notify_ok = 0u;

  /* ---------- 1단계 : 루프백 모드로 바꾼다 ----------
     MX_CAN_Init() 이 이미 정상 모드로 초기화해 둔 상태이므로,
     일단 멈춘 뒤 모드만 바꿔서 다시 초기화한다.
     여기서의 멈춤은 실패해도 그냥 넘어간다. 부팅 때 HAL_CAN_Start() 가
     실패했다면 아직 "멈출 것이 없는" 상태라 오류를 돌려주는데,
     그래도 다시 초기화하는 데에는 지장이 없기 때문이다. */
  (void)HAL_CAN_Stop(&hcan);
  g_diag_can_started = 0u;

  hcan.Init.Mode = CAN_MODE_LOOPBACK;

  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    /* 아직 프레임을 내보내는 데까지 가지도 못했으므로 송신 실패로 묶는다 */
    g_diag_can_loopback = 2u;
    printf("[CAN 자가진단] 결과 2 : 루프백 모드 초기화에 실패했습니다\r\n");
  }
  else if (Can_SetAcceptAllFilter() == 0u)
  {
    g_diag_can_loopback = 2u;
    printf("[CAN 자가진단] 결과 2 : 루프백용 필터 설정에 실패했습니다\r\n");
  }
  else if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    g_diag_can_loopback = 2u;
    printf("[CAN 자가진단] 결과 2 : 루프백 모드 시작에 실패했습니다\r\n");
  }
  else
  {
    /* ---------- 2단계 : 시험용 프레임을 하나 내보낸다 ---------- */
    tx_header.StdId              = 0x7FFu;   /* 실제 통신에 안 쓰는 값으로 골랐다 */
    tx_header.ExtId              = 0x0000u;
    tx_header.IDE                = CAN_ID_STD;
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.DLC                = 2u;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox) != HAL_OK)
    {
      g_diag_can_loopback = 2u;
      printf("[CAN 자가진단] 결과 2 : 송신함에 프레임을 넣지 못했습니다\r\n");
    }
    else
    {
      /* ---------- 3단계 : 최대 100ms 동안 되돌아오기를 기다린다 ----------
         이 함수는 FreeRTOS 스케줄러가 이미 돌고 있는 태스크 안에서 불리므로
         HAL_Delay() 가 아니라 osDelay() 를 써야 한다.
         HAL_Delay() 는 기다리는 동안 자리를 안 내주고 붙잡고 있어서
         다른 태스크가 전부 멈춰 버린다. */
      while (waited_ms < 100u)
      {
        if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0u)
        {
          got_frame = 1u;
          break;
        }
        osDelay(1);
        waited_ms++;
      }

      if (got_frame == 0u)
      {
        g_diag_can_loopback = 3u;
        printf("[CAN 자가진단] 결과 3 : 100ms 안에 되돌아오지 않았습니다\r\n");
      }
      else if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0,
                                    &rx_header, rx_data) != HAL_OK)
      {
        g_diag_can_loopback = 3u;
        printf("[CAN 자가진단] 결과 3 : 되돌아온 프레임을 꺼내지 못했습니다\r\n");
      }
      /* ---------- 4단계 : 보낸 것과 같은 내용인지 대조한다 ---------- */
      else if ((rx_header.IDE   != CAN_ID_STD) ||
               (rx_header.StdId != 0x7FFu)     ||
               (rx_header.DLC   != 2u)         ||
               (rx_data[0]      != 0xABu)      ||
               (rx_data[1]      != 0xCDu))
      {
        g_diag_can_loopback = 4u;
        printf("[CAN 자가진단] 결과 4 : 되돌아온 내용이 보낸 것과 다릅니다\r\n");
      }
      else
      {
        g_diag_can_loopback = 1u;
        printf("[CAN 자가진단] 결과 1 : 성공 - MCU 안쪽 CAN 설정은 정상입니다\r\n");
      }
    }
  }

  /* ---------- 5단계 : 반드시 정상 모드로 되돌린다 ----------
     [가장 중요한 단계]
     이 되돌리기가 한 군데라도 실패하면 이후 실제 통신이 영영 되지 않는다.
     그래서 단계마다 결과를 하나하나 확인하고,
     전부 성공했을 때에만 g_diag_can_restore_ok 를 1 로 둔다.
     위쪽 시험이 어떤 결과로 끝났든(성공이든 실패든) 이 되돌리기는
     if/else 바깥에 두어 항상 실행되도록 했다. */
  g_diag_can_restore_ok = 0u;

  (void)HAL_CAN_Stop(&hcan);
  g_diag_can_started = 0u;

  hcan.Init.Mode = CAN_MODE_NORMAL;

  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    printf("[CAN 자가진단] 되돌리기 실패 : 정상 모드 초기화가 안 됩니다\r\n");
  }
  else if (Can_SetAcceptAllFilter() == 0u)
  {
    printf("[CAN 자가진단] 되돌리기 실패 : 정상 모드 필터 재설정이 안 됩니다\r\n");
  }
  else if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    printf("[CAN 자가진단] 되돌리기 실패 : 정상 모드 시작이 안 됩니다\r\n");
  }
  else
  {
    g_diag_can_started = 1u;

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
      printf("[CAN 자가진단] 되돌리기 실패 : 수신 인터럽트를 다시 켜지 못했습니다\r\n");
    }
    else
    {
      g_diag_can_notify_ok   = 1u;
      g_diag_can_restore_ok  = 1u;   /* 모든 단계가 성공했다 */
      printf("[CAN 자가진단] 정상 모드로 되돌렸습니다 (이제부터 실제 수신을 기다립니다)\r\n");
    }
  }
}
/* USER CODE END Application */

