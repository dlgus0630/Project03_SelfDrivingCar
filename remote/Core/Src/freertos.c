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

/* USER CODE END Application */

