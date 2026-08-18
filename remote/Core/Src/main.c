/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* CAN 수신 원형 버퍼의 칸 수.
   인터럽트가 빠르게 받아 담아두고 태스크가 천천히 꺼내 가므로
   잠깐 몰리는 것만 견디면 되어 8칸이면 충분하다. */
#define CAN_RX_RING_SIZE   8u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ===== CAN 수신 원형 버퍼 =====
   수신 인터럽트(ISR)가 head 를 밀면서 쓰고,
   CanRxTask 가 tail 을 밀면서 읽는다.
   쓰는 쪽 하나, 읽는 쪽 하나뿐이므로(생산자 1 : 소비자 1)
   인터럽트를 잠그지 않아도 서로 충돌하지 않는다.
   인터럽트가 값을 바꾸는 변수들이므로 컴파일러가 레지스터에
   캐시해 두지 못하도록 전부 volatile 로 선언한다. */
static volatile CanRxFrame_t g_can_ring[CAN_RX_RING_SIZE];
static volatile uint16_t     g_can_head     = 0;  /* 인터럽트가 다음에 쓸 칸 */
static volatile uint16_t     g_can_tail     = 0;  /* 태스크가 다음에 읽을 칸 */
static volatile uint32_t     g_can_total    = 0;  /* 총 수신 개수 */
static volatile uint32_t     g_can_overflow = 0;  /* 버퍼가 꽉 차서 버린 개수 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* printf 로 찍은 글자가 버퍼에 쌓여 있다가 화면에 안 나오는 일을 막는다.
     _IONBF 는 "버퍼를 쓰지 않는다"는 뜻으로, 한 글자 생길 때마다 바로 보낸다.
     진단용 코드에서는 보드가 멈춰버려도 그 직전까지 찍힌 내용이
     그대로 남아 있어야 원인을 찾을 수 있다. */
  setvbuf(stdout, NULL, _IONBF, 0);

  /* 부팅 배너 - 지금 켜진 보드가 어느 쪽인지 한눈에 알아보기 위한 것 */
  printf("\r\n========================================\r\n");
  printf(" Project03_Remote (조종 노드) 시작\r\n");
  printf(" CAN 500kbps / I2C1 400kHz / UART 115200\r\n");
  printf("========================================\r\n");

  /* ===== CAN 수신 필터 설정 =====
     마스크를 0으로 두면 ID 를 비교하는 자리가 하나도 없어져서
     결과적으로 모든 ID 가 통과한다.
     지금은 진단이 목적이라 버스에 실제로 어떤 ID 가 흐르고 있는지를
     눈으로 전부 확인해야 하므로 일부러 전부 통과시킨다. */
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

  /* 아래 세 가지는 실패하더라도 Error_Handler() 로 보내지 않는다.
     Error_Handler() 는 인터럽트를 모두 끄고 무한루프에 갇히기 때문에,
     CAN 트랜시버를 연결하지 않은 채 전원을 켜면 보드가 통째로 먹통이 되어
     UART 로그조차 한 줄도 안 나온다. 그러면 무엇이 잘못됐는지 알 수 없다.
     그래서 실패했다는 사실만 화면에 알리고,
     나머지 진단(I2C 스캔 등)은 그대로 이어서 진행하도록 한다. */
  if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK)
  {
    printf("[CAN] 필터 설정 실패 (그래도 계속 진행합니다)\r\n");
  }
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    printf("[CAN] 시작 실패 - 트랜시버 연결과 배선을 확인하세요 (그래도 계속 진행합니다)\r\n");
  }
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    printf("[CAN] 수신 인터럽트 켜기 실패 (그래도 계속 진행합니다)\r\n");
  }
  else
  {
    printf("[CAN] 준비 완료 - 모든 ID 를 통과시키는 필터로 대기합니다\r\n");
  }
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  printf 가 글자 하나를 내보낼 때 호출되는 함수.
  *         syscalls.c 안의 _write() 가 글자마다 이 함수를 부른다.
  *         여기서 USART1 로 실제 전송하므로, 이 함수만 있으면
  *         printf 출력이 디버그용 시리얼 창에 그대로 나타난다.
  */
int __io_putchar(int ch)
{
  /* ch 는 int 지만 실제로 보낼 것은 맨 아래 1바이트뿐이다 */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
  return ch;
}

/**
  * @brief  CAN 수신 FIFO0 에 프레임이 도착했을 때 자동으로 불리는 함수(인터럽트).
  *
  *         [중요] 이 안에서는 절대로 printf 를 부르지 않는다.
  *         printf 는 UART 로 한 글자씩 다 나갈 때까지 기다리는 방식이라
  *         인터럽트가 그만큼 오래 붙잡혀 있게 되고,
  *         그 사이에 들어온 다음 프레임을 놓쳐버린다.
  *         그래서 여기서는 원형 버퍼에 담아두고 개수만 세고 곧바로 빠져나오며,
  *         실제 화면 출력은 CanRxTask 가 여유 있을 때 대신 처리한다.
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_arg)
{
  CAN_RxHeaderTypeDef header;
  uint8_t             rx_data[8];
  uint16_t            head;
  uint16_t            next;
  uint8_t             i;

  /* 하드웨어 수신함에서 프레임을 꺼낸다 */
  if (HAL_CAN_GetRxMessage(hcan_arg, CAN_RX_FIFO0, &header, rx_data) != HAL_OK)
  {
    return;
  }

  /* 총 수신 개수는 버퍼가 꽉 찼든 아니든 항상 센다 */
  g_can_total++;

  head = g_can_head;
  next = (uint16_t)(head + 1u);
  if (next >= CAN_RX_RING_SIZE)
  {
    next = 0u;
  }

  if (next == g_can_tail)
  {
    /* 버퍼가 가득 찼다. 이번 프레임은 버리고 버린 개수만 따로 센다. */
    g_can_overflow++;
    return;
  }

  if (header.IDE == CAN_ID_STD)
  {
    g_can_ring[head].Id    = header.StdId;   /* 표준 11비트 ID */
    g_can_ring[head].IsExt = 0u;
  }
  else
  {
    g_can_ring[head].Id    = header.ExtId;   /* 확장 29비트 ID */
    g_can_ring[head].IsExt = 1u;
  }
  g_can_ring[head].Dlc = (uint8_t)header.DLC;

  for (i = 0u; i < 8u; i++)
  {
    g_can_ring[head].Data[i] = rx_data[i];
  }

  /* 내용을 다 채운 뒤에 마지막으로 head 를 옮긴다.
     읽는 쪽이 아직 덜 채워진 칸을 먼저 보게 되는 것을 막기 위한 순서다. */
  g_can_head = next;
}

/**
  * @brief  원형 버퍼에서 CAN 프레임을 하나 꺼낸다. (CanRxTask 전용)
  * @retval 1 = 하나 꺼냈음, 0 = 버퍼가 비어 있음
  */
uint8_t CanRx_Pop(CanRxFrame_t *out)
{
  uint16_t tail;
  uint8_t  i;

  if (out == NULL)
  {
    return 0u;
  }

  tail = g_can_tail;
  if (tail == g_can_head)
  {
    return 0u;   /* 비어 있음 */
  }

  /* volatile 인 버퍼에서 일반 변수로 한 항목씩 옮겨 담는다 */
  out->Id    = g_can_ring[tail].Id;
  out->IsExt = g_can_ring[tail].IsExt;
  out->Dlc   = g_can_ring[tail].Dlc;
  for (i = 0u; i < 8u; i++)
  {
    out->Data[i] = g_can_ring[tail].Data[i];
  }

  tail = (uint16_t)(tail + 1u);
  if (tail >= CAN_RX_RING_SIZE)
  {
    tail = 0u;
  }
  g_can_tail = tail;   /* 다 옮긴 뒤에 tail 을 옮긴다 */

  return 1u;
}

/**
  * @brief  부팅 후 지금까지 받은 CAN 프레임 총 개수를 알려준다.
  */
uint32_t CanRx_GetTotal(void)
{
  return g_can_total;
}

/**
  * @brief  원형 버퍼가 가득 차서 버려진 프레임 개수를 알려준다.
  *         이 값이 계속 늘어난다면 출력이 수신 속도를 못 따라가고 있다는 뜻이다.
  */
uint32_t CanRx_GetOverflow(void)
{
  return g_can_overflow;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
