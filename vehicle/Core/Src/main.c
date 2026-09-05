/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  * [수정 사항]
  * 1. AUTOSAR-lite 3계층(ASW/RTE/BSW) 구조로 재편 (ecu_l298n/ecu_hcsr04/
  *    mcal_can/rte_mode_manager/rte_motor/rte_sensor/asw_manual_control/asw_autonomous)
  * 2. 초기화 순서: BSW/ECU(RTE 경유) -> RTE -> BSW/MCAL -> ASW
  * 3. CAN Rx FIFO0 콜백을 mcal_can 모듈로 위임
  * 4. osKernelStart() 이후 도달 불가능한 코드 없음 (Error_Handler 자체가
  *    내부 무한루프이므로 그 아래에는 어떤 코드도 올 수 없음)
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>   /* printf/setvbuf/stdout 사용 */
#include "rte_motor.h"
#include "rte_sensor.h"
#include "mcal_can.h"
#include "rte_mode_manager.h"
#include "asw_manual_control.h"
#include "asw_autonomous.h"
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

/* USER CODE BEGIN PV */
/* ==========================================================================
 * CAN 버스 진단용 전역 변수 (STM32CubeIDE의 Live Expressions 창 관찰 전용)
 *
 * 이 보드에는 USB-TTL 어댑터가 없어 printf 로그를 눈으로 볼 수 없다. 그래서
 * 디버거의 Live Expressions 창에서 값을 직접 들여다볼 수 있도록, 파일 스코프
 * static이 아니라 "static 없는 volatile 전역"으로 선언한다.
 *   - static을 붙이면 심볼이 파일 내부에 숨어 디버거가 못 찾는 경우가 있다.
 *   - volatile을 붙여야 컴파일러가 값을 레지스터에만 담아두지 않고 매번 RAM에
 *     써주므로, 디버거가 읽는 값이 항상 최신값이 된다.
 *
 * 값의 출처는 CAN 오류상태 레지스터(CAN_ESR)이며, 비트 배치는 RM0008
 * (STM32F1 참조매뉴얼) 기준이다.
 *   REC  = 비트 31:24 (수신 오류 카운터)
 *   TEC  = 비트 23:16 (송신 오류 카운터)
 *   LEC  = 비트  6:4  (마지막 오류 코드)
 *   BOFF = 비트 2     (버스오프 상태)
 *   EPVF = 비트 1     (오류 수동 상태)
 *   EWGF = 비트 0     (오류 경고 상태)
 *
 * LEC(마지막 오류 코드) 값 의미표
 *   0 : 오류 없음
 *   1 : 스터프 오류
 *   2 : 폼 오류
 *   3 : ACK 오류   <-- 응답(ACK)해 주는 노드가 버스에 하나도 없다는 뜻
 *   4 : 비트 리세시브 오류
 *   5 : 비트 도미넌트 오류
 *   6 : CRC 오류
 *   7 : 소프트웨어가 설정한 값
 * ========================================================================== */
volatile uint8_t  g_vdiag_can_tec         = 0u;  /* 송신 오류 카운터 (ESR 23:16) */
volatile uint8_t  g_vdiag_can_rec         = 0u;  /* 수신 오류 카운터 (ESR 31:24) */
volatile uint8_t  g_vdiag_can_lec         = 0u;  /* 마지막 오류 코드 0~7 (ESR 6:4) */
volatile uint8_t  g_vdiag_can_boff        = 0u;  /* 버스오프 상태면 1 (ESR 비트2) */
volatile uint8_t  g_vdiag_can_epvf        = 0u;  /* 오류 수동 상태면 1 (ESR 비트1) */
volatile uint8_t  g_vdiag_can_ewgf        = 0u;  /* 오류 경고 상태면 1 (ESR 비트0) */
volatile uint32_t g_vdiag_can_esr_raw     = 0u;  /* ESR 레지스터 원본값 */
volatile uint32_t g_vdiag_can_tx_ok       = 0u;  /* 송신 요청 성공 누적 횟수 */
volatile uint32_t g_vdiag_can_tx_fail     = 0u;  /* 송신 요청 실패 누적 횟수 */
volatile uint32_t g_vdiag_can_free_mb     = 0u;  /* 비어 있는 송신 메일박스 수 (0~3) */
volatile uint32_t g_vdiag_can_recover_cnt = 0u;  /* 버스오프 복구 "성공" 누적 횟수
                                                  * (시도가 아니라 Stop+Start가 모두
                                                  *  성공한 경우에만 증가한다) */
volatile uint8_t  g_vdiag_can_stuck       = 0u;  /* 지금 버스오프 복구가 안 되고 막혀 있는 상태면 1
                                                  * (영구 래치가 아니라 현재 상태를 나타내는 레벨
                                                  *  플래그. 복구에 성공하는 즉시 0으로 돌아온다) */
volatile uint8_t  g_vdiag_can_notify_ok   = 0u;  /* CAN 수신 인터럽트가 켜져 있으면 1 */

/* CtrlTask가 한 주기 돌 때마다 1씩 증가 (생존 증명).
 * CanTxTask가 약 1초 주기로 이 값의 증가 여부를 확인해 HEARTBEAT에 실어 보낸다. */
volatile uint32_t g_ctrl_loop_alive_counter = 0u;
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
  /* newlib-nano 기본값은 non-tty stdout을 Full-Buffered로 취급해서
     버퍼가 다 찰 때까지(또는 fflush) printf가 실제로 나가지 않는다.
     임베디드는 exit()이 없으므로 반드시 Unbuffered로 강제해야
     디버그 로그가 즉시 시리얼 터미널에 출력된다. */
  setvbuf(stdout, NULL, _IONBF, 0);
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  /* ---- 계층 순서대로 초기화: BSW/ECU(RTE 경유) -> RTE -> BSW/MCAL -> ASW ----
   * 1) BSW/ECU : 모터 PWM 시작, 초음파 IC 시작 (하드웨어 최우선 기동, RTE 경유)
   * 2) RTE     : 주행모드 Mutex 준비
   * 3) BSW/MCAL: CAN Mail Queue/필터/인터럽트 준비
   * 4) ASW     : 수동/자율 로직 초기 상태(정지) 세팅
   *
   * RTE_Mode_Init()이 MCAL_CAN_Init()보다 먼저 와야 하는 이유:
   *   CAN RX 인터럽트가 활성화된 순간부터 ASW_Manual_ApplyCanCommand()가
   *   RTE_Mode_SetDriveMode()를 호출할 수 있으므로, Mutex가
   *   미리 생성되어 있어야 함.
   */
  RTE_Motor_Init();
  RTE_Sensor_Init();

  RTE_Mode_Init();
  MCAL_CAN_Init();
  MCAL_BtSerial_Init();

  ASW_Manual_Init();
  ASW_Autonomous_Init();

  printf("\r\n[BOOT] Node2 (STM32F103C8T6) Ready - Mode=MANUAL, CAN=500kbps\r\n");
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* 여기까지 오면 스케줄러 시작 실패 -> 안전하게 정지
     (Error_Handler() 내부가 이미 무한루프이므로 이 아래에는
      어떤 코드도 올 수 없다) */
  Error_Handler();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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
 * @brief printf 저수준 출력 -> USART1(115200bps)로 1바이트씩 전송.
 *        syscalls.c의 _write()가 호출하는 weak 함수를 여기서 구현해야
 *        printf가 실제로 시리얼 모니터에 출력된다.
 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/**
 * @brief TIM4 IC 캡처 콜백 -> rte_sensor(ECU_HCSR04) 모듈로 위임
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    RTE_Sensor_HandleCaptureIsr(htim);
}

/**
 * @brief CAN RX FIFO0 메시지 도착 콜백 -> mcal_can 모듈로 위임 (ISR Context)
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    MCAL_CAN_HandleRxFifoIsr(hcan);
}

/**
 * @brief USART1 수신 완료 콜백 -> mcal_bt_serial 모듈로 위임 (ISR Context)
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    MCAL_BtSerial_HandleRxCpltIsr(huart);
}

/**
 * @brief USART1 수신 에러 콜백 -> mcal_bt_serial 모듈로 위임 (ISR Context)
 *        오버런(ORE) 등 에러가 나면 HAL이 수신을 중단해 버리므로,
 *        여기서 에러 플래그를 지우고 수신을 다시 걸어 복구한다.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    MCAL_BtSerial_HandleErrorIsr(huart);
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
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
