/**
  ******************************************************************************
  * @file    mpu9255.c
  * @brief   MPU-9255 6축 읽기 + 상보필터 자세 추정 알맹이
  ******************************************************************************
  * @attention
  *
  * 이 파일은 STM32CubeMX 가 만드는 파일이 아니라 사람이 직접 쓴 파일이라
  * USER CODE 표시 구역이 없고, 다시 만들기(Generate Code) 때 지워지지 않는다.
  *
  * [STM32F103 에는 하드웨어 부동소수점 계산기가 없다]
  *   Cortex-M3 코어에는 FPU 가 아예 달려 있지 않아서, 여기 나오는 float 연산은
  *   전부 컴파일러가 만들어 넣은 소프트웨어 함수로 흉내 내어 돌아간다.
  *   한 번에 수십~수백 사이클씩 먹는다는 뜻이다. 50Hz(20ms 마다 한 번)라면
  *   충분히 여유가 있지만, 여기에 실수 계산을 더 얹기 시작하면 금세 빠듯해진다.
  *   그래서 한 걸음에 하는 실수 계산은 꼭 필요한 만큼으로만 묶어 두었다.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mpu9255.h"

/* I2C1 손잡이(hi2c1)를 쓰기 위해 필요하다 */
#include "i2c.h"

/* atan2f() 와 sqrtf() 를 쓰기 위해 필요하다 */
#include <math.h>

/* osDelay() 를 쓰기 위해 필요하다.
   이 파일의 함수들은 전부 FreeRTOS 태스크 안에서 불리므로,
   기다릴 때에는 자리를 넘겨주지 않는 HAL_Delay() 가 아니라 osDelay() 를 써야 한다. */
#include "cmsis_os.h"

/* Private define ------------------------------------------------------------*/

/* AD0 핀을 GND 에 붙여 두었으므로 7비트 주소는 0x68 이다.
   (AD0 를 3.3V 에 붙였다면 0x69 가 된다)
   HAL 함수는 읽기/쓰기 구분 칸이 붙은 8비트 형태의 주소를 받으므로
   7비트 주소를 왼쪽으로 한 칸 밀어서 넘겨야 한다. 이걸 빼먹으면
   엉뚱한 절반 주소로 두드리게 되어 아무 응답도 못 받는다. */
#define MPU9255_ADDR_7BIT            0x68u
#define MPU9255_I2C_ADDR             ((uint16_t)(MPU9255_ADDR_7BIT << 1))

/* ---- 레지스터 번호 (가속도 + 자이로에 필요한 것만 추렸다) ---- */
#define MPU9255_REG_SMPLRT_DIV       0x19u   /* 표본 추출 분주비 */
#define MPU9255_REG_CONFIG           0x1Au   /* 저역통과필터(DLPF) 설정 */
#define MPU9255_REG_GYRO_CONFIG      0x1Bu   /* 자이로 측정 범위 */
#define MPU9255_REG_ACCEL_CONFIG     0x1Cu   /* 가속도 측정 범위 */
#define MPU9255_REG_ACCEL_XOUT_H     0x3Bu   /* 가속도 6바이트의 첫 칸 */
#define MPU9255_REG_GYRO_XOUT_H      0x43u   /* 자이로 6바이트의 첫 칸 */
#define MPU9255_REG_PWR_MGMT_1       0x6Bu   /* 전원 관리 1 (잠에서 깨우는 자리) */
#define MPU9255_REG_WHO_AM_I         0x75u   /* 어떤 칩인지 알려주는 자리 */

/* ---- 위 레지스터에 써 넣을 값 ---- */

/* 0x01 = 자이로의 X축 발진기를 기준 클럭으로 삼는다(PLL).
   동시에 기본값인 잠자기(sleep) 상태를 풀어 준다.
   전원을 넣은 직후의 MPU 는 잠들어 있어서, 이 값을 써 넣기 전에는
   무엇을 읽어도 전부 0 만 나온다. 그래서 이 쓰기가 모든 것보다 먼저다.
   [왜 내부 발진기(0x00)가 아니라 PLL 인가]
   칩 안의 자체 발진기보다 자이로 발진기가 온도에 덜 흔들려서
   데이터시트가 권하는 쪽이기도 하다. */
#define MPU9255_PWR_CLK_PLL_XGYRO    0x01u

/* 0x03 = 자이로 기준 대역폭 약 44Hz.
   흔들림에서 오는 잔가시(고주파 잡음)를 칩 안에서 미리 깎아 준다.
   50Hz 로 표본을 뜨는 우리 입장에서는 이 정도가 알맞다. */
#define MPU9255_DLPF_44HZ            0x03u

/* 0x00 = 자이로 +-250도/초. 이때 1도/초 가 131 LSB 에 해당한다. */
#define MPU9255_GYRO_FS_250DPS       0x00u
#define MPU9255_GYRO_LSB_PER_DPS     131.0f

/* 0x00 = 가속도 +-2g. 이때 1g 가 16384 LSB 에 해당한다.
   기울기만 볼 것이므로 범위는 좁을수록 분해능이 좋아 유리하다. */
#define MPU9255_ACCEL_FS_2G          0x00u
#define MPU9255_ACCEL_LSB_PER_G      16384.0f

/* ---- I2C 주고받기와 영점 측정에 쓰는 수치 ---- */

/* 한 번의 I2C 거래를 기다려 주는 최대 시간(밀리초) */
#define MPU9255_I2C_TIMEOUT_MS       100u

/* 초기화 단계에서만 쓰는 재시도 횟수.
   [왜 초기화에만 재시도를 두는가]
   부팅 직후에는 StartDefaultTask 가 I2C 버스 전체를 훑는 스캔을 돌리고 있어서
   같은 hi2c1 손잡이를 두 태스크가 번갈아 붙잡는 구간이 생긴다.
   그때 HAL 은 손잡이가 잠겨 있다며 HAL_BUSY 를 돌려주는데,
   센서가 고장 난 것이 전혀 아닌데도 초기화가 통째로 실패한 것처럼 보인다.
   그래서 초기화 쪽만 몇 번 더 두드려 보게 해 두었다.
   반대로 20ms 마다 도는 표본 읽기에는 재시도를 두지 않는다.
   주기 안에서 붙잡고 늘어지면 그 주기가 통째로 밀려 적분 간격이 어긋나는데,
   50Hz 로 뜨는 표본 하나쯤 건너뛰는 편이 훨씬 낫기 때문이다. */
#define MPU9255_INIT_RETRY_CNT       3u
#define MPU9255_RETRY_GAP_MS         2u

/* 잠에서 깨운 뒤 기다려 주는 시간(밀리초).
   발진기가 자리를 잡기 전에 설정값을 밀어 넣으면 그 쓰기가 먹히지 않을 수 있다. */
#define MPU9255_WAKE_DELAY_MS        50u

/* 자이로 영점 측정: 5ms 간격으로 200번 = 대략 1초 */
#define MPU9255_CAL_SAMPLE_CNT       200u
#define MPU9255_CAL_SAMPLE_GAP_MS    5u

/* Private variables ---------------------------------------------------------*/

/* [왜 헤더에 extern 으로 내놓지 않고 이 파일 안에만 두는가]
   바깥에서 이 값들을 직접 주무르기 시작하면, 필터가 쌓아 온 상태가
   언제 누구에 의해 바뀌었는지 알 수 없게 되어 값이 튀어도 원인을 못 찾는다.
   그래서 전부 이 파일 안에 가두고, 바깥에는 읽기 전용 함수만 열어 준다. */

/* 상보필터가 쌓아 온 좌우/앞뒤 기울기 (도 단위) */
static float   s_rollDeg        = 0.0f;
static float   s_pitchDeg       = 0.0f;

/* 자이로 영점(bias).
   [단위를 무엇으로 골랐는가]
   이미 도/초(dps)로 바꾼 뒤의 값으로 담아 둔다.
   200개를 더할 때에는 정수(int32_t)로 합을 모아 두었다가 마지막에 딱 한 번
   나눗셈을 해서 도/초로 바꾼다. 이렇게 하면 느린 실수 나눗셈이
   200번이 아니라 2번으로 끝나고, 디버거로 이 값을 들여다볼 때에도
   "가만히 두었는데 초당 몇 도씩 흘러가더라" 를 바로 읽어 낼 수 있어
   영점이 잘 잡혔는지 눈으로 판단하기 쉽다. */
static float   s_gyroXBias      = 0.0f;
static float   s_gyroYBias      = 0.0f;

/* 자이로 영점 측정이 끝났으면 1 (CAN 상태 바이트의 0번 비트로 나간다) */
static uint8_t s_imuCalibrated  = 0u;

/* 초기화 때 읽어 둔 WHO_AM_I 값 (읽지 못했으면 0xFF) */
static uint8_t s_whoAmI         = 0xFFu;

/* Private function prototypes -----------------------------------------------*/
static uint8_t Mpu9255_WriteReg(uint8_t reg, uint8_t value);
static uint8_t Mpu9255_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len, uint8_t max_attempt);
static int16_t Mpu9255_Be16(const uint8_t *p);
static uint8_t Mpu9255_CalibrateGyro(void);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  레지스터 한 칸에 값 하나를 써 넣는다.
  *         초기화 단계에서만 쓰이므로 몇 번 다시 두드려 보게 해 두었다.
  * @param  reg   : 레지스터 번호
  * @param  value : 써 넣을 값
  * @retval 1 = 성공, 0 = 정해진 횟수를 다 쓰고도 실패
  */
static uint8_t Mpu9255_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t attempt;
  uint8_t buf;

  /* HAL 에 넘길 값은 주소를 넘길 수 있는 변수여야 하므로 한 칸에 옮겨 담는다 */
  buf = value;

  for (attempt = 0u; attempt < MPU9255_INIT_RETRY_CNT; attempt++)
  {
    if (HAL_I2C_Mem_Write(&hi2c1, MPU9255_I2C_ADDR, (uint16_t)reg,
                          I2C_MEMADD_SIZE_8BIT, &buf, 1u,
                          MPU9255_I2C_TIMEOUT_MS) == HAL_OK)
    {
      return 1u;
    }

    /* 마지막 시도까지 실패한 뒤에는 굳이 더 쉬지 않고 곧바로 결과를 돌려준다 */
    if ((uint8_t)(attempt + 1u) < MPU9255_INIT_RETRY_CNT)
    {
      osDelay(MPU9255_RETRY_GAP_MS);
    }
  }

  return 0u;
}

/**
  * @brief  레지스터를 첫 칸부터 여러 칸 이어서 한 번에 읽어 온다.
  *         MPU 는 번호가 이어진 레지스터를 한 번의 거래로 죽 읽어 낼 수 있어서,
  *         6바이트를 여섯 번 나눠 읽지 않고 한 묶음으로 가져온다.
  *         이렇게 해야 여섯 축의 값이 서로 다른 시각의 것으로 섞이지 않는다.
  * @param  reg         : 첫 레지스터 번호
  * @param  buf         : 읽어 담을 자리
  * @param  len         : 읽을 칸 수
  * @param  max_attempt : 최대 시도 횟수 (1 이면 재시도 없이 한 번만)
  * @retval 1 = 성공, 0 = 실패
  */
static uint8_t Mpu9255_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len, uint8_t max_attempt)
{
  uint8_t attempt;

  for (attempt = 0u; attempt < max_attempt; attempt++)
  {
    if (HAL_I2C_Mem_Read(&hi2c1, MPU9255_I2C_ADDR, (uint16_t)reg,
                         I2C_MEMADD_SIZE_8BIT, buf, len,
                         MPU9255_I2C_TIMEOUT_MS) == HAL_OK)
    {
      return 1u;
    }

    if ((uint8_t)(attempt + 1u) < max_attempt)
    {
      osDelay(MPU9255_RETRY_GAP_MS);
    }
  }

  return 0u;
}

/**
  * @brief  이어진 두 바이트를 부호 있는 16비트 값 하나로 붙인다.
  *         MPU 는 큰 자리(상위 바이트)를 먼저 내보내는 빅엔디언이라,
  *         앞의 바이트를 왼쪽으로 8칸 밀고 뒤의 바이트를 얹어야 한다.
  *         순서를 바꿔 붙이면 값이 통째로 엉뚱해지므로 반드시 이대로 두어야 한다.
  * @param  p : 두 바이트가 놓인 자리 (p[0] = 상위, p[1] = 하위)
  * @retval 붙여 만든 부호 있는 16비트 값
  */
static int16_t Mpu9255_Be16(const uint8_t *p)
{
  /* 먼저 부호 없는 16비트로 온전히 붙인 다음 부호 있는 형으로 바꾼다.
     바로 int16_t 로 밀어 넣으면 중간 계산에서 부호 비트가 어떻게 다뤄질지가
     컴파일러에 따라 달라질 여지가 있어, 이렇게 두 걸음으로 나눈다. */
  return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/**
  * @brief  자이로 영점(bias)을 잰다. 부팅 직후 딱 한 번만 부른다.
  *
  *         자이로는 가만히 두어도 0 이 아니라 조금씩 치우친 값을 내놓는다.
  *         그 치우침을 그대로 적분하면 몇 초 만에 각도가 눈에 띄게 흘러간다.
  *         그래서 보드가 아직 움직이지 않는 부팅 직후에 여러 번 읽어 평균을 내고,
  *         그 값을 이후 모든 표본에서 빼 준다.
  *
  *         [읽기에 실패한 표본은 세지 않는다]
  *         부팅 직후에는 I2C 버스 스캔과 겹쳐 몇 개쯤 실패할 수 있다.
  *         실패한 것을 0 으로 쳐서 평균에 넣으면 영점이 실제보다 작게 잡혀
  *         흘러감이 그대로 남는다. 그래서 성공한 개수로만 나눈다.
  *
  * @retval 1 = 영점을 구했음, 0 = 한 표본도 못 읽어 영점을 못 구했음
  */
static uint8_t Mpu9255_CalibrateGyro(void)
{
  uint8_t  buf[6];
  uint16_t i;
  uint16_t good_cnt = 0u;   /* 제대로 읽어 낸 표본 개수 */
  int32_t  sum_x    = 0;    /* 원시값(LSB) 합계 - 실수 나눗셈을 마지막에 한 번만 하려고 */
  int32_t  sum_y    = 0;

  for (i = 0u; i < MPU9255_CAL_SAMPLE_CNT; i++)
  {
    /* 여기서도 재시도는 두지 않는다. 어차피 200번을 도는 중이라
       한두 개 놓쳐도 평균에는 거의 영향이 없고,
       재시도로 붙잡고 있으면 영점 측정 시간만 들쭉날쭉해진다. */
    if (Mpu9255_ReadRegs(MPU9255_REG_GYRO_XOUT_H, buf, 6u, 1u) != 0u)
    {
      sum_x += (int32_t)Mpu9255_Be16(&buf[0]);
      sum_y += (int32_t)Mpu9255_Be16(&buf[2]);
      good_cnt++;
    }

    /* 표본 사이를 5ms 씩 띄운다. 붙여서 몰아 읽으면 같은 순간의 잡음을
       여러 번 세는 꼴이 되어 평균을 내는 뜻이 옅어진다.
       그리고 이 틈에 다른 태스크가 돌 자리도 생긴다. */
    osDelay(MPU9255_CAL_SAMPLE_GAP_MS);
  }

  if (good_cnt == 0u)
  {
    /* 한 개도 못 읽었다면 영점을 0 으로 두는 수밖에 없다.
       이때는 "쟀다"고 말할 수 없으므로 표시를 세우지 않는다.
       CAN 상태 바이트의 0번 비트가 계속 0 으로 나가므로,
       받는 쪽에서도 이 각도를 믿으면 안 된다는 것을 알 수 있다. */
    s_gyroXBias     = 0.0f;
    s_gyroYBias     = 0.0f;
    s_imuCalibrated = 0u;
    return 0u;
  }

  /* 원시값 평균을 구한 뒤 도/초 단위로 바꾼다 (실수 나눗셈은 여기서 딱 두 번) */
  s_gyroXBias = ((float)sum_x / (float)good_cnt) / MPU9255_GYRO_LSB_PER_DPS;
  s_gyroYBias = ((float)sum_y / (float)good_cnt) / MPU9255_GYRO_LSB_PER_DPS;

  s_imuCalibrated = 1u;
  return 1u;
}

/* Public user code ----------------------------------------------------------*/

/**
  * @brief  센서를 깨우고 설정한 뒤 자이로 영점까지 잰다.
  *
  *         [WHO_AM_I 가 달라도 멈추지 않는 이유]
  *         MPU-9255 는 보통 0x73 을 돌려주지만, 시중에 도는 모듈 중에는
  *         겉면만 9255 이고 실제로는 6500 이나 9250 이 올라간 것이 흔하다.
  *         레지스터 배치와 눈금은 어차피 같으므로 그런 칩이어도 이 코드는 그대로 돈다.
  *         여기서 값이 다르다고 딱 잘라 실패로 처리해 버리면, 멀쩡히 동작할
  *         하드웨어를 붙여 놓고도 시험을 시작조차 못 하게 된다.
  *         그래서 값은 남겨만 두고 판단은 사람에게 맡긴다.
  *         0 도 0xFF 도 아닌 값이 돌아왔다면 그 자체로 I2C 는 살아 있다는 뜻이다.
  *
  * @retval 1 = 모든 단계 성공, 0 = 어느 한 단계라도 실패
  */
uint8_t Mpu9255_Init(void)
{
  uint8_t who = 0u;
  uint8_t all_ok = 1u;

  /* 다시 불릴 일은 없지만, 불렸을 때 지난 상태가 남아 있지 않도록 정리해 둔다 */
  s_rollDeg       = 0.0f;
  s_pitchDeg      = 0.0f;
  s_gyroXBias     = 0.0f;
  s_gyroYBias     = 0.0f;
  s_imuCalibrated = 0u;
  s_whoAmI        = 0xFFu;

  /* ---------- 1단계 : 통신이 되는지 먼저 확인한다 ---------- */
  if (Mpu9255_ReadRegs(MPU9255_REG_WHO_AM_I, &who, 1u, MPU9255_INIT_RETRY_CNT) != 0u)
  {
    s_whoAmI = who;
  }
  else
  {
    /* 0xFF 는 "읽지 못했다"는 뜻으로 쓰는 값이다 (freertos.c 의 스캔과 같은 약속) */
    s_whoAmI = 0xFFu;
    all_ok   = 0u;
  }

  /* ---------- 2단계 : 잠에서 깨운다 ----------
     이것이 가장 먼저 성공해야 하는 쓰기다. 이 전에는 무엇을 읽어도 0 만 나온다. */
  if (Mpu9255_WriteReg(MPU9255_REG_PWR_MGMT_1, MPU9255_PWR_CLK_PLL_XGYRO) == 0u)
  {
    all_ok = 0u;
  }

  /* 발진기가 자리를 잡을 때까지 잠깐 기다린다.
     이 자리는 FreeRTOS 태스크 안이므로 HAL_Delay() 가 아니라 osDelay() 다.
     HAL_Delay() 는 기다리는 동안 자리를 안 내주어 다른 태스크를 전부 세워 버린다. */
  osDelay(MPU9255_WAKE_DELAY_MS);

  /* ---------- 3단계 : 측정 조건을 정한다 ----------
     [SMPLRT_DIV(0x19)를 건드리지 않는 이유]
     전원을 넣으면 이 값은 0 이고, 그것이 곧 "가장 빠른 속도로 갱신하라"는 뜻이다.
     우리는 20ms 마다 필요할 때 꺼내 쓰는 방식이라 칩이 우리보다 빨리 갱신해 주는 편이
     오히려 낫다. 그래서 일부러 손대지 않고 기본값 그대로 둔다. */
  if (Mpu9255_WriteReg(MPU9255_REG_CONFIG, MPU9255_DLPF_44HZ) == 0u)
  {
    all_ok = 0u;
  }

  if (Mpu9255_WriteReg(MPU9255_REG_GYRO_CONFIG, MPU9255_GYRO_FS_250DPS) == 0u)
  {
    all_ok = 0u;
  }

  if (Mpu9255_WriteReg(MPU9255_REG_ACCEL_CONFIG, MPU9255_ACCEL_FS_2G) == 0u)
  {
    all_ok = 0u;
  }

  /* ---------- 4단계 : 자이로 영점을 잰다 (약 1초) ----------
     보드가 아직 가만히 있다는 것을 전제로 한다.
     이 사이에 보드를 들거나 흔들면 그 움직임이 영점으로 굳어 버려
     이후 각도가 계속 한쪽으로 흘러간다. */
  if (Mpu9255_CalibrateGyro() == 0u)
  {
    all_ok = 0u;
  }

  return all_ok;
}

/**
  * @brief  가속도와 자이로를 한 번씩 읽어 상보필터를 한 걸음 전진시킨다.
  *
  *         [두 값을 왜 섞는가]
  *         가속도만 쓰면 중력 방향을 보고 각도를 바로 알 수 있지만,
  *         차가 흔들리거나 가속하면 그 힘까지 중력으로 착각해 각도가 요동친다.
  *         자이로만 쓰면 짧은 시간은 매끈하고 정확하지만, 아주 작은 치우침이
  *         적분에 계속 쌓여 시간이 지날수록 실제와 멀어진다.
  *         그래서 자이로로 매끄럽게 이어 가되(98%), 가속도로 조금씩 끌어당겨(2%)
  *         흘러감을 계속 되잡아 준다. 이것이 상보필터다.
  *
  * @retval 1 = 이번 표본을 반영했음, 0 = I2C 읽기에 실패해 건너뜀
  */
uint8_t Mpu9255_Update(void)
{
  uint8_t buf[6];
  float   ax;             /* 앞뒤 축 가속도 (g 단위) */
  float   ay;             /* 좌우 축 가속도 (g 단위) */
  float   az;             /* 위아래 축 가속도 (g 단위) */
  float   gx;             /* 좌우 기울기 각속도 (도/초, 영점을 이미 뺀 값) */
  float   gy;             /* 앞뒤 기울기 각속도 (도/초, 영점을 이미 뺀 값) */
  float   accelRollDeg;   /* 가속도만 보고 구한 좌우 기울기 */
  float   accelPitchDeg;  /* 가속도만 보고 구한 앞뒤 기울기 */

  /* ---------- 가속도 6바이트를 한 묶음으로 읽는다 ---------- */
  if (Mpu9255_ReadRegs(MPU9255_REG_ACCEL_XOUT_H, buf, 6u, 1u) == 0u)
  {
    /* 실패했으면 이번 걸음은 통째로 건너뛴다.
       반쪽짜리 값으로 적분하면 그 오차가 상태에 영원히 남기 때문이다. */
    return 0u;
  }

  ax = (float)Mpu9255_Be16(&buf[0]) / MPU9255_ACCEL_LSB_PER_G;
  ay = (float)Mpu9255_Be16(&buf[2]) / MPU9255_ACCEL_LSB_PER_G;
  az = (float)Mpu9255_Be16(&buf[4]) / MPU9255_ACCEL_LSB_PER_G;

  /* ---------- 자이로 6바이트를 한 묶음으로 읽는다 ---------- */
  if (Mpu9255_ReadRegs(MPU9255_REG_GYRO_XOUT_H, buf, 6u, 1u) == 0u)
  {
    return 0u;
  }

  /* 원시값을 도/초로 바꾼 뒤, 부팅 때 재 둔 영점을 뺀다.
     Z축(방위각)은 이번 단계에서 쓰지 않으므로 아예 꺼내지도 않는다.
     FPU 가 없는 칩이라 쓰지 않을 값을 굳이 실수로 바꾸는 것도 낭비다. */
  gx = ((float)Mpu9255_Be16(&buf[0]) / MPU9255_GYRO_LSB_PER_DPS) - s_gyroXBias;
  gy = ((float)Mpu9255_Be16(&buf[2]) / MPU9255_GYRO_LSB_PER_DPS) - s_gyroYBias;

  /* ---------- 가속도만 보고 각도를 구한다 ----------
     보드가 가만히 있으면 가속도계가 느끼는 것은 중력뿐이다.
     그 중력이 어느 축으로 얼마나 나뉘어 들어오는지를 보면 기울기를 알 수 있다.
       좌우(roll)  : 왼쪽 축과 위쪽 축이 나눠 가진 비율
       앞뒤(pitch) : 앞쪽 축이, 나머지 두 축을 합친 크기에 견주어 가진 비율
     atan2f 는 두 값의 부호까지 함께 보아 -180 ~ +180 도 전 구간을 가려낸다.
     (atanf 였다면 -90 ~ +90 도밖에 못 가려 뒤집힌 자세를 구분하지 못한다) */
  accelRollDeg  = atan2f(ay, az) * IMU_RAD_TO_DEG;
  accelPitchDeg = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * IMU_RAD_TO_DEG;

  /* ---------- 상보필터 한 걸음 ----------
     괄호 안 : 직전 각도에 이번 20ms 동안 돌아간 만큼을 더한다 (자이로 적분)
     바깥    : 그 결과를 98%, 가속도가 말하는 각도를 2% 로 섞는다 */
  s_rollDeg  = (IMU_ALPHA * (s_rollDeg  + (gx * IMU_DT_SEC))) +
               ((1.0f - IMU_ALPHA) * accelRollDeg);
  s_pitchDeg = (IMU_ALPHA * (s_pitchDeg + (gy * IMU_DT_SEC))) +
               ((1.0f - IMU_ALPHA) * accelPitchDeg);

  return 1u;
}

/**
  * @brief  걸러낸 좌우 기울기(도)를 돌려준다.
  *
  *         [부호 뒤집기를 왜 여기서 하는가]
  *         뒤집기를 필터 안에서 s_rollDeg 자체에 걸어 버리면,
  *         20ms 마다 쌓아 온 상태의 부호가 한 걸음마다 뒤집힌다.
  *         그러면 다음 걸음이 부호가 뒤집힌 값 위에 다시 적분을 얹게 되어
  *         값이 제자리에서 +와 - 를 오가며 튀기만 하고 실제 각도에
  *         영영 다다르지 못한다. 필터가 통째로 망가지는 것이다.
  *         뒤집기는 "밖으로 내보낼 때의 방향 약속"일 뿐 필터가 쌓는 상태와는
  *         상관이 없으므로, 상태는 손대지 않고 내보내는 길목에서만 뒤집는다.
  *         이렇게 해도 스위치를 켰을 때 얻고자 하는 결과(좌우가 반대로 나오던 것이
  *         바로잡히는 것)는 똑같이 얻어진다.
  * @retval 좌우 기울기 (도)
  */
float Mpu9255_GetRollDeg(void)
{
#if IMU_ROLL_INVERT
  return -s_rollDeg;
#else
  return s_rollDeg;
#endif
}

/**
  * @brief  걸러낸 앞뒤 기울기(도)를 돌려준다.
  *         부호 뒤집기를 내보내는 길목에서 하는 이유는 위 함수의 설명과 같다.
  * @retval 앞뒤 기울기 (도)
  */
float Mpu9255_GetPitchDeg(void)
{
#if IMU_PITCH_INVERT
  return -s_pitchDeg;
#else
  return s_pitchDeg;
#endif
}

/**
  * @brief  자이로 영점 측정이 끝났는지 알려준다.
  * @retval 1 = 끝났음(각도를 믿어도 된다), 0 = 아직이거나 측정에 실패했음
  */
uint8_t Mpu9255_IsCalibrated(void)
{
  return s_imuCalibrated;
}

/**
  * @brief  초기화 때 읽어 둔 WHO_AM_I 값을 돌려준다.
  * @retval 읽은 값 (읽지 못했으면 0xFF)
  */
uint8_t Mpu9255_GetWhoAmI(void)
{
  return s_whoAmI;
}
