/*
 * asw_autonomous.c
 *  Layer  : ASW
 *  Module : ASW_AUTONOMOUS
 *
 *  3개 초음파 센서(좌/전/우) 거리값 기반 독립적 자율주행(미로 통과) 로직.
 *
 *  상태머신(DRIVE / ARC_TURN / PIVOT / BACKING_UP) 기반 비차단 설계.
 *  CtrlTask가 50ms마다 ASW_Autonomous_ProcessControl()을 호출하므로,
 *  본 모듈 내부에서는 osDelay 등 블로킹 호출을 사용하지 않는다.
 *
 *  [v4 설계 철학 : "멈추지 않는다"]
 *   이전 버전(정지->피벗->정지)은 코너마다 차가 서고, 펄스 피벗은
 *   회전-정지를 반복(띠띠띠띠)하며 시간을 크게 잃었다. v4는 일반
 *   코너를 전부 "아크 선회"(바깥 바퀴 고속/안쪽 바퀴 저속 전진)로
 *   주행 중에 통과한다. 제자리 피벗은 아크로도 못 도는 비상시에만.
 *
 *  [v4.1 추가 : 방향 근거 없을 때의 CREEP]
 *   대각선 박스는 초음파를 빗겨 반사시켜 "전방만 먼저 막히고 좌우
 *   입구는 아직 안 보이는" 상황을 만든다. 이때 즉시 회전을 잠그면
 *   반반 확률로 U턴이므로, 방향 근거가 생길 때까지 저속 직진(creep)
 *   하며 기다렸다가 확실해진 뒤 선회한다.
 *
 *  [U턴(출발지 복귀) 방지 핵심]
 *   - 원인 1 : 코너 도착 "순간"의 좌/우 비교 1회로 회전 방향을 결정
 *     -> 대각선 박스 난반사(실제보다 멀게 읽힘) 한 방에 반대로 회전,
 *        반대로 돌면 90도 통로보다 왔던 길(180도)이 먼저 열려 U턴.
 *   - 해결 1 : 접근 구간(전방<65cm) 동안 (좌-우)를 매 사이클 적분한
 *     "투표 점수"로 방향 결정. 순간 오판이 누적 점수를 못 뒤집는다.
 *   - 원인 2 : 회전 중 전방 개방 판정을 놓쳐 과회전.
 *   - 해결 2 : 일반 코너는 아크 선회라 애초에 회전각이 90도 부근으로
 *     기하학적으로 제한된다. 비상 피벗도 올바른 방향으로 돌기 때문에
 *     90도 부근에서 통로가 먼저 열려 180도까지 갈 일이 없다.
 *   - 원인 3 : 방향 락(DIR_LOCK)이 다른 모든 판단보다 먼저 걸려서, 코너에
 *     명백한 반대쪽 개방 증거(결정적 순간 gap)가 있어도 무조건 직전 회전
 *     방향을 반복 -> ㄱ자(단일) 코너에서도 락에 걸려 왔던 길로 회전.
 *   - 해결 3 : DecideDir()에서 "결정적 순간 개방"(GAP_DECISIVE_CM) 판정을
 *     락보다 먼저 확인하도록 순서 변경. 상승제한 필터를 거친 값이라 순간
 *     난반사로는 이 정도 벌어지지 않으므로, 명백한 증거는 락도 이긴다.
 *
 *  [방향반전 정지 완충]
 *   바퀴 회전방향이 반전되는 전이(전진<->피벗, 아크<->피벗)는 같은 사이클에
 *   반대 방향 명령을 바로 내지 않고 RTE_Motor_Stop()만 호출한 뒤, 다음 50ms
 *   주기에 해당 상태 핸들러가 실제 명령을 내도록 한다(BACKING_UP 전이와 동일
 *   패턴). 양쪽 다 전진이라 방향 반전이 없는 ARC_TURN<->DRIVE 전이는 대상 아님.
 *
 *  상태 개요
 *   - DRIVE      : PD 센터링 + 전방거리 비례 속도로 전진. 접근 구간
 *                  방향 투표 적분. 전방<TURN_ENTER -> ARC_TURN(무정지),
 *                  전방<EMERGENCY -> PIVOT, 좌우까지 막힘 -> BACKING_UP.
 *   - ARC_TURN   : 잠긴 방향으로 이동 선회. 전방 개방 연속 확인 시
 *                  DRIVE 복귀(무정지). ARC_MAX 내 미개방 -> PIVOT.
 *   - PIVOT      : 잠긴 방향 제자리 회전. 개방 연속 확인 시 DRIVE 복귀.
 *                  PIVOT_MAX 초과(갇힘) -> BACKING_UP.
 *   - BACKING_UP : 연속 후진 + 매 사이클 재평가. 회복 -> DRIVE.
 *                  한도 초과 -> 반대 방향 PIVOT 재시도.
 */
#include "asw_autonomous.h"
#include "rte_motor.h"
#include <stdio.h>   /* 진단 로그(printf -> main.c의 __io_putchar -> USART1) */

/* ------------------------------------------------------------------------
 * 내부 상태 정의
 * ------------------------------------------------------------------------ */
typedef enum
{
    ASW_AUTO_STATE_DRIVE = 0,
    ASW_AUTO_STATE_ARC_TURN,
    ASW_AUTO_STATE_PIVOT,
    ASW_AUTO_STATE_BACKING_UP
} AswAutoState_t;

typedef enum
{
    ASW_AUTO_TURN_LEFT = 0,
    ASW_AUTO_TURN_RIGHT
} AswAutoTurnDir_t;

static AswAutoState_t   s_state           = ASW_AUTO_STATE_DRIVE;
static AswAutoTurnDir_t s_turnDir         = ASW_AUTO_TURN_LEFT;
static uint32_t         s_turnElapsedMs   = 0u; /* ARC/PIVOT 경과 시간 */
static uint32_t         s_openCount       = 0u; /* 전방 개방 연속 확인 횟수 */
static uint32_t         s_backupElapsedMs = 0u;

/* 센서 EMA 필터 상태 */
static uint32_t s_fLeft   = 0u;
static uint32_t s_fFront  = 0u;
static uint32_t s_fRight  = 0u;
static uint8_t  s_filtInit = 0u;

/* PD 센터링 / 슬루 제한 상태 */
static int32_t  s_prevDiff  = 0;
static uint8_t  s_prevDiffValid = 0u;
static int32_t  s_curCorr   = 0;  /* 조향 슬루 제한용 현재 보정값 */
static uint32_t s_curLeftDuty  = 0u;
static uint32_t s_curRightDuty = 0u;

/* 회전 방향 투표 점수 : +쪽 = 좌측이 넓음(좌회전), -쪽 = 우회전 */
static int32_t  s_dirScore = 0;

/* 마지막 회전(ARC/PIVOT) 종료 후 경과 시간 : 연속 코너 방향 잠금용 */
static uint32_t s_sinceTurnMs = 60000u;

void ASW_Autonomous_Init(void)
{
    RTE_Motor_Stop(); /* 자율모드 최초 진입 시 안전 정지 상태로 시작 */

    s_state           = ASW_AUTO_STATE_DRIVE;
    s_turnDir         = ASW_AUTO_TURN_LEFT;
    s_turnElapsedMs   = 0u;
    s_openCount       = 0u;
    s_backupElapsedMs = 0u;

    s_filtInit        = 0u;
    s_prevDiff        = 0;
    s_prevDiffValid   = 0u;
    s_curCorr         = 0;
    s_curLeftDuty     = 0u;
    s_curRightDuty    = 0u;
    s_dirScore        = 0;
    s_sinceTurnMs     = 60000u;
}

/* ------------------------------------------------------------------------
 * 내부 헬퍼
 * ------------------------------------------------------------------------ */

/* 비대칭 EMA 필터 : filt = filt + (raw - filt)/w
 *  - 감소(가까워짐)는 즉시 반영 경로(EMA)로, 증가(멀어짐)는 사이클당
 *    RISE_MAX로 제한 -> 타임아웃/난반사 스파이크로 인한 가짜 개방 차단 */
static uint32_t AswAuto_Ema(uint32_t filt, uint32_t raw, uint32_t w)
{
    int32_t d = (int32_t)raw - (int32_t)filt;
    int32_t out = (int32_t)filt + d / (int32_t)w;
    if (out > (int32_t)(filt + ASW_AUTO_FILT_RISE_MAX_CM))
    {
        out = (int32_t)(filt + ASW_AUTO_FILT_RISE_MAX_CM);
    }
    return (uint32_t)out;
}

/* 목표 duty를 향해 사이클당 SLEW_STEP 이내로만 이동 */
static uint32_t AswAuto_SlewLimit(uint32_t current, uint32_t target)
{
    if (target > current)
    {
        uint32_t up = target - current;
        return current + ((up > ASW_AUTO_DUTY_SLEW_STEP) ? ASW_AUTO_DUTY_SLEW_STEP : up);
    }
    else
    {
        uint32_t down = current - target;
        return current - ((down > ASW_AUTO_DUTY_SLEW_STEP) ? ASW_AUTO_DUTY_SLEW_STEP : down);
    }
}

/* DRIVE 상태 (재)진입 준비 */
static void AswAuto_EnterDrive(void)
{
    s_state         = ASW_AUTO_STATE_DRIVE;
    s_prevDiffValid = 0u; /* D항 킥 방지 */
    s_curCorr       = 0;  /* 조향 슬루 기준점 리셋 */
    s_filtInit      = 0u; /* 회전 복귀 = 방향 전환 -> 이전 방향의 묵은 필터값 제거 */
    /* 아크 선회에서 복귀할 때는 이미 구르고 있으므로 duty를 0부터 올리지
     * 않고 SLOW 수준에서 이어받아 무정지 복귀한다 */
    if (s_curLeftDuty  < ASW_AUTO_DUTY_MIN) { s_curLeftDuty  = ASW_AUTO_DUTY_MIN; }
    if (s_curRightDuty < ASW_AUTO_DUTY_MIN) { s_curRightDuty = ASW_AUTO_DUTY_MIN; }
}

/* 회전류 상태(ARC/PIVOT) 공통 진입 준비 */
static void AswAuto_EnterTurnState(AswAutoState_t state, AswAutoTurnDir_t dir)
{
    s_state         = state;
    s_turnDir       = dir;
    s_turnElapsedMs = 0u;
    s_openCount     = 0u;
    s_sinceTurnMs   = 0u; /* 회전 시작 = 연속 코너 방향 잠금 타이머 리셋 */
    s_filtInit      = 0u; /* 회전 진입 = 방향 전환 -> 이전 방향의 묵은 필터값 제거 */
}

/* BACKING_UP 상태 진입 준비 */
static void AswAuto_EnterBackingUp(void)
{
    s_state           = ASW_AUTO_STATE_BACKING_UP;
    s_backupElapsedMs = 0u;
}

/* 회전 방향 결정 : 현재 확실한 개방 > 연속 코너 잠금 > 적분 투표 > 순간값 */
static AswAutoTurnDir_t AswAuto_DecideDir(uint32_t dist_left, uint32_t dist_right)
{
    int32_t gap = (int32_t)dist_left - (int32_t)dist_right;

    /* 지금 이 순간 한쪽이 확실히 열려 있으면(통로 입구) 최우선으로 그쪽 선택.
     * 좌우값은 상승제한 필터를 거친 값이므로 순간 난반사로는 이만큼 안 벌어짐 ->
     * 아래 "직전 회전 방향 유지" 락보다도 신뢰도가 높다. 락을 먼저 보면 ㄱ자
     * 코너처럼 명백히 반대쪽이 열린 상황에서도 락에 걸려 직전 방향을 반복해
     * 왔던 길로 되돌아가는(U턴) 오작동이 났었다 - 반드시 락보다 먼저 확인. */
    if (gap >=  ASW_AUTO_GAP_DECISIVE_CM) { return ASW_AUTO_TURN_LEFT;  }
    if (gap <= -ASW_AUTO_GAP_DECISIVE_CM) { return ASW_AUTO_TURN_RIGHT; }
    /* 직전 회전 직후의 재회전 = 같은 밴드의 연속 코너 -> 같은 방향 유지
     * (위에서 명백한 반대 증거는 이미 걸러졌으므로, 애매한 경우에만 적용) */
    if (s_sinceTurnMs < ASW_AUTO_DIR_LOCK_MS)
    {
        return s_turnDir;
    }
    /* 접근 구간 적분 투표 */
    if (s_dirScore >= ASW_AUTO_VOTE_TH)  { return ASW_AUTO_TURN_LEFT;  }
    if (s_dirScore <= -ASW_AUTO_VOTE_TH) { return ASW_AUTO_TURN_RIGHT; }
    /* 중간 크기의 순간 차이라도 있으면 그쪽 */
    if (gap >=  12) { return ASW_AUTO_TURN_LEFT;  }
    if (gap <= -12) { return ASW_AUTO_TURN_RIGHT; }
    /* 아무 근거도 없으면 직전 회전 방향 반복 : S자 미로의 교차 밴드에서
     * 연속 코너 2개는 같은 방향 쌍이므로 기하학적으로 가장 유리한 추정 */
    return s_turnDir;
}

/* 아크 선회 모터 출력 (무정지 이동 선회) */
static void AswAuto_DriveArc(AswAutoTurnDir_t dir)
{
    if (dir == ASW_AUTO_TURN_RIGHT)
    {
        s_curLeftDuty  = ASW_AUTO_ARC_OUT_DUTY;
        s_curRightDuty = ASW_AUTO_ARC_IN_DUTY;
    }
    else
    {
        s_curLeftDuty  = ASW_AUTO_ARC_IN_DUTY;
        s_curRightDuty = ASW_AUTO_ARC_OUT_DUTY;
    }
    RTE_Motor_DriveForwardDifferential((uint16_t)s_curLeftDuty, (uint16_t)s_curRightDuty);
}

/* ------------------------------------------------------------------------
 * DRIVE 상태 처리
 * ------------------------------------------------------------------------ */
static void AswAuto_HandleDrive(uint32_t dist_left, uint32_t dist_front, uint32_t dist_right)
{
    s_sinceTurnMs += ASW_AUTO_CTRL_PERIOD_MS;

    /* ---- 회전 방향 투표 적분 (접근 구간에서만) ---- */
    {
        uint32_t lc = (dist_left  > ASW_AUTO_CENTER_TRUST_CM) ? ASW_AUTO_CENTER_TRUST_CM : dist_left;
        uint32_t rc = (dist_right > ASW_AUTO_CENTER_TRUST_CM) ? ASW_AUTO_CENTER_TRUST_CM : dist_right;
        if (lc >= (uint32_t)ASW_AUTO_VOTE_OPEN_CM || rc >= (uint32_t)ASW_AUTO_VOTE_OPEN_CM)
        {
            /* 한쪽 입구 개방 신호가 보이는 동안엔 (전방 거리와 무관하게) 적분.
             * 밴드(교차 구간) 통과 중엔 전방이 열려 있어도 옆 입구가 지나가므로
             * 이때의 증거가 다음 코너 방향의 핵심이다 */
            int32_t c = (int32_t)lc - (int32_t)rc;
            if (c >  ASW_AUTO_VOTE_CLAMP) { c =  ASW_AUTO_VOTE_CLAMP; }
            if (c < -ASW_AUTO_VOTE_CLAMP) { c = -ASW_AUTO_VOTE_CLAMP; }
            s_dirScore = (s_dirScore * 3) / 4 + c;
        }
        else if (dist_front >= ASW_AUTO_VOTE_CM)
        {
            s_dirScore = (s_dirScore * 3) / 4; /* 개방 주행 + 입구 안 보임 -> 서서히 소거 */
        }
        /* else : 접근 중 센서 사각지대 -> 점수 유지 (직전 입구 기억 보존) */
    }

    /* ---- 비상/코너 판단 ---- */
    if (dist_front <= ASW_AUTO_DIST_EMERGENCY_CM)
    {
        if (dist_left <= ASW_AUTO_DIST_SIDE_BLOCK_CM && dist_right <= ASW_AUTO_DIST_SIDE_BLOCK_CM)
        {
            /* 전방이 임박한 비상 상황 -> 관성으로 미끄러지는 Stop() 대신
             * 급제동으로 즉시 감속(방향반전 완충도 겸함) */
            RTE_Motor_Brake();
            AswAuto_EnterBackingUp();
        }
        else
        {
            /* 아크로도 못 도는 급접근 -> 제자리 피벗 전환.
             * 전방이 임박했으므로 급제동으로 즉시 감속(방향반전 완충도 겸함) 후
             * 다음 주기에 HandlePivot()이 실제 피벗 명령을 낸다. */
            AswAutoTurnDir_t dir = AswAuto_DecideDir(dist_left, dist_right);
            AswAuto_EnterTurnState(ASW_AUTO_STATE_PIVOT, dir);
            RTE_Motor_Brake();
        }
        return;
    }

    uint8_t creep = 0u;
    if (dist_front <= ASW_AUTO_DIST_TURN_ENTER_CM)
    {
        /* 방향 근거가 있는가? (연속 코너 잠금 / 확실한 개방 / 적분 투표) */
        int32_t gap = (int32_t)dist_left - (int32_t)dist_right;
        if (gap < 0) { gap = -gap; }
        uint8_t known = (s_sinceTurnMs < ASW_AUTO_DIR_LOCK_MS) ||
                        (gap >= ASW_AUTO_GAP_DECISIVE_CM) ||
                        (s_dirScore >= ASW_AUTO_VOTE_TH) ||
                        (s_dirScore <= -ASW_AUTO_VOTE_TH);

        if (known != 0u || dist_front <= ASW_AUTO_CREEP_FLOOR_CM)
        {
            /* 코너 확정 -> 멈추지 않고 아크 선회 시작
             * (근거 없이 CREEP_FLOOR까지 온 경우엔 그 시점 최선의 정보로 강제) */
            AswAutoTurnDir_t dir = AswAuto_DecideDir(dist_left, dist_right);
            AswAuto_EnterTurnState(ASW_AUTO_STATE_ARC_TURN, dir);
            AswAuto_DriveArc(dir);
            return;
        }
        /* 방향 미확정(대각선 박스 등으로 전방만 먼저 막힘) ->
         * 회전을 잠그지 말고 저속 직진하며 한쪽이 열리기를 기다림 */
        creep = 1u;
    }

    /* ---- 전방 거리 비례 목표 속도 (TURN_ENTER~SPEED_FULL 선형 보간) ---- */
    uint32_t base;
    if (creep != 0u)
    {
        base = ASW_AUTO_CREEP_DUTY;
    }
    else if (dist_front >= ASW_AUTO_SPEED_FULL_CM)
    {
        base = ASW_AUTO_DRIVE_BASE_DUTY;
    }
    else
    {
        uint32_t span  = ASW_AUTO_SPEED_FULL_CM - ASW_AUTO_DIST_TURN_ENTER_CM;
        uint32_t ahead = dist_front - ASW_AUTO_DIST_TURN_ENTER_CM;
        base = ASW_AUTO_DRIVE_SLOW_DUTY
             + ((ASW_AUTO_DRIVE_BASE_DUTY - ASW_AUTO_DRIVE_SLOW_DUTY) * ahead) / span;
    }

    /* ---- PD 센터링 ---- */
    uint32_t l = (dist_left  > ASW_AUTO_CENTER_TRUST_CM) ? ASW_AUTO_CENTER_TRUST_CM : dist_left;
    uint32_t r = (dist_right > ASW_AUTO_CENTER_TRUST_CM) ? ASW_AUTO_CENTER_TRUST_CM : dist_right;

    int32_t diff = (int32_t)l - (int32_t)r; /* >0 : 좌측이 넓음 -> 좌로 조향 */

    int32_t pTerm = 0;
    if (diff > ASW_AUTO_CENTER_DEADBAND_CM || diff < -ASW_AUTO_CENTER_DEADBAND_CM)
    {
        pTerm = diff * (int32_t)ASW_AUTO_CENTER_KP;
    }

    int32_t dTerm = 0;
    if (s_prevDiffValid != 0u)
    {
        int32_t dDiff = diff - s_prevDiff;
        /* 벽 모서리 통과 등으로 인한 비물리적 점프는 D항에서 배제 */
        if (dDiff >  ASW_AUTO_CENTER_DDIFF_MAX) { dDiff =  ASW_AUTO_CENTER_DDIFF_MAX; }
        if (dDiff < -ASW_AUTO_CENTER_DDIFF_MAX) { dDiff = -ASW_AUTO_CENTER_DDIFF_MAX; }
        dTerm = dDiff * (int32_t)ASW_AUTO_CENTER_KD;
    }
    s_prevDiff      = diff;
    s_prevDiffValid = 1u;

    int32_t correction = pTerm + dTerm;
    if (correction >  (int32_t)ASW_AUTO_CENTER_MAX_CORR) { correction =  (int32_t)ASW_AUTO_CENTER_MAX_CORR; }
    if (correction < -(int32_t)ASW_AUTO_CENTER_MAX_CORR) { correction = -(int32_t)ASW_AUTO_CENTER_MAX_CORR; }

    /* 조향 슬루 제한 : 보정값 자체도 사이클당 변화 상한 적용 (채찍질 방지) */
    if (correction > s_curCorr + ASW_AUTO_STEER_SLEW)      { correction = s_curCorr + ASW_AUTO_STEER_SLEW; }
    else if (correction < s_curCorr - ASW_AUTO_STEER_SLEW) { correction = s_curCorr - ASW_AUTO_STEER_SLEW; }
    s_curCorr = correction;

    int32_t leftDuty  = (int32_t)base - correction;
    int32_t rightDuty = (int32_t)base + correction;

    if (leftDuty  < (int32_t)ASW_AUTO_DUTY_MIN)   { leftDuty  = (int32_t)ASW_AUTO_DUTY_MIN; }
    if (rightDuty < (int32_t)ASW_AUTO_DUTY_MIN)   { rightDuty = (int32_t)ASW_AUTO_DUTY_MIN; }
    if (leftDuty  > (int32_t)ECU_L298N_MAX_DUTY)  { leftDuty  = (int32_t)ECU_L298N_MAX_DUTY; }
    if (rightDuty > (int32_t)ECU_L298N_MAX_DUTY)  { rightDuty = (int32_t)ECU_L298N_MAX_DUTY; }

    /* ---- 슬루 제한 후 출력 ---- */
    s_curLeftDuty  = AswAuto_SlewLimit(s_curLeftDuty,  (uint32_t)leftDuty);
    s_curRightDuty = AswAuto_SlewLimit(s_curRightDuty, (uint32_t)rightDuty);

    RTE_Motor_DriveForwardDifferential((uint16_t)s_curLeftDuty, (uint16_t)s_curRightDuty);
}

/* ------------------------------------------------------------------------
 * ARC_TURN 상태 처리 : 무정지 이동 선회
 * ------------------------------------------------------------------------ */
static void AswAuto_HandleArcTurn(uint32_t dist_left, uint32_t dist_front, uint32_t dist_right)
{
    s_turnElapsedMs += ASW_AUTO_CTRL_PERIOD_MS;

    if (dist_front <= ASW_AUTO_DIST_EMERGENCY_CM)
    {
        /* 아크 중 급접근 -> 같은 방향 제자리 피벗으로 강화 (방향 유지가 U턴 방지).
         * 전방이 임박했으므로 급제동으로 즉시 감속(방향반전 완충도 겸함) 후
         * 다음 주기에 피벗을 시작한다. */
        AswAuto_EnterTurnState(ASW_AUTO_STATE_PIVOT, s_turnDir);
        RTE_Motor_Brake();
        return;
    }

    if (dist_front > ASW_AUTO_ARC_EXIT_CM) { s_openCount++; }
    else                                   { s_openCount = 0u; }

    if (s_turnElapsedMs >= ASW_AUTO_ARC_MIN_MS &&
        s_openCount     >= ASW_AUTO_EXIT_CONFIRM)
    {
        /* 코너 통과 -> 무정지로 직선 주행 복귀 */
        AswAuto_EnterDrive();
        AswAuto_HandleDrive(dist_left, dist_front, dist_right);
        return;
    }

    if (s_turnElapsedMs >= ASW_AUTO_ARC_MAX_MS)
    {
        /* 아크로 못 도는 좁은 코너 -> 같은 방향 제자리 피벗으로 강화.
         * 바퀴 회전방향 반전 전 한 사이클 정지로 완충(다음 주기에 피벗 시작) */
        AswAuto_EnterTurnState(ASW_AUTO_STATE_PIVOT, s_turnDir);
        RTE_Motor_Stop();
        return;
    }

    AswAuto_DriveArc(s_turnDir);
}

/* ------------------------------------------------------------------------
 * PIVOT 상태 처리 : 비상 제자리 피벗 (연속 회전)
 * ------------------------------------------------------------------------ */
static void AswAuto_HandlePivot(uint32_t dist_front)
{
    s_turnElapsedMs += ASW_AUTO_CTRL_PERIOD_MS;

    if (s_turnElapsedMs >= ASW_AUTO_PIVOT_MAX_MS)
    {
        /* 회전으로 못 벗어남(갇힘) -> 후진 회복 */
        RTE_Motor_Stop();
        AswAuto_EnterBackingUp();
        return;
    }

    if (dist_front > ASW_AUTO_PIVOT_EXIT_CM) { s_openCount++; }
    else                                     { s_openCount = 0u; }

    if (s_turnElapsedMs >= ASW_AUTO_PIVOT_MIN_MS &&
        s_openCount     >= ASW_AUTO_EXIT_CONFIRM)
    {
        /* 피벗(한쪽 후진) -> 전진(양쪽 전진) 복귀도 바퀴 회전방향이 반전되므로
         * 아크 복귀(무정지)와 달리 한 사이클 정지로 완충 후 다음 주기에 전진한다. */
        RTE_Motor_Stop();
        AswAuto_EnterDrive();
        return;
    }

    /* 과회전(U턴) 방지 : 예상 회전시간을 넘기고도 EXIT_CM 조건을 못 채웠다면
     * 그 이상 계속 돌면 왔던 길(반대편)까지 만나 U턴할 위험이 크므로,
     * 더 느슨한 기준(WARNING_CM)만 만족해도 즉시 복귀시킨다.
     * (자이로가 없어 정확한 각도 측정은 불가 - EXPECT_MS는 경험적 추정치이며
     *  실제 회전 속도(PIVOT_DUTY)에 따라 실측 보정이 필요할 수 있다) */
    if (s_turnElapsedMs >= ASW_AUTO_PIVOT_EXPECT_MS && dist_front > ASW_AUTO_DIST_WARNING_CM)
    {
        /* 위와 동일한 이유로 정지 완충 후 다음 주기에 전진 복귀 */
        RTE_Motor_Stop();
        AswAuto_EnterDrive();
        return;
    }

    if (s_turnDir == ASW_AUTO_TURN_RIGHT)
    {
        RTE_Motor_PivotRight(ASW_AUTO_PIVOT_DUTY);
    }
    else
    {
        RTE_Motor_PivotLeft(ASW_AUTO_PIVOT_DUTY);
    }
}

/* ------------------------------------------------------------------------
 * BACKING_UP 상태 처리 : 연속 후진 + 매 사이클 전방 재평가
 * ------------------------------------------------------------------------ */
static void AswAuto_HandleBackingUp(uint32_t dist_left, uint32_t dist_front, uint32_t dist_right)
{
    if (dist_front > ASW_AUTO_DIST_WARNING_CM)
    {
        RTE_Motor_Stop(); /* 전진 복귀 전 한 사이클 완충 */
        AswAuto_EnterDrive();
        return;
    }

    /* 한 스텝 후진 완료 시점마다 회전 여유가 생겼는지 확인 - MAX_RETRY까지
     * 다 채우고서야 회전하면 그 사이 정지-후진을 반복해 "가다서다"처럼
     * 보이므로, 회전 공간이 생기는 즉시 넘어간다 */
    if (s_backupElapsedMs > 0u && (s_backupElapsedMs % ASW_AUTO_BACKUP_STEP_MS) == 0u)
    {
        if (dist_left > ASW_AUTO_DIST_SIDE_BLOCK_CM || dist_right > ASW_AUTO_DIST_SIDE_BLOCK_CM)
        {
            RTE_Motor_Stop();
            AswAuto_EnterTurnState(ASW_AUTO_STATE_PIVOT,
                                   (dist_right >= dist_left) ? ASW_AUTO_TURN_RIGHT
                                                              : ASW_AUTO_TURN_LEFT);
            return;
        }
    }

    if (s_backupElapsedMs >= (ASW_AUTO_BACKUP_STEP_MS * ASW_AUTO_BACKUP_MAX_RETRY))
    {
        /* 누적 후진에도 회복 실패 -> 반대 방향 피벗 재시도 */
        RTE_Motor_Stop();
        AswAuto_EnterTurnState(ASW_AUTO_STATE_PIVOT,
                               (s_turnDir == ASW_AUTO_TURN_RIGHT) ? ASW_AUTO_TURN_LEFT
                                                                  : ASW_AUTO_TURN_RIGHT);
        return;
    }

    RTE_Motor_DriveBackward(ASW_AUTO_BACKUP_DUTY);
    s_backupElapsedMs += ASW_AUTO_CTRL_PERIOD_MS;
}

/*
 * ASW_Autonomous_ProcessControl : CtrlTask 주기(50ms) 호출 (RTE_DRIVE_MODE_AUTO일 때만)
 *   dist_left/front/right : rte_sensor(ecu_hcsr04)에서 읽어온 최신 거리값(cm)
 */
void ASW_Autonomous_ProcessControl(uint32_t dist_left,
                                 uint32_t dist_front,
                                 uint32_t dist_right)
{
    /* ---- 센서 EMA 필터링 (모든 상태 공통) ---- */
    if (s_filtInit == 0u)
    {
        s_fLeft  = dist_left;
        s_fFront = dist_front;
        s_fRight = dist_right;
        s_filtInit = 1u;
    }
    else
    {
        s_fLeft  = AswAuto_Ema(s_fLeft,  dist_left,  ASW_AUTO_FILT_SIDE_NEW_W);
        s_fFront = AswAuto_Ema(s_fFront, dist_front, ASW_AUTO_FILT_FRONT_NEW_W);
        s_fRight = AswAuto_Ema(s_fRight, dist_right, ASW_AUTO_FILT_SIDE_NEW_W);
    }

    /* ---- 진단 로그 (읽기 전용 : 상태/방향을 절대 변경하지 않는다) ----
     * 50ms마다 찍으면 너무 빨라 터미널이 밀리므로 5사이클(=250ms)에 1회만 출력.
     * 필터값(fL/fF/fR)과 원본(raw front)을 같이 봐야 트리거 순환(195ms) 때문에
     * 전방값이 얼마나 오래 정체되는지 확인할 수 있다. */
    {
        static uint32_t s_logDiv = 0u;
        if (++s_logDiv >= 5u)
        {
            s_logDiv = 0u;
            printf("[AUTO] st=%d fL=%lu fF=%lu fR=%lu rawF=%lu turn=%d vote=%ld\r\n",
                   (int)s_state,
                   (unsigned long)s_fLeft, (unsigned long)s_fFront, (unsigned long)s_fRight,
                   (unsigned long)dist_front,
                   (int)s_turnDir, (long)s_dirScore);
        }
    }

    switch (s_state)
    {
    case ASW_AUTO_STATE_ARC_TURN:
        AswAuto_HandleArcTurn(s_fLeft, s_fFront, s_fRight);
        break;

    case ASW_AUTO_STATE_PIVOT:
        AswAuto_HandlePivot(s_fFront);
        break;

    case ASW_AUTO_STATE_BACKING_UP:
        AswAuto_HandleBackingUp(s_fLeft, s_fFront, s_fRight);
        break;

    case ASW_AUTO_STATE_DRIVE:
    default:
        s_state = ASW_AUTO_STATE_DRIVE; /* 알 수 없는 값 방어적으로 복구 */
        AswAuto_HandleDrive(s_fLeft, s_fFront, s_fRight);
        break;
    }
}
