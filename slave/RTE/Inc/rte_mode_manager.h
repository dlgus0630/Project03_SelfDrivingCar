/*
 * rte_mode_manager.h
 *
 *  Layer  : RTE (Runtime Environment)
 *  Module : RTE_MODE_MANAGER
 *  Desc   : FreeRTOS 태스크 총괄 관리 및 전역 주행 모드(수동/자동)
 *           상태 관리. Mutex로 보호되는 전역 상태를 통해
 *           asw_manual_control / asw_autonomous 간 모드를 중재한다.
 *
 *  Naming Rule : RTE_Mode_<Verb><Object>()
 */

#ifndef __RTE_MODE_MANAGER_H
#define __RTE_MODE_MANAGER_H

#include "main.h"
#include "cmsis_os.h"

/* ------------------------------------------------------------------------
 * 전역 주행 모드
 * ------------------------------------------------------------------------ */
typedef enum {
    RTE_DRIVE_MODE_MANUAL = 0,  /* 기본값: 전원 인가 시 수동 대기 */
    RTE_DRIVE_MODE_AUTO   = 1   /* 마스터 CAN 명령 수신 시 전환 */
} RteDriveMode_t;

/* 참고용 Task 스택 크기 (.ioc / freertos.c 실제 정의와 일치시킬 것) */
#define RTE_TASK_TRIG_STACK_SIZE     128u
#define RTE_TASK_CTRL_STACK_SIZE     256u
#define RTE_TASK_CAN_STACK_SIZE      256u

/* ------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------ */
void            RTE_Mode_Init(void);
void            RTE_Mode_SetDriveMode(RteDriveMode_t mode);
RteDriveMode_t  RTE_Mode_GetDriveMode(void);

extern osMutexId g_rteModeMutexHandle;

#endif /* __RTE_MODE_MANAGER_H */

