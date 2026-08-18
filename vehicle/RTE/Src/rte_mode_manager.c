/*
 * rte_mode_manager.c
 *  Layer  : RTE
 *  Module : RTE_MODE_MANAGER
 */
#include "rte_mode_manager.h"

static RteDriveMode_t s_driveMode = RTE_DRIVE_MODE_MANUAL; /* 기본값: 전원 인가 시 휴대폰 수동조작 대기 (자동 시작 방지) */

osMutexId g_rteModeMutexHandle = NULL;
osMutexDef(driveModeMutex);

void RTE_Mode_Init(void)
{
    g_rteModeMutexHandle = osMutexCreate(osMutex(driveModeMutex));
    if (g_rteModeMutexHandle == NULL)
    {
        Error_Handler();
    }
    s_driveMode = RTE_DRIVE_MODE_MANUAL;
}

void RTE_Mode_SetDriveMode(RteDriveMode_t mode)
{
    osMutexWait(g_rteModeMutexHandle, osWaitForever);
    s_driveMode = mode;
    osMutexRelease(g_rteModeMutexHandle);
}

RteDriveMode_t RTE_Mode_GetDriveMode(void)
{
    RteDriveMode_t mode;
    osMutexWait(g_rteModeMutexHandle, osWaitForever);
    mode = s_driveMode;
    osMutexRelease(g_rteModeMutexHandle);
    return mode;
}
