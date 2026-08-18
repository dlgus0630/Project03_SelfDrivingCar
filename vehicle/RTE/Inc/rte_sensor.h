/*
 * rte_sensor.h
 *
 *  Layer  : RTE (Runtime Environment)
 *  Module : RTE_SENSOR
 *  Desc   : ASW <-> ECU_HCSR04 초음파 드라이버 사이의 1:1 브릿지.
 *           ecu_hcsr04.h를 포함해 EcuHcsr04Index_t / ECU_HCSR04_SENSOR_* /
 *           ECU_HCSR04_DIST_MAX_CM 및 ISR 시그니처용 TIM_HandleTypeDef를
 *           상위(ASW)로 재노출한다. (RTE 계층은 HAL 타입 참조 허용)
 *
 *  Naming Rule : RTE_Sensor_<Verb><Object>()
 */
#ifndef __RTE_SENSOR_H
#define __RTE_SENSOR_H

#include "ecu_hcsr04.h"   /* EcuHcsr04Index_t, ECU_HCSR04_*, TIM_HandleTypeDef */

/* ------------------------------------------------------------------------
 * Public API (ECU_HCSR04 1:1 pass-through)
 * ------------------------------------------------------------------------ */
void      RTE_Sensor_Init(void);
void      RTE_Sensor_TriggerSensor(EcuHcsr04Index_t sensor);
void      RTE_Sensor_TriggerAll(void);
uint32_t  RTE_Sensor_GetDistance(EcuHcsr04Index_t sensor);
uint8_t   RTE_Sensor_IsReady(EcuHcsr04Index_t sensor);
void      RTE_Sensor_HandleCaptureIsr(TIM_HandleTypeDef *p_htim);

#endif /* __RTE_SENSOR_H */
