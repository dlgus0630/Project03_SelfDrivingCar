/*
 * rte_sensor.c
 *  Layer  : RTE (Runtime Environment)
 *  Module : RTE_SENSOR
 *  Desc   : ECU_HCSR04 초음파 드라이버로의 1:1 pass-through 구현.
 */
#include "rte_sensor.h"

void RTE_Sensor_Init(void)
{
    ECU_HCSR04_Init();
}

void RTE_Sensor_TriggerSensor(EcuHcsr04Index_t sensor)
{
    ECU_HCSR04_TriggerSensor(sensor);
}

void RTE_Sensor_TriggerAll(void)
{
    ECU_HCSR04_TriggerAllSensors();
}

uint32_t RTE_Sensor_GetDistance(EcuHcsr04Index_t sensor)
{
    return ECU_HCSR04_GetDistanceCm(sensor);
}

uint8_t RTE_Sensor_IsReady(EcuHcsr04Index_t sensor)
{
    return ECU_HCSR04_IsReady(sensor);
}

void RTE_Sensor_HandleCaptureIsr(TIM_HandleTypeDef *p_htim)
{
    ECU_HCSR04_HandleCaptureIsr(p_htim);
}
