/**
 * @file    power_manager.h
 */
#ifndef __POWER_MANAGER_H
#define __POWER_MANAGER_H
#include "stm32f4xx_hal.h"
#include <stdint.h>
void    POWER_Init(ADC_HandleTypeDef *hadc);
void    POWER_Update(void);
float   POWER_GetVoltage(void);
float   POWER_GetCurrent(void);
float   POWER_GetSoC(void);
float   POWER_GetTemperature(void);
uint8_t POWER_IsLowBattery(void);
uint8_t POWER_IsCritical(void);
void    vPowerManagerTask(void *pvParameters);
#endif
