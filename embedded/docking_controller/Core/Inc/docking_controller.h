/**
 * @file    docking_controller.h
 */
#ifndef __DOCKING_CONTROLLER_H
#define __DOCKING_CONTROLLER_H
#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdint.h>
void    DOCK_Init(TIM_HandleTypeDef *htim);
uint8_t DOCK_Run(const SensorData_t *sensor,
                  float *vx_cmd, float *vy_cmd, float *vz_cmd);
void    DOCK_OnArUcoFrame(uint8_t found, uint8_t id,
                           int16_t cx, int16_t cy, float dist);
float   DOCK_GetXError(void);
float   DOCK_GetYError(void);
float   DOCK_GetLaserAlt(void);
uint8_t DOCK_MarkerFound(void);
#endif
