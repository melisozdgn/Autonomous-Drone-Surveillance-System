/**
 * @file    sensor_manager.h
 */
#ifndef __SENSOR_MANAGER_H
#define __SENSOR_MANAGER_H

#include "main.h"

void SENSOR_Init(I2C_HandleTypeDef *hi2c1,
                 I2C_HandleTypeDef *hi2c2,
                 SPI_HandleTypeDef *hspi);
void vSensorManagerTask(void *pvParameters);

#endif
