/**
 * @file  vl53l1x.h
 * @brief VL53L1X Laser Rangefinder — I2C driver (0-4m, ±1mm)
 */
#ifndef __VL53L1X_H
#define __VL53L1X_H
#include "stm32f4xx_hal.h"
#include <stdint.h>
HAL_StatusTypeDef VL53L1X_Init(I2C_HandleTypeDef *hi2c);
float             VL53L1X_ReadDistance_m(I2C_HandleTypeDef *hi2c);
uint8_t           VL53L1X_DataReady(I2C_HandleTypeDef *hi2c);
#endif
