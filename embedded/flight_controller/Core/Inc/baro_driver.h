/**
 * @file    baro_driver.h
 * @brief   BMP280 Barometer driver
 */
#ifndef __BARO_DRIVER_H
#define __BARO_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef BARO_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BARO_Read(I2C_HandleTypeDef *hi2c,
                              float *pressure_hpa,
                              float *temp_c,
                              float *altitude_m);
#endif
