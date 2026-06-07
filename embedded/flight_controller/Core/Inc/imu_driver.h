/**
 * @file    imu_driver.h
 * @brief   MPU-6050 IMU driver
 */
#ifndef __IMU_DRIVER_H
#define __IMU_DRIVER_H

#include "stm32f4xx_hal.h"
#include "main.h"

HAL_StatusTypeDef IMU_Init(I2C_HandleTypeDef *hi2c);
void              IMU_CalibrateGyro(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef IMU_Read(I2C_HandleTypeDef *hi2c, SensorData_t *out);

#endif
