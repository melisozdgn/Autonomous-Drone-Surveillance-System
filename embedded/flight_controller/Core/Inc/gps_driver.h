/**
 * @file    gps_driver.h
 * @brief   NEO-M8N GPS — NMEA/UBX parser
 */
#ifndef __GPS_DRIVER_H
#define __GPS_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    double  latitude;
    double  longitude;
    float   altitude;
    float   speed_mps;
    float   heading;
    uint8_t fix_type;   /* 0=no fix, 2=2D, 3=3D */
    uint8_t satellites;
    uint8_t valid;
} GPSData_t;

void GPS_Init(UART_HandleTypeDef *huart);
void GPS_Update(void);
void GPS_GetData(GPSData_t *out);
void vGPSTask(void *pvParameters);

#endif
