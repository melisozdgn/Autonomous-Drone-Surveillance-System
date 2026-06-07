/**
 * @file    flight_controller.h
 * @brief   Flight Controller — public API
 */
#ifndef __FLIGHT_CONTROLLER_H
#define __FLIGHT_CONTROLLER_H

#include "main.h"

typedef struct {
    double  latitude;
    double  longitude;
    float   altitude;
    float   speed;
} Waypoint_t;

typedef struct {
    Waypoint_t wp[16];
    uint8_t    count;
} WaypointList_t;

void         FC_Init(void);
DroneState_t FC_GetState(void);
float        FC_GetAltSP(void);
uint32_t     FC_GetFlightTime(void);

void vFlightControlTask(void *pvParameters);

#endif
