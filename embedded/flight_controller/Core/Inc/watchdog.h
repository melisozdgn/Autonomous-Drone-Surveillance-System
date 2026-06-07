/**
 * @file    watchdog.h
 */
#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#include "stm32f4xx_hal.h"

void WDG_Init(IWDG_HandleTypeDef *hiwdg);
void WDG_Kick(void);
void vWatchdogTask(void *pvParameters);

#endif
