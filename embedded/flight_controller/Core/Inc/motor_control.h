/**
 * @file    motor_control.h
 */
#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

void MOTOR_Init(TIM_HandleTypeDef *htim);
void MOTOR_SetPWM(TIM_HandleTypeDef *htim,
                  int16_t m1, int16_t m2,
                  int16_t m3, int16_t m4);
void MOTOR_SetAll(int16_t us);
void MOTOR_Arm(TIM_HandleTypeDef *htim);
void MOTOR_Disarm(TIM_HandleTypeDef *htim);

#endif
