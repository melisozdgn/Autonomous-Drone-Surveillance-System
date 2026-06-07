/**
 * @file  mems_mic.h
 * @brief 4x MEMS Microphone Array — I2S / PDM driver
 */
#ifndef __MEMS_MIC_H
#define __MEMS_MIC_H
#include "stm32f4xx_hal.h"
#include <stdint.h>
#define MIC_COUNT        4
#define MIC_SAMPLE_RATE  8000
#define MIC_FRAME_SIZE   256
HAL_StatusTypeDef MEMS_MIC_Init(I2S_HandleTypeDef *hi2s);
uint8_t           MEMS_MIC_ReadFrame(int16_t out[MIC_COUNT][MIC_FRAME_SIZE]);
float             MEMS_MIC_GetRMS(uint8_t mic_id);
#endif
