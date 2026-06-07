/**
 * @file  flir_lepton.h
 * @brief FLIR Lepton 3.5 — 160x120 thermal camera driver (VoSPI/SPI2 + CCI/I2C)
 */
#ifndef __FLIR_LEPTON_H
#define __FLIR_LEPTON_H
#include "stm32f4xx_hal.h"
#include <stdint.h>
#define FLIR_COLS 160
#define FLIR_ROWS 120
void     FLIR_Init(SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c);
uint8_t  FLIR_CaptureFrame(void);
uint8_t  FLIR_FrameReady(void);
uint16_t FLIR_GetPixel(uint8_t row, uint8_t col);
const uint16_t (*FLIR_GetFrame(void))[FLIR_COLS];
uint8_t  FLIR_FindHotspot(uint16_t threshold, uint8_t *cx, uint8_t *cy, uint16_t *peak_val);
#endif
