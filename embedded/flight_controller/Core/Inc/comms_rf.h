/**
 * @file    comms_rf.h
 * @brief   AES-256 RF Communications — NRF24L01 + SiK
 */
#ifndef __COMMS_RF_H
#define __COMMS_RF_H

#include "stm32f4xx_hal.h"
#include "main.h"

void RF_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart);
void RF_SendTelemetry(const TelemetryPacket_t *pkt);
void RF_ReceiveCommand(DroneCommand_t *cmd);
void vCommsTask(void *pvParameters);

#endif
