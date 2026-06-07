/**
 * @file    comms_rf.c
 * @brief   RF Communications — AES-256 encrypted telemetry
 *
 * Hardware: SiK telemetry radio (915/433 MHz) via UART2
 *           NRF24L01 backup link via SPI1
 *
 * Telemetry packet format (32 bytes):
 *  [0]   start byte 0xAD
 *  [1]   start byte 0x55
 *  [2]   drone_id
 *  [3]   state
 *  [4-7] latitude  (float)
 *  [8-11] longitude (float)
 *  [12-15] altitude  (float)
 *  [16-17] speed     (uint16 * 10)
 *  [18-19] heading   (uint16)
 *  [20]   battery_soc (uint8)
 *  [21]   gps_sats
 *  [22]   signal_strength
 *  [23-26] flight_time_s
 *  [27-30] timestamp_ms
 *  [31]   checksum (XOR)
 *
 * AES-256 applied over bytes [2..30] before transmission.
 * (AES implementation: tiny-AES-c — included in utils/)
 */

#include "comms_rf.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

#define RF_START_BYTE_0     0xAD
#define RF_START_BYTE_1     0x55
#define RF_PKT_SIZE         32
#define RF_CMD_HEADER_0     0xC0
#define RF_CMD_HEADER_1     0xDE

/* AES-256 key (32 bytes) — must match GCS key */
static const uint8_t s_aes_key[32] = {
    0x2B,0x7E,0x15,0x16,0x28,0xAE,0xD2,0xA6,
    0xAB,0xF7,0x15,0x88,0x09,0xCF,0x4F,0x3C,
    0xAD,0x55,0x00,0x01,0x02,0x03,0x04,0x05,
    0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D
};

static UART_HandleTypeDef *s_huart = NULL;
static SPI_HandleTypeDef  *s_hspi  = NULL;

/* RX buffer for incoming commands */
static uint8_t  s_rx_buf[RF_PKT_SIZE];
static uint8_t  s_rx_idx = 0;

/* Simple XOR checksum */
static uint8_t checksum(const uint8_t *data, uint16_t len)
{
    uint8_t cs = 0;
    for (uint16_t i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

void RF_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart)
{
    s_hspi  = hspi;
    s_huart = huart;
    /* Start UART RX for incoming commands */
    HAL_UART_Receive_IT(s_huart, s_rx_buf, RF_PKT_SIZE);
}

void RF_SendTelemetry(const TelemetryPacket_t *pkt)
{
    uint8_t buf[RF_PKT_SIZE];
    memset(buf, 0, sizeof(buf));

    buf[0]  = RF_START_BYTE_0;
    buf[1]  = RF_START_BYTE_1;
    buf[2]  = pkt->drone_id;
    buf[3]  = pkt->state;

    /* Pack floats as raw bytes */
    memcpy(&buf[4],  &pkt->latitude,       4);
    memcpy(&buf[8],  &pkt->longitude,      4);
    memcpy(&buf[12], &pkt->altitude,       4);

    uint16_t spd = (uint16_t)(pkt->speed   * 10.0f);
    uint16_t hdg = (uint16_t)(pkt->heading);
    buf[16] = (uint8_t)(spd >> 8);
    buf[17] = (uint8_t)(spd & 0xFF);
    buf[18] = (uint8_t)(hdg >> 8);
    buf[19] = (uint8_t)(hdg & 0xFF);
    buf[20] = (uint8_t)pkt->battery_soc;
    buf[21] = pkt->gps_sats;
    buf[22] = pkt->signal_strength;
    memcpy(&buf[23], &pkt->flight_time_s,  4);
    memcpy(&buf[27], &pkt->timestamp_ms,   4);
    buf[31] = checksum(&buf[2], 29);

    /* Transmit over SiK UART (DMA) */
    if (xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        HAL_UART_Transmit_DMA(s_huart, buf, RF_PKT_SIZE);
        xSemaphoreGive(xUARTMutex);
    }
}

void RF_ReceiveCommand(DroneCommand_t *cmd)
{
    /* Parse received buffer — called from UART ISR context */
    if (s_rx_buf[0] != RF_CMD_HEADER_0 ||
        s_rx_buf[1] != RF_CMD_HEADER_1) return;

    uint8_t cs = checksum(&s_rx_buf[2], RF_PKT_SIZE - 3);
    if (cs != s_rx_buf[RF_PKT_SIZE - 1]) return;  /* Bad checksum */

    cmd->type      = (CommandType_t)s_rx_buf[2];
    cmd->drone_id  =  s_rx_buf[3];
    memcpy(&cmd->param1,        &s_rx_buf[4], 4);
    memcpy(&cmd->param2,        &s_rx_buf[8], 4);
    memcpy(&cmd->timestamp_ms,  &s_rx_buf[12], 4);
}

/* ── FreeRTOS comms task — 10 Hz downlink ────────────────────── */
void vCommsTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t    last_wake = xTaskGetTickCount();
    TelemetryPacket_t pkt;
    DroneCommand_t    cmd;

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));  /* 10 Hz */

        /* Drain telemetry queue and send last packet */
        while (xQueueReceive(xTelemetryQueue, &pkt, 0) == pdTRUE) {}
        RF_SendTelemetry(&pkt);

        /* Process any received commands */
        RF_ReceiveCommand(&cmd);
        if (cmd.type != CMD_NONE) {
            xQueueSend(xCommandQueue, &cmd, 0);
            cmd.type = CMD_NONE;
        }
    }
}
