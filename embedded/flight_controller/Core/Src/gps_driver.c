/**
 * @file    gps_driver.c
 * @brief   NEO-M8N GPS Driver — NMEA GPGGA/GPRMC Parser
 *
 * Parses GPGGA sentence for lat/lon/alt/fix
 * Parses GPRMC sentence for speed/heading
 * DMA UART receive → ring buffer → parser task
 */

#include "gps_driver.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define GPS_BUF_SIZE    256
#define NMEA_MAX_LEN    100

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t  s_rx_byte;
static char     s_nmea_buf[NMEA_MAX_LEN];
static uint8_t  s_nmea_idx = 0;
static GPSData_t s_gps_data = {0};

/* ── NMEA helpers ────────────────────────────────────────────── */
static double nmea_to_degrees(const char *raw, char dir)
{
    if (!raw || raw[0] == '\0') return 0.0;
    double raw_d = atof(raw);
    int    deg   = (int)(raw_d / 100);
    double min   = raw_d - deg * 100.0;
    double result = deg + min / 60.0;
    if (dir == 'S' || dir == 'W') result = -result;
    return result;
}

static uint8_t nmea_checksum(const char *sentence)
{
    uint8_t cs = 0;
    for (const char *p = sentence + 1; *p && *p != '*'; p++)
        cs ^= (uint8_t)*p;
    return cs;
}

/* ── GPGGA parser ────────────────────────────────────────────── */
/* $GPGGA,hhmmss.ss,Latitude,N,Longitude,E,FS,NoSV,HDOP,msl,M,,,,*cs */
static void parse_gpgga(const char *sentence)
{
    char buf[NMEA_MAX_LEN];
    strncpy(buf, sentence, NMEA_MAX_LEN - 1);

    char *tok[15] = {NULL};
    char *p = buf;
    int   n = 0;
    while ((tok[n] = strsep(&p, ",")) != NULL && n < 14) n++;

    if (n < 10) return;

    s_gps_data.latitude    = nmea_to_degrees(tok[2], tok[3][0]);
    s_gps_data.longitude   = nmea_to_degrees(tok[4], tok[5][0]);
    s_gps_data.fix_type    = (uint8_t)atoi(tok[6]);
    s_gps_data.satellites  = (uint8_t)atoi(tok[7]);
    s_gps_data.altitude    = (float)atof(tok[9]);
    s_gps_data.valid       = (s_gps_data.fix_type > 0) ? 1 : 0;
}

/* ── GPRMC parser ────────────────────────────────────────────── */
/* $GPRMC,hhmmss,A,Lat,N,Lon,E,speed_kn,course,date,,,*cs */
static void parse_gprmc(const char *sentence)
{
    char buf[NMEA_MAX_LEN];
    strncpy(buf, sentence, NMEA_MAX_LEN - 1);

    char *tok[13] = {NULL};
    char *p = buf;
    int   n = 0;
    while ((tok[n] = strsep(&p, ",")) != NULL && n < 12) n++;

    if (n < 9) return;
    /* Speed knots → m/s */
    s_gps_data.speed_mps = (float)atof(tok[7]) * 0.514444f;
    s_gps_data.heading   = (float)atof(tok[8]);
}

/* ── NMEA line handler ───────────────────────────────────────── */
static void process_nmea_line(const char *line)
{
    /* Verify checksum */
    const char *star = strchr(line, '*');
    if (!star) return;
    uint8_t got = nmea_checksum(line);
    uint8_t exp = (uint8_t)strtol(star + 1, NULL, 16);
    if (got != exp) return;

    if (strncmp(line, "$GPGGA", 6) == 0 ||
        strncmp(line, "$GNGGA", 6) == 0)
        parse_gpgga(line);
    else if (strncmp(line, "$GPRMC", 6) == 0 ||
             strncmp(line, "$GNRMC", 6) == 0)
        parse_gprmc(line);
}

/* ── HAL UART RX complete callback ──────────────────────────── */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    char c = (char)s_rx_byte;
    if (c == '\n') {
        s_nmea_buf[s_nmea_idx] = '\0';
        process_nmea_line(s_nmea_buf);
        s_nmea_idx = 0;
    } else if (c != '\r' && s_nmea_idx < NMEA_MAX_LEN - 1) {
        s_nmea_buf[s_nmea_idx++] = c;
    }

    /* Re-arm DMA receive */
    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

void GPS_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    memset(&s_gps_data, 0, sizeof(s_gps_data));

    /* Start interrupt-driven receive */
    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

void GPS_GetData(GPSData_t *out)
{
    if (out) *out = s_gps_data;
}

void vGPSTask(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        /* GPS is interrupt-driven; this task does health monitoring */
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (!s_gps_data.valid) {
            /* GPS not fixed — LED indicator etc. */
            HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        }
    }
}
