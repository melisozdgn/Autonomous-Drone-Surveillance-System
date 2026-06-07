/**
 * @file    sensor_manager.c
 * @brief   FreeRTOS Sensor Manager Task — 200 Hz
 *
 * Reads all sensors, fuses data, pushes to xSensorDataQueue.
 * Sensor fusion order:
 *   1. IMU  (MPU-6050) — 200 Hz via I2C + complementary filter
 *   2. Baro (BMP280)   — 50 Hz  via I2C
 *   3. GPS  (NEO-M8N)  — 5-10 Hz interrupt-driven
 *   4. Altitude fusion — weighted average baro + GPS
 */

#include "sensor_manager.h"
#include "imu_driver.h"
#include "baro_driver.h"
#include "gps_driver.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

static I2C_HandleTypeDef *s_hi2c1 = NULL;
static I2C_HandleTypeDef *s_hi2c2 = NULL;
static SPI_HandleTypeDef *s_hspi  = NULL;

/* Low-pass filter for altitude fusion */
#define ALT_ALPHA   0.95f
static float s_fused_alt = 0.0f;

void SENSOR_Init(I2C_HandleTypeDef *hi2c1,
                 I2C_HandleTypeDef *hi2c2,
                 SPI_HandleTypeDef *hspi)
{
    s_hi2c1 = hi2c1;
    s_hi2c2 = hi2c2;
    s_hspi  = hspi;

    IMU_Init(hi2c1);
    BARO_Init(hi2c2);
}

void vSensorManagerTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t   last_wake  = xTaskGetTickCount();
    SensorData_t data       = {0};
    GPSData_t    gps        = {0};
    uint8_t      baro_count = 0;
    float        baro_alt   = 0.0f;

    for (;;) {
        /* 200 Hz — every 5 ms */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));

        /* ── IMU (every cycle) ── */
        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            IMU_Read(s_hi2c1, &data);
            xSemaphoreGive(xI2CMutex);
        }

        /* ── Barometer (every 4 cycles = 50 Hz) ── */
        if (++baro_count >= 4) {
            baro_count = 0;
            float p, t, h;
            if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                BARO_Read(s_hi2c2, &p, &t, &h);
                xSemaphoreGive(xI2CMutex);
            }
            data.pressure       = p;
            data.temperature    = t;
            baro_alt            = h;
        }

        /* ── GPS (non-blocking read) ── */
        GPS_GetData(&gps);
        if (gps.valid) {
            data.latitude      = gps.latitude;
            data.longitude     = gps.longitude;
            data.altitude_gps  = gps.altitude;
            data.speed_gps     = gps.speed_mps;
            data.heading_gps   = gps.heading;
            data.gps_fix       = gps.fix_type;
            data.gps_satellites = gps.satellites;
        }

        /* ── Altitude fusion: baro (high rate) + GPS (low rate) ── */
        /* Weight: 95% baro + 5% GPS when GPS valid */
        if (gps.valid && gps.fix_type >= 3) {
            s_fused_alt = ALT_ALPHA * baro_alt
                        + (1.0f - ALT_ALPHA) * gps.altitude;
        } else {
            s_fused_alt = baro_alt;
        }
        data.altitude_baro = s_fused_alt;

        data.timestamp_ms = HAL_GetTick();

        /* Push to queue (overwrite if full — newest data wins) */
        xQueueOverwrite(xSensorDataQueue, &data);
    }
}
