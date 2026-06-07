/**
 * @file    telemetry_task.c
 * @brief   FreeRTOS Telemetry Task — 10 Hz
 *
 * Collects state from all subsystems, packs TelemetryPacket_t,
 * pushes to xTelemetryQueue for RF transmission.
 */

#include "main.h"
#include "flight_controller.h"
#include "power_manager.h"
#include "gps_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void vTelemetryTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t    last_wake = xTaskGetTickCount();
    SensorData_t  sensor    = {0};
    TelemetryPacket_t pkt   = {0};

    static uint32_t seq = 0;

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));  /* 10 Hz */

        /* Get latest sensor data */
        xQueuePeek(xSensorDataQueue, &sensor, 0);

        /* Fill packet */
        pkt.drone_id        = 1;
        pkt.state           = (uint8_t)FC_GetState();
        pkt.latitude        = (float)sensor.latitude;
        pkt.longitude       = (float)sensor.longitude;
        pkt.altitude        = sensor.altitude_baro;
        pkt.speed           = sensor.speed_gps;
        pkt.heading         = sensor.heading_gps;
        pkt.battery_soc     = (uint8_t)POWER_GetSoC();
        pkt.battery_voltage = POWER_GetVoltage();
        pkt.gps_sats        = sensor.gps_satellites;
        pkt.signal_strength = 100;    /* placeholder — RSSI from RF */
        pkt.flight_time_s   = FC_GetFlightTime();
        pkt.timestamp_ms    = HAL_GetTick();

        /* Simple XOR checksum */
        uint8_t cs = 0;
        uint8_t *p = (uint8_t *)&pkt;
        for (size_t i = 0; i < sizeof(pkt) - 1; i++) cs ^= p[i];
        pkt.checksum = cs;

        xQueueSend(xTelemetryQueue, &pkt, 0);
        seq++;
    }
}
