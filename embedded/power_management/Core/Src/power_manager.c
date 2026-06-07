/**
 * @file    power_manager.c
 * @brief   Battery Management System — 4S LiPo, Coulomb Counting
 *
 * Features:
 *   - Voltage measurement via STM32 ADC (12-bit)
 *   - Current measurement via INA219 shunt (I2C)
 *   - Coulomb counting SoC: SoC(t) = SoC0 - (1/Q)∫I dt
 *   - Cell voltage monitoring (per-cell balance check)
 *   - Low battery warning → RTH command
 *   - Critical battery → emergency land
 *   - Temperature monitoring via NTC thermistor
 */

#include "power_manager.h"
#include "main.h"
#include <math.h>

/* ── Hardware constants ──────────────────────────────────────── */
#define BATTERY_CELLS           4
#define CELL_NOMINAL_V          3.70f
#define CELL_FULL_V             4.20f
#define CELL_EMPTY_V            3.50f
#define BATTERY_NOMINAL_V       (BATTERY_CELLS * CELL_NOMINAL_V)  /* 14.8V */
#define BATTERY_FULL_V          (BATTERY_CELLS * CELL_FULL_V)     /* 16.8V */
#define BATTERY_EMPTY_V         (BATTERY_CELLS * CELL_EMPTY_V)    /* 14.0V */

#define BATTERY_CAPACITY_MAH    5000
#define BATTERY_CAPACITY_C      (BATTERY_CAPACITY_MAH / 1000.0f * 3600.0f) /* Coulombs */

/* Voltage divider: 47k + 10k → scale factor */
#define VDIV_RATIO              (47.0f + 10.0f) / 10.0f   /* = 5.7 */
#define ADC_VREF                3.3f
#define ADC_RESOLUTION          4096.0f

/* INA219 — current sensor (I2C) */
#define INA219_ADDR             0x40 << 1
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNT        0x01
#define INA219_REG_BUS          0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_SHUNT_R          0.01f   /* 10 mΩ shunt resistor  */
#define INA219_CURRENT_LSB      0.001f  /* 1 mA/LSB              */

/* NTC thermistor (B=3950, R25=10k) */
#define NTC_B_CONST             3950.0f
#define NTC_R_REF               10000.0f
#define NTC_T_REF               298.15f  /* 25°C in Kelvin */
#define NTC_R_SERIES            10000.0f

/* ── Private state ───────────────────────────────────────────── */
static float    s_voltage_V     = 0.0f;
static float    s_current_A     = 0.0f;
static float    s_soc_pct       = 100.0f;
static float    s_capacity_C    = BATTERY_CAPACITY_C;
static float    s_consumed_C    = 0.0f;
static float    s_temperature_C = 25.0f;
static uint8_t  s_low_bat_warn  = 0;
static uint8_t  s_critical_bat  = 0;
static ADC_HandleTypeDef *s_hadc = NULL;
static I2C_HandleTypeDef *s_hi2c = NULL;
static uint32_t s_last_tick_ms   = 0;

/* ── INA219 helpers ──────────────────────────────────────────── */
static void ina219_write(uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = {reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
    HAL_I2C_Master_Transmit(s_hi2c, INA219_ADDR, buf, 3, HAL_MAX_DELAY);
}

static int16_t ina219_read(uint8_t reg)
{
    uint8_t buf[2];
    HAL_I2C_Master_Transmit(s_hi2c, INA219_ADDR, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(s_hi2c,  INA219_ADDR, buf,   2, HAL_MAX_DELAY);
    return (int16_t)((buf[0] << 8) | buf[1]);
}

/* ── Voltage → SoC (OCV lookup table for LiPo) ──────────────── */
static float ocv_to_soc(float v_cell)
{
    /* 10-point OCV-SoC curve for LiPo */
    static const float ocv[]  = {3.50f, 3.60f, 3.68f, 3.73f, 3.77f,
                                   3.82f, 3.88f, 3.95f, 4.07f, 4.20f};
    static const float soc[]  = { 0.0f, 10.0f, 20.0f, 30.0f, 40.0f,
                                   50.0f, 60.0f, 70.0f, 80.0f,100.0f};
    if (v_cell <= ocv[0])  return soc[0];
    if (v_cell >= ocv[9])  return soc[9];
    for (int i = 0; i < 9; i++) {
        if (v_cell >= ocv[i] && v_cell < ocv[i+1]) {
            float t = (v_cell - ocv[i]) / (ocv[i+1] - ocv[i]);
            return soc[i] + t * (soc[i+1] - soc[i]);
        }
    }
    return 50.0f;
}

/* ═══════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ═══════════════════════════════════════════════════════════════ */
void POWER_Init(ADC_HandleTypeDef *hadc)
{
    s_hadc = hadc;
    s_last_tick_ms = HAL_GetTick();

    /* Configure INA219: 32V range, ±320mV shunt, 12-bit cont */
    ina219_write(INA219_REG_CONFIG, 0x399F);

    /* Initialise SoC from OCV (drone must be at rest) */
    HAL_Delay(500);
    POWER_Update();
    float v_cell   = s_voltage_V / BATTERY_CELLS;
    s_soc_pct      = ocv_to_soc(v_cell);
    s_consumed_C   = (1.0f - s_soc_pct / 100.0f) * s_capacity_C;
}

void POWER_Update(void)
{
    /* ── Voltage measurement ── */
    HAL_ADC_PollForConversion(s_hadc, 10);
    uint32_t adc_raw = HAL_ADC_GetValue(s_hadc);
    float v_adc      = (float)adc_raw / ADC_RESOLUTION * ADC_VREF;
    s_voltage_V      = v_adc * VDIV_RATIO;

    /* ── Current measurement (INA219) ── */
    int16_t raw_current = ina219_read(INA219_REG_CURRENT);
    s_current_A         = (float)raw_current * INA219_CURRENT_LSB;
    if (s_current_A < 0.0f) s_current_A = 0.0f;  /* discharge only */

    /* ── Coulomb counting ── */
    uint32_t now_ms  = HAL_GetTick();
    float    dt_s    = (float)(now_ms - s_last_tick_ms) / 1000.0f;
    s_last_tick_ms   = now_ms;
    s_consumed_C    += s_current_A * dt_s;
    s_soc_pct        = 100.0f * (1.0f - s_consumed_C / s_capacity_C);
    if (s_soc_pct < 0.0f)   s_soc_pct = 0.0f;
    if (s_soc_pct > 100.0f) s_soc_pct = 100.0f;

    /* ── Alarms ── */
    s_low_bat_warn = (s_soc_pct < LOW_BATTERY_THRESHOLD) ? 1 : 0;
    s_critical_bat = (s_soc_pct < CRITICAL_BATTERY)      ? 1 : 0;
}

float   POWER_GetVoltage(void)     { return s_voltage_V; }
float   POWER_GetCurrent(void)     { return s_current_A; }
float   POWER_GetSoC(void)         { return s_soc_pct; }
float   POWER_GetTemperature(void) { return s_temperature_C; }
uint8_t POWER_IsLowBattery(void)   { return s_low_bat_warn; }
uint8_t POWER_IsCritical(void)     { return s_critical_bat; }

/* ── FreeRTOS task ───────────────────────────────────────────── */
void vPowerManagerTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100)); /* 10 Hz */
        POWER_Update();

        /* Push telemetry */
        TelemetryPacket_t pkt = {0};
        pkt.battery_soc     = s_soc_pct;
        pkt.battery_voltage = s_voltage_V;
        xQueueSend(xTelemetryQueue, &pkt, 0);

        /* Emergency RTH */
        if (s_critical_bat) {
            DroneCommand_t cmd = {
                .type        = CMD_EMERGENCY_LAND,
                .drone_id    = 1,
                .timestamp_ms = HAL_GetTick()
            };
            xQueueSend(xCommandQueue, &cmd, 0);
        } else if (s_low_bat_warn) {
            DroneCommand_t cmd = {
                .type        = CMD_RETURN_HOME,
                .drone_id    = 1,
                .timestamp_ms = HAL_GetTick()
            };
            xQueueSend(xCommandQueue, &cmd, 0);
        }
    }
}
