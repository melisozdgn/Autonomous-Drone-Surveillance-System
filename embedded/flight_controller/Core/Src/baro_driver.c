/**
 * @file    baro_driver.c
 * @brief   BMP280 Barometer Driver — I2C
 *
 * Provides: pressure (hPa), temperature (°C), altitude AGL (m)
 * Altitude from barometric formula:
 *   h = 44330 * (1 - (P/P0)^(1/5.255))
 */

#include "baro_driver.h"
#include "main.h"

#define BMP280_ADDR         0x76 << 1
#define BMP280_REG_ID       0xD0
#define BMP280_REG_RESET    0xE0
#define BMP280_REG_STATUS   0xF3
#define BMP280_REG_CTRL     0xF4
#define BMP280_REG_CONFIG   0xF5
#define BMP280_REG_PRES_MSB 0xF7
#define BMP280_REG_CALIB    0x88
#define BMP280_CHIP_ID      0x60

/* Calibration data */
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5;
static int16_t  dig_P6, dig_P7, dig_P8, dig_P9;
static float    s_sea_level_pressure = 1013.25f;
static I2C_HandleTypeDef *s_hi2c = NULL;

static HAL_StatusTypeDef baro_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return HAL_I2C_Master_Transmit(s_hi2c, BMP280_ADDR, buf, 2, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef baro_read(uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_I2C_Master_Transmit(s_hi2c, BMP280_ADDR, &reg, 1, HAL_MAX_DELAY);
    return HAL_I2C_Master_Receive(s_hi2c, BMP280_ADDR, data, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BARO_Init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    uint8_t id;
    baro_read(BMP280_REG_ID, &id, 1);
    if (id != BMP280_CHIP_ID) return HAL_ERROR;

    /* Read calibration (24 bytes) */
    uint8_t cal[24];
    baro_read(BMP280_REG_CALIB, cal, 24);
    dig_T1 = (uint16_t)(cal[1]  << 8 | cal[0]);
    dig_T2 = (int16_t) (cal[3]  << 8 | cal[2]);
    dig_T3 = (int16_t) (cal[5]  << 8 | cal[4]);
    dig_P1 = (uint16_t)(cal[7]  << 8 | cal[6]);
    dig_P2 = (int16_t) (cal[9]  << 8 | cal[8]);
    dig_P3 = (int16_t) (cal[11] << 8 | cal[10]);
    dig_P4 = (int16_t) (cal[13] << 8 | cal[12]);
    dig_P5 = (int16_t) (cal[15] << 8 | cal[14]);
    dig_P6 = (int16_t) (cal[17] << 8 | cal[16]);
    dig_P7 = (int16_t) (cal[19] << 8 | cal[18]);
    dig_P8 = (int16_t) (cal[21] << 8 | cal[20]);
    dig_P9 = (int16_t) (cal[23] << 8 | cal[22]);

    /* Normal mode, oversampling x4 temp, x4 pressure, t_standby 125ms */
    baro_write(BMP280_REG_CONFIG, 0x10);
    baro_write(BMP280_REG_CTRL,   0x93);

    /* Calibrate sea-level pressure from 10 samples */
    HAL_Delay(200);
    float p_sum = 0.0f;
    for (int i = 0; i < 10; i++) {
        float p, t, h;
        BARO_Read(hi2c, &p, &t, &h);
        p_sum += p;
        HAL_Delay(50);
    }
    s_sea_level_pressure = p_sum / 10.0f;

    return HAL_OK;
}

HAL_StatusTypeDef BARO_Read(I2C_HandleTypeDef *hi2c,
                              float *pressure_hpa,
                              float *temp_c,
                              float *altitude_m)
{
    uint8_t data[6];
    baro_read(BMP280_REG_PRES_MSB, data, 6);

    int32_t adc_P = (int32_t)(((uint32_t)data[0] << 12) |
                               ((uint32_t)data[1] <<  4) |
                               ((uint32_t)data[2] >>  4));
    int32_t adc_T = (int32_t)(((uint32_t)data[3] << 12) |
                               ((uint32_t)data[4] <<  4) |
                               ((uint32_t)data[5] >>  4));

    /* Temperature compensation */
    int64_t var1 = ((int64_t)adc_T >> 3) - ((int64_t)dig_T1 << 1);
    var1 = (var1 * (int64_t)dig_T2) >> 11;
    int64_t var2 = (((int64_t)adc_T >> 4) - (int64_t)dig_T1);
    var2 = (((var2 * var2) >> 12) * (int64_t)dig_T3) >> 14;
    int64_t t_fine = var1 + var2;
    *temp_c = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

    /* Pressure compensation */
    int64_t pvar1 = (int64_t)t_fine - 128000;
    int64_t pvar2 = pvar1 * pvar1 * (int64_t)dig_P6;
    pvar2 += ((pvar1 * (int64_t)dig_P5) << 17);
    pvar2 += ((int64_t)dig_P4 << 35);
    pvar1  = ((pvar1 * pvar1 * (int64_t)dig_P3) >> 8) +
             ((pvar1 * (int64_t)dig_P2) << 12);
    pvar1  = ((((int64_t)1 << 47) + pvar1) * (int64_t)dig_P1) >> 33;
    if (pvar1 == 0) { *pressure_hpa = 0; *altitude_m = 0; return HAL_OK; }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - pvar2) * 3125) / pvar1;
    pvar1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    pvar2 = ((int64_t)dig_P8 * p) >> 19;
    p = ((p + pvar1 + pvar2) >> 8) + ((int64_t)dig_P7 << 4);
    *pressure_hpa = (float)(uint32_t)p / 25600.0f;

    /* Altitude from barometric formula */
    *altitude_m = 44330.0f * (1.0f -
                  powf(*pressure_hpa / s_sea_level_pressure,
                       1.0f / 5.2553f));
    return HAL_OK;
}
