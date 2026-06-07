/**
 * @file  vl53l1x.c
 * @brief VL53L1X Time-of-Flight Laser Rangefinder
 *
 * Used for precision altitude measurement during docking (< 3 m)
 * Range: 40 mm – 4000 mm | Accuracy: ±1 mm | I2C addr: 0x29
 */
#include "vl53l1x.h"

#define ADDR        (0x29 << 1)
#define REG_START   0x0087
#define REG_RESULT  0x0096
#define REG_STATUS  0x0089

static void wr16(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t val) {
    uint8_t b[3] = {reg>>8, reg&0xFF, val};
    HAL_I2C_Master_Transmit(hi2c, ADDR, b, 3, HAL_MAX_DELAY);
}
static uint8_t rd8(I2C_HandleTypeDef *hi2c, uint16_t reg) {
    uint8_t a[2]={reg>>8,reg&0xFF}, v=0;
    HAL_I2C_Master_Transmit(hi2c, ADDR, a, 2, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive (hi2c, ADDR, &v, 1, HAL_MAX_DELAY);
    return v;
}

HAL_StatusTypeDef VL53L1X_Init(I2C_HandleTypeDef *hi2c) {
    HAL_Delay(10);
    wr16(hi2c, REG_START, 0x40); /* Start continuous ranging */
    HAL_Delay(100);
    return HAL_OK;
}

uint8_t VL53L1X_DataReady(I2C_HandleTypeDef *hi2c) {
    return (rd8(hi2c, REG_STATUS) & 0x01) ? 1 : 0;
}

float VL53L1X_ReadDistance_m(I2C_HandleTypeDef *hi2c) {
    uint8_t a[2] = {REG_RESULT>>8, REG_RESULT&0xFF};
    uint8_t d[2] = {0};
    HAL_I2C_Master_Transmit(hi2c, ADDR, a, 2, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive (hi2c, ADDR, d, 2, HAL_MAX_DELAY);
    uint16_t mm = ((uint16_t)d[0]<<8)|d[1];
    /* Clear interrupt */
    wr16(hi2c, 0x0086, 0x01);
    return mm / 1000.0f;
}
