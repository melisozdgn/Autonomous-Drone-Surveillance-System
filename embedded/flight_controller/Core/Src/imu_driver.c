/**
 * @file    imu_driver.c
 * @brief   MPU-6050 IMU Driver — I2C + Complementary Filter
 *
 * Features:
 *   - 6-DOF: 3-axis accel + 3-axis gyro
 *   - I2C at 400 kHz
 *   - DMP bypass mode — raw data + SW filter
 *   - Complementary filter: α=0.98 for roll/pitch
 *   - Gyro calibration on startup (2s still required)
 */

#include "imu_driver.h"
#include "main.h"
#include <math.h>
#include <string.h>

/* MPU-6050 register map */
#define MPU6050_ADDR        0x68 << 1   /* AD0=GND */
#define MPU6050_SMPLRT_DIV  0x19
#define MPU6050_CONFIG      0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_FIFO_EN     0x23
#define MPU6050_INT_ENABLE  0x38
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_WHO_AM_I    0x75

/* Sensitivity (±2g accel, ±500°/s gyro) */
#define ACCEL_SENS  16384.0f   /* LSB/g    */
#define GYRO_SENS   65.5f      /* LSB/°/s  */
#define GRAVITY     9.81f

/* Complementary filter coefficient */
#define CF_ALPHA    0.98f
#define DT_IMU      (1.0f / SENSOR_LOOP_HZ)

/* Private state */
static float s_gyro_offset_x = 0.0f;
static float s_gyro_offset_y = 0.0f;
static float s_gyro_offset_z = 0.0f;
static float s_roll  = 0.0f;
static float s_pitch = 0.0f;

static HAL_StatusTypeDef imu_write_reg(I2C_HandleTypeDef *hi2c,
                                        uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, buf, 2,
                                    HAL_MAX_DELAY);
}

static HAL_StatusTypeDef imu_read_reg(I2C_HandleTypeDef *hi2c,
                                       uint8_t reg,
                                       uint8_t *data, uint16_t len)
{
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, &reg, 1, HAL_MAX_DELAY);
    return HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, data, len,
                                   HAL_MAX_DELAY);
}

/* ═══════════════════════════════════════════════════════════════
 *  IMU INIT
 * ═══════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef IMU_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who;
    imu_read_reg(hi2c, MPU6050_WHO_AM_I, &who, 1);
    if (who != 0x68) return HAL_ERROR;   /* Wrong device */

    /* Wake up — use PLL with X gyro */
    imu_write_reg(hi2c, MPU6050_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    /* Sample rate: 1 kHz / (1 + 4) = 200 Hz */
    imu_write_reg(hi2c, MPU6050_SMPLRT_DIV, 0x04);

    /* DLPF: ~44 Hz bandwidth */
    imu_write_reg(hi2c, MPU6050_CONFIG, 0x03);

    /* Gyro full scale: ±500°/s */
    imu_write_reg(hi2c, MPU6050_GYRO_CONFIG, 0x08);

    /* Accel full scale: ±2g */
    imu_write_reg(hi2c, MPU6050_ACCEL_CONFIG, 0x00);

    /* Calibrate gyro — average 500 samples */
    IMU_CalibrateGyro(hi2c);

    return HAL_OK;
}

void IMU_CalibrateGyro(I2C_HandleTypeDef *hi2c)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint8_t buf[6];
    int16_t raw_x, raw_y, raw_z;

    for (int i = 0; i < 500; i++) {
        imu_read_reg(hi2c, MPU6050_GYRO_XOUT_H, buf, 6);
        raw_x = (int16_t)((buf[0] << 8) | buf[1]);
        raw_y = (int16_t)((buf[2] << 8) | buf[3]);
        raw_z = (int16_t)((buf[4] << 8) | buf[5]);
        sum_x += raw_x;
        sum_y += raw_y;
        sum_z += raw_z;
        HAL_Delay(2);
    }
    s_gyro_offset_x = (float)sum_x / 500.0f;
    s_gyro_offset_y = (float)sum_y / 500.0f;
    s_gyro_offset_z = (float)sum_z / 500.0f;
}

/* ═══════════════════════════════════════════════════════════════
 *  READ + COMPLEMENTARY FILTER
 * ═══════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef IMU_Read(I2C_HandleTypeDef *hi2c, SensorData_t *out)
{
    uint8_t  buf[14];
    int16_t  raw_ax, raw_ay, raw_az;
    int16_t  raw_gx, raw_gy, raw_gz;

    /* Burst-read accel + temp + gyro (14 bytes from 0x3B) */
    HAL_StatusTypeDef status = imu_read_reg(hi2c, MPU6050_ACCEL_XOUT_H,
                                             buf, 14);
    if (status != HAL_OK) return status;

    raw_ax = (int16_t)((buf[0]  << 8) | buf[1]);
    raw_ay = (int16_t)((buf[2]  << 8) | buf[3]);
    raw_az = (int16_t)((buf[4]  << 8) | buf[5]);
    /* buf[6..7] = temperature (unused here) */
    raw_gx = (int16_t)((buf[8]  << 8) | buf[9]);
    raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    /* Scale to physical units */
    out->accel_x = (float)raw_ax / ACCEL_SENS * GRAVITY;
    out->accel_y = (float)raw_ay / ACCEL_SENS * GRAVITY;
    out->accel_z = (float)raw_az / ACCEL_SENS * GRAVITY;

    out->gyro_x  = ((float)raw_gx - s_gyro_offset_x) / GYRO_SENS;
    out->gyro_y  = ((float)raw_gy - s_gyro_offset_y) / GYRO_SENS;
    out->gyro_z  = ((float)raw_gz - s_gyro_offset_z) / GYRO_SENS;

    /* Accel-derived angles */
    float roll_acc  = atan2f(out->accel_y, out->accel_z) * 180.0f
                      / (float)M_PI;
    float pitch_acc = atan2f(-out->accel_x,
                              sqrtf(out->accel_y * out->accel_y +
                                    out->accel_z * out->accel_z))
                      * 180.0f / (float)M_PI;

    /* Complementary filter */
    s_roll  = CF_ALPHA * (s_roll  + out->gyro_x * DT_IMU)
              + (1.0f - CF_ALPHA) * roll_acc;
    s_pitch = CF_ALPHA * (s_pitch + out->gyro_y * DT_IMU)
              + (1.0f - CF_ALPHA) * pitch_acc;

    out->roll  = s_roll;
    out->pitch = s_pitch;
    /* Yaw integrates gyro only (no magnetometer) */
    out->yaw  += out->gyro_z * DT_IMU;

    out->timestamp_ms = HAL_GetTick();
    return HAL_OK;
}
