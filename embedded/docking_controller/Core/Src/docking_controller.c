/**
 * @file    docking_controller.c
 * @brief   Autonomous Docking — ArUco Marker Vision + PID Descent
 *
 * Implements the precision landing sequence described in:
 *   Lee et al. (2018) "Vision-based Autonomous Landing using RL"
 *
 * Hardware:
 *   - Downward-facing OV7670 camera (320×240, I2C config, parallel data)
 *   - STM32 DCMI peripheral captures frames
 *   - VL53L1X laser rangefinder (I2C) for altitude < 3m
 *   - Servo motor for landing gear deployment (TIM2 CH1)
 *
 * Algorithm:
 *   Phase 1 (> 5m)  : GPS-guided approach to dock coordinates
 *   Phase 2 (5-1.5m): Camera ON, detect ArUco ID=42, PID align
 *   Phase 3 (< 1.5m): Laser rangefinder takes over, fine descent
 *   Phase 4 (< 0.2m): Motor cutoff, landing gear deploy
 */

#include "docking_controller.h"
#include "main.h"
#include "pid_controller.h"
#include <math.h>
#include <string.h>

/* ── Constants ───────────────────────────────────────────────── */
#define ARUCO_ID_TARGET         42
#define CAMERA_WIDTH            320
#define CAMERA_HEIGHT           240
#define CAMERA_FOV_DEG          60.0f
#define FOCAL_LENGTH_PX         (CAMERA_WIDTH / (2.0f * tanf(CAMERA_FOV_DEG * (float)M_PI / 360.0f)))
#define MARKER_SIZE_M           0.15f   /* 15 cm ArUco marker   */

#define PHASE2_ALT_M            5.0f
#define PHASE3_ALT_M            1.5f
#define PHASE4_ALT_M            0.2f

#define DOCK_LAT                39.9135
#define DOCK_LON                32.2851

#define XY_TOLERANCE_M          0.03f   /* ±3 cm final precision */
#define DESCENT_SPEED_MS        0.3f    /* 0.3 m/s slow descent  */

/* VL53L1X laser rangefinder (I2C) */
#define VL53L1X_ADDR            0x29 << 1
#define VL53L1X_REG_RESULT      0x0096
#define VL53L1X_REG_START       0x0087

/* ── Private state ───────────────────────────────────────────── */
typedef enum {
    DOCK_PHASE_GPS    = 0,
    DOCK_PHASE_VISION,
    DOCK_PHASE_LASER,
    DOCK_PHASE_LANDED
} DockPhase_t;

static DockPhase_t  s_phase        = DOCK_PHASE_GPS;
static float        s_x_error_m   = 0.0f;
static float        s_y_error_m   = 0.0f;
static float        s_laser_alt_m = 0.0f;
static uint8_t      s_marker_found = 0;
static PIDController_t s_pid_dock_x;
static PIDController_t s_pid_dock_y;
static TIM_HandleTypeDef *s_htim_servo = NULL;

/* ── VL53L1X laser rangefinder ───────────────────────────────── */
static HAL_StatusTypeDef laser_init(I2C_HandleTypeDef *hi2c)
{
    /* Start continuous ranging */
    uint8_t buf[3] = {(VL53L1X_REG_START >> 8),
                       (VL53L1X_REG_START & 0xFF), 0x40};
    return HAL_I2C_Master_Transmit(hi2c, VL53L1X_ADDR, buf, 3,
                                    HAL_MAX_DELAY);
}

static float laser_read_m(I2C_HandleTypeDef *hi2c)
{
    uint8_t addr[2] = {(VL53L1X_REG_RESULT >> 8),
                        (VL53L1X_REG_RESULT & 0xFF)};
    uint8_t data[2];
    HAL_I2C_Master_Transmit(hi2c, VL53L1X_ADDR, addr, 2, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c,  VL53L1X_ADDR, data, 2, HAL_MAX_DELAY);
    uint16_t mm = ((uint16_t)data[0] << 8) | data[1];
    return (float)mm / 1000.0f;
}

/* ── Simple ArUco corner detection (stub — runs on companion CPU) */
/*
 * In real deployment: GAP8 or RPi companion computer runs OpenCV
 * ArUco detection and sends pixel offset via UART to STM32.
 * Here we model the interface.
 */
typedef struct {
    uint8_t  found;
    uint8_t  marker_id;
    int16_t  cx_px;       /* marker centre X in image, px from centre */
    int16_t  cy_px;       /* marker centre Y in image, px from centre */
    float    estimated_dist_m;
} ArUcoResult_t;

static ArUcoResult_t s_aruco = {0};

/* Called when companion MCU sends UART frame with detection result */
void DOCK_OnArUcoFrame(uint8_t found, uint8_t id,
                        int16_t cx, int16_t cy, float dist)
{
    s_aruco.found          = found;
    s_aruco.marker_id      = id;
    s_aruco.cx_px          = cx;
    s_aruco.cy_px          = cy;
    s_aruco.estimated_dist_m = dist;
}

/* ── Pixel offset → metres ───────────────────────────────────── */
static void pixel_to_meters(int16_t cx_px, int16_t cy_px,
                              float alt_m,
                              float *x_m, float *y_m)
{
    /* Pinhole model: x_m = cx_px * alt_m / focal_px */
    *x_m = (float)cx_px * alt_m / FOCAL_LENGTH_PX;
    *y_m = (float)cy_px * alt_m / FOCAL_LENGTH_PX;
}

/* ── Landing gear servo ──────────────────────────────────────── */
static void deploy_landing_gear(void)
{
    /* 1800 µs = extended position */
    __HAL_TIM_SET_COMPARE(s_htim_servo, TIM_CHANNEL_1, 1800);
}

static void retract_landing_gear(void)
{
    /* 1200 µs = retracted */
    __HAL_TIM_SET_COMPARE(s_htim_servo, TIM_CHANNEL_1, 1200);
}

/* ═══════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ═══════════════════════════════════════════════════════════════ */
void DOCK_Init(TIM_HandleTypeDef *htim)
{
    s_htim_servo = htim;
    s_phase      = DOCK_PHASE_GPS;

    /* Docking position PIDs — tight bandwidth for precision */
    PID_Init(&s_pid_dock_x,
             2.0f, 0.5f, 1.0f,
            -1.5f, 1.5f,   /* horizontal velocity ±1.5 m/s */
             2.0f, 1.0f / 50.0f);   /* 50 Hz docking loop */

    PID_Init(&s_pid_dock_y,
             2.0f, 0.5f, 1.0f,
            -1.5f, 1.5f,
             2.0f, 1.0f / 50.0f);

    /* Start PWM for servo */
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    retract_landing_gear();
}

/**
 * @brief  Run docking state machine — call at 50 Hz
 * @retval 1 if landed successfully, 0 if still descending
 */
uint8_t DOCK_Run(const SensorData_t *sensor,
                  float *vx_cmd, float *vy_cmd, float *vz_cmd)
{
    float alt = sensor->altitude_baro;

    switch (s_phase) {

    /* ── Phase 1: GPS approach ── */
    case DOCK_PHASE_GPS:
        *vx_cmd = 0.0f;
        *vy_cmd = 0.0f;
        *vz_cmd = -DESCENT_SPEED_MS;
        if (alt < PHASE2_ALT_M) {
            s_phase = DOCK_PHASE_VISION;
        }
        break;

    /* ── Phase 2: Vision alignment ── */
    case DOCK_PHASE_VISION:
        if (s_aruco.found && s_aruco.marker_id == ARUCO_ID_TARGET) {
            s_marker_found = 1;
            pixel_to_meters(s_aruco.cx_px, s_aruco.cy_px,
                             alt, &s_x_error_m, &s_y_error_m);

            *vx_cmd = PID_Compute(&s_pid_dock_x, 0.0f, s_x_error_m);
            *vy_cmd = PID_Compute(&s_pid_dock_y, 0.0f, s_y_error_m);
            *vz_cmd = -DESCENT_SPEED_MS;
        } else {
            /* Hover and wait for marker */
            *vx_cmd = 0.0f;
            *vy_cmd = 0.0f;
            *vz_cmd = 0.0f;
        }
        if (alt < PHASE3_ALT_M && s_marker_found) {
            s_phase = DOCK_PHASE_LASER;
            PID_Reset(&s_pid_dock_x);
            PID_Reset(&s_pid_dock_y);
        }
        break;

    /* ── Phase 3: Laser fine descent ── */
    case DOCK_PHASE_LASER:
        s_laser_alt_m = laser_read_m(&hi2c2);

        if (s_aruco.found) {
            pixel_to_meters(s_aruco.cx_px, s_aruco.cy_px,
                             s_laser_alt_m, &s_x_error_m, &s_y_error_m);
            *vx_cmd = PID_Compute(&s_pid_dock_x, 0.0f, s_x_error_m);
            *vy_cmd = PID_Compute(&s_pid_dock_y, 0.0f, s_y_error_m);
        } else {
            *vx_cmd = 0.0f;
            *vy_cmd = 0.0f;
        }
        *vz_cmd = -DESCENT_SPEED_MS * 0.5f;  /* slower final approach */

        if (s_laser_alt_m < PHASE4_ALT_M) {
            s_phase = DOCK_PHASE_LANDED;
        }
        break;

    /* ── Phase 4: Touchdown ── */
    case DOCK_PHASE_LANDED:
        *vx_cmd = 0.0f;
        *vy_cmd = 0.0f;
        *vz_cmd = 0.0f;
        deploy_landing_gear();
        return 1;   /* Signal: landed */
    }

    return 0;
}

float   DOCK_GetXError(void)     { return s_x_error_m; }
float   DOCK_GetYError(void)     { return s_y_error_m; }
float   DOCK_GetLaserAlt(void)   { return s_laser_alt_m; }
uint8_t DOCK_MarkerFound(void)   { return s_marker_found; }
