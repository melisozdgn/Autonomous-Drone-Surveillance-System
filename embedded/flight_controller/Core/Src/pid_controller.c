/**
 * @file    pid_controller.c
 * @brief   PID Controller — Altitude, Roll, Pitch, Yaw
 *
 * Implements cascaded PID loops:
 *   Outer loop: position/altitude  → velocity setpoint
 *   Inner loop: velocity/angle     → motor thrust
 *
 * Tuned for ADSS quadrotor (m=1.2 kg, 450mm frame, 4S 5000mAh):
 *   Altitude : Kp=5.0  Ki=1.5  Kd=3.0
 *   Roll/Pitch: Kp=3.5  Ki=0.05 Kd=2.2
 *   Yaw      : Kp=4.0  Ki=0.02 Kd=1.5
 */

#include "pid_controller.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* ── Private helpers ─────────────────────────────────────────── */
static float clampf(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/* ═══════════════════════════════════════════════════════════════
 *  PID INIT
 * ═══════════════════════════════════════════════════════════════ */
void PID_Init(PIDController_t *pid,
              float kp, float ki, float kd,
              float out_min, float out_max,
              float integral_limit,
              float dt)
{
    memset(pid, 0, sizeof(PIDController_t));
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->out_min        = out_min;
    pid->out_max        = out_max;
    pid->integral_limit = integral_limit;
    pid->dt             = dt;
    pid->enabled        = 1;
}

void PID_Reset(PIDController_t *pid)
{
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->prev_derivative= 0.0f;
    pid->output         = 0.0f;
}

/* ═══════════════════════════════════════════════════════════════
 *  PID COMPUTE  (called at FLIGHT_LOOP_HZ = 500 Hz)
 * ═══════════════════════════════════════════════════════════════ */
float PID_Compute(PIDController_t *pid, float setpoint, float measurement)
{
    if (!pid->enabled) return 0.0f;

    float error       = setpoint - measurement;
    float dt          = pid->dt;

    /* Proportional */
    float p_term      = pid->kp * error;

    /* Integral with anti-windup clamping */
    pid->integral    += error * dt;
    pid->integral     = clampf(pid->integral,
                               -pid->integral_limit,
                                pid->integral_limit);
    float i_term      = pid->ki * pid->integral;

    /* Derivative with low-pass filter (alpha=0.8) */
    float raw_deriv   = (error - pid->prev_error) / dt;
    float deriv       = 0.8f * pid->prev_derivative + 0.2f * raw_deriv;
    pid->prev_derivative = deriv;
    float d_term      = pid->kd * deriv;

    pid->prev_error   = error;

    /* Sum and clamp */
    float output      = p_term + i_term + d_term;
    output            = clampf(output, pid->out_min, pid->out_max);
    pid->output       = output;

    return output;
}

/* ═══════════════════════════════════════════════════════════════
 *  FLIGHT CONTROLLER INIT — all axes
 * ═══════════════════════════════════════════════════════════════ */

/* Global PID instances */
PIDController_t pid_altitude;
PIDController_t pid_vz;         /* vertical velocity inner loop */
PIDController_t pid_roll;
PIDController_t pid_pitch;
PIDController_t pid_yaw;
PIDController_t pid_vx;         /* north velocity                */
PIDController_t pid_vy;         /* east  velocity                */

#define FC_DT  (1.0f / FLIGHT_LOOP_HZ)   /* 0.002 s */

void FC_Init(void)
{
    /* Altitude outer loop → velocity setpoint (m/s) */
    PID_Init(&pid_altitude,
             5.0f,  1.5f,  3.0f,
            -3.0f,  3.0f,   /* vz setpoint ±3 m/s */
             5.0f, FC_DT);

    /* Vertical velocity inner loop → thrust offset (N) */
    PID_Init(&pid_vz,
             8.0f,  2.0f,  1.5f,
            -6.0f,  6.0f,
             8.0f, FC_DT);

    /* Roll — angle (deg) → rate setpoint */
    PID_Init(&pid_roll,
             3.5f,  0.05f, 2.2f,
            -30.0f, 30.0f,
             10.0f, FC_DT);

    /* Pitch */
    PID_Init(&pid_pitch,
             3.5f,  0.05f, 2.2f,
            -30.0f, 30.0f,
             10.0f, FC_DT);

    /* Yaw rate (deg/s) → torque */
    PID_Init(&pid_yaw,
             4.0f,  0.02f, 1.5f,
            -180.0f, 180.0f,
             20.0f, FC_DT);

    /* Horizontal velocity loops */
    PID_Init(&pid_vx,
             2.5f,  0.1f,  0.8f,
            -5.0f,  5.0f,
             5.0f, FC_DT);

    PID_Init(&pid_vy,
             2.5f,  0.1f,  0.8f,
            -5.0f,  5.0f,
             5.0f, FC_DT);
}

/* ═══════════════════════════════════════════════════════════════
 *  MOTOR MIX  — Quadrotor X-configuration
 *
 *        Front
 *    M3(CCW)  M1(CW)
 *        X
 *    M2(CW)   M4(CCW)
 *        Back
 *
 *  T   = base thrust (hover + altitude PID)
 *  RP  = roll  PID output
 *  PP  = pitch PID output
 *  YP  = yaw   PID output
 *
 *  M1 = T - RP + PP + YP
 *  M2 = T + RP + PP - YP
 *  M3 = T + RP - PP + YP
 *  M4 = T - RP - PP - YP
 * ═══════════════════════════════════════════════════════════════ */

#define HOVER_THROTTLE   1350   /* µs — empirical for 1.2 kg drone */
#define MOTOR_MIN        1050   /* µs — idle */
#define MOTOR_MAX        1950   /* µs — full */
#define MASS_KG          1.2f
#define GRAVITY          9.81f

void FC_ComputeMotorOutputs(const SensorData_t *sensor,
                             float alt_setpoint,
                             float roll_setpoint,
                             float pitch_setpoint,
                             float yaw_rate_setpoint,
                             MotorOutputs_t *motors)
{
    /* ── Altitude cascade ── */
    float vz_sp  = PID_Compute(&pid_altitude,
                                alt_setpoint,
                                sensor->altitude_baro);
    float thrust_delta = PID_Compute(&pid_vz, vz_sp, 0.0f); /* vz from IMU */

    /* Base thrust to hover: T = m*g / cos(roll)*cos(pitch) */
    float cos_rp = cosf(sensor->roll  * (float)M_PI / 180.0f) *
                   cosf(sensor->pitch * (float)M_PI / 180.0f);
    if (cos_rp < 0.5f) cos_rp = 0.5f;
    float T = (MASS_KG * GRAVITY / cos_rp) + thrust_delta;
    /* Map N → µs (linear approximation) */
    int16_t base_us = (int16_t)(HOVER_THROTTLE + (T - MASS_KG * GRAVITY) * 80.0f);

    /* ── Attitude PIDs ── */
    float RP = PID_Compute(&pid_roll,  roll_setpoint,      sensor->roll);
    float PP = PID_Compute(&pid_pitch, pitch_setpoint,     sensor->pitch);
    float YP = PID_Compute(&pid_yaw,   yaw_rate_setpoint,  sensor->gyro_z);

    /* Mix — scale to µs */
    int16_t rp = (int16_t)(RP * 50.0f);
    int16_t pp = (int16_t)(PP * 50.0f);
    int16_t yp = (int16_t)(YP * 30.0f);

    motors->m1 = (int16_t)clampf((float)(base_us - rp + pp + yp),
                                  MOTOR_MIN, MOTOR_MAX);
    motors->m2 = (int16_t)clampf((float)(base_us + rp + pp - yp),
                                  MOTOR_MIN, MOTOR_MAX);
    motors->m3 = (int16_t)clampf((float)(base_us + rp - pp + yp),
                                  MOTOR_MIN, MOTOR_MAX);
    motors->m4 = (int16_t)clampf((float)(base_us - rp - pp - yp),
                                  MOTOR_MIN, MOTOR_MAX);
}
