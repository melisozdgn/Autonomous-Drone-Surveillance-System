/**
 * @file    pid_controller.h
 */
#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float prev_derivative;
    float output;
    float out_min, out_max;
    float integral_limit;
    float dt;
    uint8_t enabled;
} PIDController_t;

typedef struct {
    int16_t m1, m2, m3, m4;  /* µs pulse width */
} MotorOutputs_t;

void  PID_Init(PIDController_t *pid,
               float kp, float ki, float kd,
               float out_min, float out_max,
               float integral_limit, float dt);
void  PID_Reset(PIDController_t *pid);
float PID_Compute(PIDController_t *pid, float setpoint, float measurement);

void  FC_Init(void);
void  FC_ComputeMotorOutputs(const void *sensor,
                              float alt_sp,
                              float roll_sp,
                              float pitch_sp,
                              float yaw_rate_sp,
                              MotorOutputs_t *motors);

/* Global PID instances */
extern PIDController_t pid_altitude;
extern PIDController_t pid_vz;
extern PIDController_t pid_roll;
extern PIDController_t pid_pitch;
extern PIDController_t pid_yaw;
extern PIDController_t pid_vx;
extern PIDController_t pid_vy;

#endif /* __PID_CONTROLLER_H */
