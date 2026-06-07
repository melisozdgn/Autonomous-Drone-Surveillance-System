/**
 * @file    motor_control.c
 * @brief   ESC Motor Control — PWM via STM32 TIM1
 *
 * ESC Protocol: Standard PWM 50Hz, 1000-2000 µs
 *   1000 µs = motor off (disarmed)
 *   1050 µs = minimum throttle (armed, spinning)
 *   2000 µs = full throttle
 *
 * Motor layout (X-frame):
 *        FRONT
 *   M3(CCW)  M1(CW)
 *       \    /
 *        \  /
 *        /  \
 *       /    \
 *   M2(CW)  M4(CCW)
 *        BACK
 *
 * TIM1 Channels:
 *   CH1 → M1 (front-right)
 *   CH2 → M2 (back-left)
 *   CH3 → M3 (front-left)
 *   CH4 → M4 (back-right)
 */

#include "motor_control.h"
#include "main.h"

#define MOTOR_MIN_US    1000
#define MOTOR_MAX_US    2000
#define MOTOR_ARM_US    1050
#define MOTOR_IDLE_US   1000
#define ARM_DELAY_MS    3000

static TIM_HandleTypeDef *s_htim = NULL;
static uint8_t s_armed = 0;

static inline int16_t clamp_us(int16_t us)
{
    if (us < MOTOR_MIN_US) return MOTOR_MIN_US;
    if (us > MOTOR_MAX_US) return MOTOR_MAX_US;
    return us;
}

void MOTOR_Init(TIM_HandleTypeDef *htim)
{
    s_htim = htim;

    /* Start all PWM channels */
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);

    /* Hold 1000 µs for ESC init */
    MOTOR_SetAll(MOTOR_IDLE_US);
    HAL_Delay(ARM_DELAY_MS);
}

void MOTOR_Arm(TIM_HandleTypeDef *htim)
{
    /* ESC arm sequence: 2000 µs then 1000 µs */
    MOTOR_SetAll(MOTOR_MAX_US);
    HAL_Delay(2000);
    MOTOR_SetAll(MOTOR_IDLE_US);
    HAL_Delay(1000);
    s_armed = 1;
}

void MOTOR_Disarm(TIM_HandleTypeDef *htim)
{
    MOTOR_SetAll(MOTOR_IDLE_US);
    s_armed = 0;
}

void MOTOR_SetPWM(TIM_HandleTypeDef *htim,
                  int16_t m1, int16_t m2,
                  int16_t m3, int16_t m4)
{
    if (!s_armed) return;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)clamp_us(m1));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)clamp_us(m2));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint32_t)clamp_us(m3));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, (uint32_t)clamp_us(m4));
}

void MOTOR_SetAll(int16_t us)
{
    if (s_htim == NULL) return;
    uint32_t v = (uint32_t)clamp_us(us);
    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, v);
    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_2, v);
    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_3, v);
    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_4, v);
}
