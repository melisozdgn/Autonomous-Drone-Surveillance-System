/**
 * @file    flight_task.c
 * @brief   FreeRTOS Flight Control Task — 500 Hz
 *
 * This is the highest-priority task. It:
 *   1. Reads sensor data from the queue (produced by SensorManagerTask)
 *   2. Runs the cascaded PID loops
 *   3. Applies motor mix and writes PWM
 *   4. Manages state machine transitions
 */

#include "main.h"
#include "flight_controller.h"
#include "pid_controller.h"
#include "motor_control.h"
#include "gps_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <math.h>
#include <string.h>

/* ── Private state ───────────────────────────────────────────── */
static DroneState_t   s_state         = DRONE_STATE_IDLE;
static SensorData_t   s_sensor        = {0};
static WaypointList_t s_waypoints     = {0};
static uint8_t        s_wp_index      = 0;
static float          s_alt_setpoint  = 0.0f;
static float          s_yaw_setpoint  = 0.0f;
static uint32_t       s_flight_time_s = 0;
static TickType_t     s_last_tick;

/* ── Waypoint helpers ────────────────────────────────────────── */
static float haversine_distance(double lat1, double lon1,
                                  double lat2, double lon2)
{
    const double R = 6371000.0;
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dLat/2)*sin(dLat/2) +
               cos(lat1*M_PI/180.0)*cos(lat2*M_PI/180.0)*
               sin(dLon/2)*sin(dLon/2);
    double c = 2*atan2(sqrt(a), sqrt(1-a));
    return (float)(R * c);
}

static float bearing_to(double lat1, double lon1,
                          double lat2, double lon2)
{
    double dLon  = (lon2 - lon1) * M_PI / 180.0;
    double y     = sin(dLon) * cos(lat2 * M_PI / 180.0);
    double x     = cos(lat1 * M_PI / 180.0)*sin(lat2 * M_PI / 180.0) -
                   sin(lat1 * M_PI / 180.0)*cos(lat2 * M_PI / 180.0)*cos(dLon);
    float  brng  = (float)(atan2(y, x) * 180.0 / M_PI);
    return fmodf(brng + 360.0f, 360.0f);
}

/* ── Default patrol waypoints (Atılım Campus) ───────────────── */
static void load_default_waypoints(void)
{
    s_waypoints.count = 6;
    /* Zone A patrol — counter-clockwise rectangle */
    s_waypoints.wp[0] = (Waypoint_t){39.9130, 32.2820, HOME_ALTITUDE_M, 4.0f};
    s_waypoints.wp[1] = (Waypoint_t){39.9130, 32.2850, HOME_ALTITUDE_M, 4.0f};
    s_waypoints.wp[2] = (Waypoint_t){39.9110, 32.2850, HOME_ALTITUDE_M, 4.0f};
    s_waypoints.wp[3] = (Waypoint_t){39.9110, 32.2820, HOME_ALTITUDE_M, 4.0f};
    s_waypoints.wp[4] = (Waypoint_t){39.9120, 32.2835, HOME_ALTITUDE_M, 3.0f};
    s_waypoints.wp[5] = (Waypoint_t){39.9130, 32.2820, HOME_ALTITUDE_M, 4.0f};
}

/* ── State machine ───────────────────────────────────────────── */
static void process_command(const DroneCommand_t *cmd)
{
    switch (cmd->type) {
    case CMD_ARM:
        if (s_state == DRONE_STATE_IDLE) {
            s_state = DRONE_STATE_ARMED;
            HAL_GPIO_WritePin(LED_ARMED_GPIO_Port, LED_ARMED_Pin, GPIO_PIN_SET);
        }
        break;
    case CMD_DISARM:
        MOTOR_SetAll(1000);
        s_state = DRONE_STATE_IDLE;
        HAL_GPIO_WritePin(LED_ARMED_GPIO_Port, LED_ARMED_Pin, GPIO_PIN_RESET);
        break;
    case CMD_TAKEOFF:
        if (s_state == DRONE_STATE_ARMED) {
            s_alt_setpoint = cmd->param1 > 0.0f ? cmd->param1 : HOME_ALTITUDE_M;
            s_state        = DRONE_STATE_TAKEOFF;
        }
        break;
    case CMD_LAND:
        s_alt_setpoint = 0.0f;
        s_state        = DRONE_STATE_DOCKING;
        break;
    case CMD_RETURN_HOME:
        s_state = DRONE_STATE_RETURN;
        break;
    case CMD_START_PATROL:
        load_default_waypoints();
        s_wp_index = 0;
        s_state    = DRONE_STATE_PATROL;
        break;
    case CMD_STOP_PATROL:
        s_state = DRONE_STATE_RETURN;
        break;
    case CMD_EMERGENCY_LAND:
        s_alt_setpoint = 0.0f;
        s_state        = DRONE_STATE_DOCKING;
        MOTOR_SetAll(1000);
        break;
    default:
        break;
    }
}

static void run_state_machine(void)
{
    switch (s_state) {

    case DRONE_STATE_TAKEOFF:
        if (s_sensor.altitude_baro >= s_alt_setpoint - 1.0f) {
            s_state = DRONE_STATE_PATROL;
            load_default_waypoints();
            s_wp_index = 0;
        }
        break;

    case DRONE_STATE_PATROL:
        if (s_waypoints.count == 0) break;
        {
            Waypoint_t *wp  = &s_waypoints.wp[s_wp_index];
            float dist      = haversine_distance(s_sensor.latitude,
                                                  s_sensor.longitude,
                                                  wp->latitude, wp->longitude);
            s_alt_setpoint  = wp->altitude;

            if (dist < 2.0f) {   /* within 2m of waypoint */
                s_wp_index = (s_wp_index + 1) % s_waypoints.count;
            }
            /* Heading toward waypoint */
            s_yaw_setpoint = bearing_to(s_sensor.latitude,
                                         s_sensor.longitude,
                                         wp->latitude, wp->longitude);
        }
        /* Battery check — autonomous RTH */
        if (s_sensor.battery_soc < LOW_BATTERY_THRESHOLD) {
            s_state = DRONE_STATE_RETURN;
        }
        break;

    case DRONE_STATE_RETURN:
        {
            /* Fly toward home (0,0 placeholder — replace with dock GPS) */
            float dist = haversine_distance(s_sensor.latitude,
                                             s_sensor.longitude,
                                             39.9135, 32.2851);
            if (dist < 1.5f) {
                s_state        = DRONE_STATE_DOCKING;
                s_alt_setpoint = DOCKING_ALTITUDE_M;
            }
        }
        break;

    case DRONE_STATE_DOCKING:
        if (s_sensor.altitude_baro < 0.3f) {
            MOTOR_SetAll(1000);
            s_state = DRONE_STATE_LANDED;
        }
        break;

    case DRONE_STATE_LANDED:
        s_state = DRONE_STATE_CHARGING;
        break;

    case DRONE_STATE_ERROR:
        MOTOR_SetAll(1000);
        break;

    default:
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  FLIGHT CONTROL TASK  — 500 Hz
 * ═══════════════════════════════════════════════════════════════ */
void vFlightControlTask(void *pvParameters)
{
    (void)pvParameters;

    MotorOutputs_t motors = {0};
    DroneCommand_t cmd;
    s_last_tick = xTaskGetTickCount();

    /* Arm ESCs */
    MOTOR_Init(&htim1);
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (;;) {
        /* Strict 2 ms period (500 Hz) */
        vTaskDelayUntil(&s_last_tick, pdMS_TO_TICKS(2));

        /* ── Read latest sensor data ── */
        xQueuePeek(xSensorDataQueue, &s_sensor, 0);

        /* ── Process any incoming commands ── */
        while (xQueueReceive(xCommandQueue, &cmd, 0) == pdTRUE) {
            process_command(&cmd);
        }

        /* ── Emergency checks ── */
        if (s_sensor.battery_soc < CRITICAL_BATTERY &&
            s_state != DRONE_STATE_LANDED &&
            s_state != DRONE_STATE_CHARGING) {
            s_state        = DRONE_STATE_DOCKING;
            s_alt_setpoint = 0.0f;
        }

        /* ── State machine ── */
        run_state_machine();

        /* ── PID + Motor mix ── */
        if (s_state >= DRONE_STATE_TAKEOFF &&
            s_state <= DRONE_STATE_DOCKING) {
            FC_ComputeMotorOutputs(&s_sensor,
                                    s_alt_setpoint,
                                    0.0f,
                                    0.0f,
                                    s_yaw_setpoint,
                                    &motors);
            MOTOR_SetPWM(&htim1, motors.m1, motors.m2,
                                  motors.m3, motors.m4);
        } else {
            MOTOR_SetAll(1000);
        }

        /* ── Flight time counter (every 500 ticks = 1s) ── */
        static uint16_t tick_counter = 0;
        if (++tick_counter >= 500) {
            tick_counter = 0;
            s_flight_time_s++;
        }
    }
}

/* Getters for telemetry task */
DroneState_t FC_GetState(void)      { return s_state; }
float        FC_GetAltSP(void)      { return s_alt_setpoint; }
uint32_t     FC_GetFlightTime(void) { return s_flight_time_s; }
