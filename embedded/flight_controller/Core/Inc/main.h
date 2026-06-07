/**
 * @file    main.h
 * @brief   ADSS Flight Controller — Global definitions
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ── Pin definitions ─────────────────────────────────────────── */
#define LED_STATUS_Pin          GPIO_PIN_12
#define LED_STATUS_GPIO_Port    GPIOD
#define LED_ARMED_Pin           GPIO_PIN_13
#define LED_ARMED_GPIO_Port     GPIOD
#define LED_ERROR_Pin           GPIO_PIN_14
#define LED_ERROR_GPIO_Port     GPIOD

#define RF_CS_Pin               GPIO_PIN_4
#define RF_CS_GPIO_Port         GPIOA
#define RF_CE_Pin               GPIO_PIN_3
#define RF_CE_GPIO_Port         GPIOA
#define RF_IRQ_Pin              GPIO_PIN_2
#define RF_IRQ_GPIO_Port        GPIOA

#define ARM_BUTTON_Pin          GPIO_PIN_0
#define ARM_BUTTON_GPIO_Port    GPIOC

#define BATTERY_ADC_Pin         GPIO_PIN_0
#define BATTERY_ADC_GPIO_Port   GPIOA

/* ── System constants ────────────────────────────────────────── */
#define ADSS_VERSION_MAJOR      1
#define ADSS_VERSION_MINOR      0
#define ADSS_VERSION_PATCH      0

#define FLIGHT_LOOP_HZ          500     /* 500 Hz flight control loop */
#define SENSOR_LOOP_HZ          200     /* 200 Hz sensor update        */
#define TELEMETRY_HZ            10      /* 10  Hz telemetry downlink   */
#define ACOUSTIC_SAMPLE_HZ      8000    /* 8   kHz acoustic sampling   */

#define MAX_DRONES              3
#define BATTERY_CELLS           4       /* 4S LiPo                     */
#define BATTERY_CAPACITY_MAH    5000
#define LOW_BATTERY_THRESHOLD   20.0f   /* % → return to dock          */
#define CRITICAL_BATTERY        10.0f   /* % → emergency land          */

#define MAX_ALTITUDE_M          120.0f  /* Legal limit AGL             */
#define HOME_ALTITUDE_M         40.0f   /* Default patrol altitude     */
#define DOCKING_ALTITUDE_M      1.5f    /* Final approach altitude     */

/* ── Drone flight states ─────────────────────────────────────── */
typedef enum {
    DRONE_STATE_IDLE        = 0,
    DRONE_STATE_ARMED,
    DRONE_STATE_TAKEOFF,
    DRONE_STATE_PATROL,
    DRONE_STATE_TRACKING,
    DRONE_STATE_RETURN,
    DRONE_STATE_DOCKING,
    DRONE_STATE_LANDED,
    DRONE_STATE_CHARGING,
    DRONE_STATE_ERROR
} DroneState_t;

/* ── Sensor data structure ───────────────────────────────────── */
typedef struct {
    /* IMU */
    float accel_x, accel_y, accel_z;   /* m/s²   */
    float gyro_x,  gyro_y,  gyro_z;    /* rad/s  */
    float roll, pitch, yaw;            /* degrees */
    /* Barometer */
    float pressure;                    /* hPa    */
    float temperature;                 /* °C     */
    float altitude_baro;               /* m AGL  */
    /* GPS */
    double latitude;
    double longitude;
    float  altitude_gps;               /* m MSL  */
    float  speed_gps;                  /* m/s    */
    float  heading_gps;                /* degrees */
    uint8_t gps_fix;
    uint8_t gps_satellites;
    /* Battery */
    float battery_voltage;             /* V      */
    float battery_current;             /* A      */
    float battery_soc;                 /* %      */
    /* Timestamp */
    uint32_t timestamp_ms;
} SensorData_t;

/* ── Drone command structure ─────────────────────────────────── */
typedef enum {
    CMD_NONE          = 0,
    CMD_ARM,
    CMD_DISARM,
    CMD_TAKEOFF,
    CMD_LAND,
    CMD_RETURN_HOME,
    CMD_GOTO_WAYPOINT,
    CMD_SET_ALTITUDE,
    CMD_EMERGENCY_LAND,
    CMD_START_PATROL,
    CMD_STOP_PATROL,
    CMD_TRACK_TARGET
} CommandType_t;

typedef struct {
    CommandType_t type;
    float param1;          /* altitude / lat / etc */
    float param2;          /* longitude            */
    float param3;          /* speed                */
    uint8_t drone_id;
    uint32_t timestamp_ms;
} DroneCommand_t;

/* ── Telemetry packet ────────────────────────────────────────── */
typedef struct {
    uint8_t  drone_id;
    uint8_t  state;
    float    latitude;
    float    longitude;
    float    altitude;
    float    speed;
    float    heading;
    float    battery_soc;
    float    battery_voltage;
    uint16_t latency_ms;
    uint8_t  gps_sats;
    uint8_t  signal_strength;
    uint32_t flight_time_s;
    uint32_t timestamp_ms;
    uint8_t  checksum;
} TelemetryPacket_t;

/* ── Global task handles (extern) ────────────────────────────── */
extern TaskHandle_t  xFlightTaskHandle;
extern TaskHandle_t  xSensorTaskHandle;
extern TaskHandle_t  xCommsTaskHandle;
extern TaskHandle_t  xAcousticTaskHandle;
extern TaskHandle_t  xPowerTaskHandle;
extern TaskHandle_t  xTelemetryTaskHandle;

extern QueueHandle_t xSensorDataQueue;
extern QueueHandle_t xCommandQueue;
extern QueueHandle_t xTelemetryQueue;

extern SemaphoreHandle_t xI2CMutex;
extern SemaphoreHandle_t xSPIMutex;
extern SemaphoreHandle_t xUARTMutex;
extern SemaphoreHandle_t xFlightStateMutex;

/* ── HAL handles (extern) ────────────────────────────────────── */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef  hi2c1;
extern I2C_HandleTypeDef  hi2c2;
extern SPI_HandleTypeDef  hspi1;
extern SPI_HandleTypeDef  hspi2;
extern TIM_HandleTypeDef  htim1;
extern TIM_HandleTypeDef  htim2;
extern ADC_HandleTypeDef  hadc1;
extern IWDG_HandleTypeDef hiwdg;

void Error_Handler(void);

#ifdef __cplusplus
}
#endif
#endif /* __MAIN_H */
