/**
 ******************************************************************************
 * @file    main.c
 * @brief   ADSS Flight Controller — STM32F407VGT6
 *          Autonomous Drone Surveillance System
 *          Atılım University · MECE 322 · Spring 2025-2026
 *
 * Hardware:
 *   MCU   : STM32F407VGT6 @ 168 MHz (Cortex-M4 + FPU)
 *   RTOS  : FreeRTOS v10.5.1
 *   Sensor: MPU-6050 IMU, BMP280 Barometer, NEO-M8N GPS
 *   Comms : UART (GPS), I2C (IMU/Baro), SPI (RF), PWM (ESC)
 ******************************************************************************
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "flight_controller.h"
#include "sensor_manager.h"
#include "pid_controller.h"
#include "motor_control.h"
#include "gps_driver.h"
#include "imu_driver.h"
#include "baro_driver.h"
#include "comms_rf.h"
#include "power_manager.h"
#include "acoustic_tdoa.h"
#include "docking_controller.h"
#include "watchdog.h"

/* ── Private defines ─────────────────────────────────────────── */
#define TASK_PRIORITY_FLIGHT     (configMAX_PRIORITIES - 1)   /* 5 */
#define TASK_PRIORITY_SENSOR     (configMAX_PRIORITIES - 2)   /* 4 */
#define TASK_PRIORITY_COMMS      (configMAX_PRIORITIES - 3)   /* 3 */
#define TASK_PRIORITY_ACOUSTIC   (configMAX_PRIORITIES - 3)   /* 3 */
#define TASK_PRIORITY_POWER      (configMAX_PRIORITIES - 4)   /* 2 */
#define TASK_PRIORITY_TELEMETRY  (configMAX_PRIORITIES - 4)   /* 2 */
#define TASK_PRIORITY_WATCHDOG   (configMAX_PRIORITIES - 5)   /* 1 */

#define STACK_SIZE_FLIGHT     512
#define STACK_SIZE_SENSOR     384
#define STACK_SIZE_COMMS      384
#define STACK_SIZE_ACOUSTIC   512
#define STACK_SIZE_POWER      256
#define STACK_SIZE_TELEMETRY  384
#define STACK_SIZE_WATCHDOG   128

/* ── Global handles ──────────────────────────────────────────── */
TaskHandle_t  xFlightTaskHandle     = NULL;
TaskHandle_t  xSensorTaskHandle     = NULL;
TaskHandle_t  xCommsTaskHandle      = NULL;
TaskHandle_t  xAcousticTaskHandle   = NULL;
TaskHandle_t  xPowerTaskHandle      = NULL;
TaskHandle_t  xTelemetryTaskHandle  = NULL;
TaskHandle_t  xWatchdogTaskHandle   = NULL;

QueueHandle_t xSensorDataQueue      = NULL;
QueueHandle_t xCommandQueue         = NULL;
QueueHandle_t xTelemetryQueue       = NULL;

SemaphoreHandle_t xI2CMutex         = NULL;
SemaphoreHandle_t xSPIMutex         = NULL;
SemaphoreHandle_t xUARTMutex        = NULL;
SemaphoreHandle_t xFlightStateMutex = NULL;

/* ── HAL handles ─────────────────────────────────────────────── */
UART_HandleTypeDef huart1;   /* GPS  — NEO-M8N  */
UART_HandleTypeDef huart2;   /* RF   — SiK 433  */
UART_HandleTypeDef huart3;   /* Debug / GCS USB */
I2C_HandleTypeDef  hi2c1;    /* IMU  — MPU-6050 */
I2C_HandleTypeDef  hi2c2;    /* Baro — BMP280   */
SPI_HandleTypeDef  hspi1;    /* RF NRF24L01     */
SPI_HandleTypeDef  hspi2;    /* FLIR Lepton SPI */
TIM_HandleTypeDef  htim1;    /* PWM ESC ch1-ch4 */
TIM_HandleTypeDef  htim2;    /* PWM Servo dock  */
ADC_HandleTypeDef  hadc1;    /* Battery voltage */
IWDG_HandleTypeDef hiwdg;    /* Watchdog        */

/* ── Private function prototypes ─────────────────────────────── */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
static void ADSS_CreateTasks(void);
static void ADSS_CreateQueues(void);
static void ADSS_CreateMutexes(void);
static void Error_Handler(void);

/* ── FreeRTOS Task prototypes ────────────────────────────────── */
void vFlightControlTask(void *pvParameters);
void vSensorManagerTask(void *pvParameters);
void vCommsTask(void *pvParameters);
void vAcousticTask(void *pvParameters);
void vPowerManagerTask(void *pvParameters);
void vTelemetryTask(void *pvParameters);
void vWatchdogTask(void *pvParameters);

/* ═══════════════════════════════════════════════════════════════
 *  MAIN
 * ═══════════════════════════════════════════════════════════════ */
int main(void)
{
    /* MCU init */
    HAL_Init();
    SystemClock_Config();

    /* Peripheral init */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_ADC1_Init();
    MX_IWDG_Init();

    /* Status LED ON during init */
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);

    /* Subsystem init */
    ADSS_CreateQueues();
    ADSS_CreateMutexes();

    FC_Init();
    SENSOR_Init(&hi2c1, &hi2c2, &hspi2);
    MOTOR_Init(&htim1);
    GPS_Init(&huart1);
    IMU_Init(&hi2c1);
    BARO_Init(&hi2c2);
    RF_Init(&hspi1, &huart2);
    POWER_Init(&hadc1);
    ACOUSTIC_Init();
    DOCK_Init(&htim2);
    WDG_Init(&hiwdg);

    /* Create RTOS tasks */
    ADSS_CreateTasks();

    /* Start scheduler — never returns */
    vTaskStartScheduler();

    /* Should never reach here */
    Error_Handler();
    while (1) {}
}

/* ─────────────────────────────────────────────────────────────── */

static void ADSS_CreateQueues(void)
{
    xSensorDataQueue = xQueueCreate(16, sizeof(SensorData_t));
    xCommandQueue    = xQueueCreate(8,  sizeof(DroneCommand_t));
    xTelemetryQueue  = xQueueCreate(32, sizeof(TelemetryPacket_t));

    configASSERT(xSensorDataQueue != NULL);
    configASSERT(xCommandQueue    != NULL);
    configASSERT(xTelemetryQueue  != NULL);
}

static void ADSS_CreateMutexes(void)
{
    xI2CMutex         = xSemaphoreCreateMutex();
    xSPIMutex         = xSemaphoreCreateMutex();
    xUARTMutex        = xSemaphoreCreateMutex();
    xFlightStateMutex = xSemaphoreCreateMutex();

    configASSERT(xI2CMutex         != NULL);
    configASSERT(xSPIMutex         != NULL);
    configASSERT(xUARTMutex        != NULL);
    configASSERT(xFlightStateMutex != NULL);
}

static void ADSS_CreateTasks(void)
{
    BaseType_t ret;

    ret = xTaskCreate(vFlightControlTask,  "FlightCtrl",
                      STACK_SIZE_FLIGHT,    NULL,
                      TASK_PRIORITY_FLIGHT, &xFlightTaskHandle);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(vSensorManagerTask,  "SensorMgr",
                      STACK_SIZE_SENSOR,    NULL,
                      TASK_PRIORITY_SENSOR, &xSensorTaskHandle);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(vCommsTask,          "Comms",
                      STACK_SIZE_COMMS,     NULL,
                      TASK_PRIORITY_COMMS,  &xCommsTaskHandle);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(vAcousticTask,        "Acoustic",
                      STACK_SIZE_ACOUSTIC,   NULL,
                      TASK_PRIORITY_ACOUSTIC,&xAcousticTaskHandle);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(vPowerManagerTask,   "PowerMgr",
                      STACK_SIZE_POWER,     NULL,
                      TASK_PRIORITY_POWER,  &xPowerTaskHandle);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(vTelemetryTask,      "Telemetry",
                      STACK_SIZE_TELEMETRY,  NULL,
                      TASK_PRIORITY_TELEMETRY,&xTelemetryTaskHandle);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(vWatchdogTask,        "Watchdog",
                      STACK_SIZE_WATCHDOG,   NULL,
                      TASK_PRIORITY_WATCHDOG,&xWatchdogTaskHandle);
    configASSERT(ret == pdPASS);
}

/* ─────────────────────────────────────────────────────────────── */

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Enable HSE oscillator, configure PLL to 168 MHz */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  |
                                       RCC_CLOCKTYPE_SYSCLK|
                                       RCC_CLOCKTYPE_PCLK1 |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Status LEDs */
    GPIO_InitStruct.Pin   = LED_STATUS_Pin | LED_ARMED_Pin | LED_ERROR_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

    /* RF chip-select (SPI1) */
    GPIO_InitStruct.Pin   = RF_CS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(RF_CS_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(RF_CS_GPIO_Port, RF_CS_Pin, GPIO_PIN_SET);

    /* Arm button input */
    GPIO_InitStruct.Pin  = ARM_BUTTON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ARM_BUTTON_GPIO_Port, &GPIO_InitStruct);
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA1 Stream6 — USART2 TX (RF) */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    /* DMA2 Stream0 — ADC1 (battery) */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void MX_USART1_UART_Init(void)
{
    /* GPS — NEO-M8N — 9600 baud default, upgrade to 115200 */
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

static void MX_USART2_UART_Init(void)
{
    /* RF SiK telemetry radio — 57600 baud */
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 57600;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

static void MX_USART3_UART_Init(void)
{
    /* Debug UART — 115200 baud */
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);
}

static void MX_I2C1_Init(void)
{
    /* IMU — MPU-6050 — 400 kHz Fast Mode */
    hi2c1.Instance              = I2C1;
    hi2c1.Init.ClockSpeed       = 400000;
    hi2c1.Init.DutyCycle        = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

static void MX_I2C2_Init(void)
{
    /* Barometer — BMP280 — 100 kHz */
    hi2c2.Instance              = I2C2;
    hi2c2.Init.ClockSpeed       = 100000;
    hi2c2.Init.DutyCycle        = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1      = 0;
    hi2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c2);
}

static void MX_SPI1_Init(void)
{
    /* NRF24L01 RF module */
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    HAL_SPI_Init(&hspi1);
}

static void MX_SPI2_Init(void)
{
    /* FLIR Lepton 3.5 thermal camera */
    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_MASTER;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize          = SPI_DATASIZE_16BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase          = SPI_PHASE_2EDGE;
    hspi2.Init.NSS               = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    HAL_SPI_Init(&hspi2);
}

static void MX_TIM1_Init(void)
{
    /* PWM for 4x ESC — 50 Hz, 1000-2000 µs pulse */
    TIM_OC_InitTypeDef sConfigOC = {0};
    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = (168000000 / 1000000) - 1; /* 1 MHz */
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 20000 - 1;  /* 20ms = 50Hz */
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    HAL_TIM_PWM_Init(&htim1);

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 1000;  /* 1ms = motor off */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);
}

static void MX_TIM2_Init(void)
{
    /* Docking servo / landing gear — 50 Hz */
    TIM_OC_InitTypeDef sConfigOC = {0};
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance           = TIM2;
    htim2.Init.Prescaler     = (84000000 / 1000000) - 1;
    htim2.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim2.Init.Period        = 20000 - 1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 1500;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
}

static void MX_ADC1_Init(void)
{
    /* Battery voltage divider — PA0 */
    ADC_ChannelConfTypeDef sConfig = {0};
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    HAL_ADC_Init(&hadc1);

    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
}

static void MX_IWDG_Init(void)
{
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Reload    = 4095;  /* ~32s timeout */
    HAL_IWDG_Init(&hiwdg);
}

void Error_Handler(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
    while (1) {}
}
