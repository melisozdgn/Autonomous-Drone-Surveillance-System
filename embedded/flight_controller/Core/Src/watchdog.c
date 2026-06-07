/**
 * @file    watchdog.c
 * @brief   Independent Watchdog (IWDG) + Software Task Monitor
 *
 * Two-level watchdog:
 *   1. Hardware IWDG: ~32s timeout → MCU reset if not kicked
 *   2. Software:      monitors each task heartbeat every 1s
 *      If any critical task misses 3 consecutive beats → emergency land
 */

#include "watchdog.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#define TASK_COUNT          3   /* flight, sensor, comms */
#define MAX_MISSED_BEATS    3

static IWDG_HandleTypeDef *s_hiwdg = NULL;
static volatile uint8_t s_task_beats[TASK_COUNT] = {0};
static uint8_t s_missed[TASK_COUNT] = {0};

void WDG_Init(IWDG_HandleTypeDef *hiwdg)
{
    s_hiwdg = hiwdg;
}

void WDG_Kick(void)
{
    HAL_IWDG_Refresh(s_hiwdg);
}

/* Called by each monitored task every cycle */
void WDG_TaskBeat(uint8_t task_id)
{
    if (task_id < TASK_COUNT)
        s_task_beats[task_id] = 1;
}

void vWatchdogTask(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));  /* Check every 1 second */

        /* Hardware watchdog kick */
        WDG_Kick();

        /* Software task monitor */
        for (uint8_t i = 0; i < TASK_COUNT; i++) {
            if (s_task_beats[i] == 0) {
                s_missed[i]++;
                if (s_missed[i] >= MAX_MISSED_BEATS) {
                    /* Critical task dead — emergency land */
                    DroneCommand_t cmd = {
                        .type        = CMD_EMERGENCY_LAND,
                        .drone_id    = 1,
                        .timestamp_ms = HAL_GetTick()
                    };
                    xQueueSend(xCommandQueue, &cmd, 0);
                    HAL_GPIO_WritePin(LED_ERROR_GPIO_Port,
                                      LED_ERROR_Pin, GPIO_PIN_SET);
                }
            } else {
                s_missed[i] = 0;
            }
            s_task_beats[i] = 0;  /* Reset for next window */
        }
    }
}
