/**
 * @file  mems_mic.c
 * @brief 4x MEMS Microphone Array Driver (IMP34DT05 / MP34DT01)
 *
 * Interface: I2S (PDM) — DMA double-buffer mode
 * Layout   : 50 mm cross array (front, right, back, left)
 * SNR      : 64 dBSPL | Sensitivity: -26 dBFS
 * Used for : TDOA acoustic source localization
 */
#include "mems_mic.h"
#include <math.h>
#include <string.h>

#define PDM_BUF_SIZE  (MIC_FRAME_SIZE * MIC_COUNT / 8)  /* PDM raw bytes */

static I2S_HandleTypeDef *s_hi2s = NULL;
static uint16_t s_pdm_buf[2][PDM_BUF_SIZE];   /* double buffer */
static int16_t  s_pcm[MIC_COUNT][MIC_FRAME_SIZE];
static uint8_t  s_buf_ready = 0;

/* PDM → PCM decimation filter (CIC order 5, decimation 16) */
static int16_t pdm_to_pcm_sample(uint8_t pdm_byte)
{
    int32_t acc = 0;
    for (int b = 7; b >= 0; b--)
        acc += ((pdm_byte >> b) & 1) ? 1 : -1;
    return (int16_t)(acc * 4096 / 8);
}

HAL_StatusTypeDef MEMS_MIC_Init(I2S_HandleTypeDef *hi2s) {
    s_hi2s = hi2s;
    memset(s_pcm, 0, sizeof(s_pcm));
    /* Start DMA circular receive */
    return HAL_I2S_Receive_DMA(hi2s, s_pdm_buf[0], PDM_BUF_SIZE);
}

/* Called from DMA half/full complete ISR */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    (void)hi2s;
    s_buf_ready = 1;
}
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
    (void)hi2s;
    s_buf_ready = 2;
}

uint8_t MEMS_MIC_ReadFrame(int16_t out[MIC_COUNT][MIC_FRAME_SIZE]) {
    if (!s_buf_ready) return 0;
    uint8_t idx = s_buf_ready - 1;
    s_buf_ready = 0;

    /* Demux 4 mic PDM streams and convert to PCM */
    uint8_t *raw = (uint8_t *)s_pdm_buf[idx];
    for (int i = 0; i < MIC_FRAME_SIZE; i++) {
        for (int m = 0; m < MIC_COUNT; m++)
            out[m][i] = pdm_to_pcm_sample(raw[i * MIC_COUNT + m]);
    }
    if (out) memcpy(s_pcm, out, sizeof(s_pcm));
    return 1;
}

float MEMS_MIC_GetRMS(uint8_t mic_id) {
    if (mic_id >= MIC_COUNT) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < MIC_FRAME_SIZE; i++)
        sum += (float)s_pcm[mic_id][i] * s_pcm[mic_id][i];
    return sqrtf(sum / MIC_FRAME_SIZE);
}
