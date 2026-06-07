/**
 * @file    acoustic_tdoa.c
 * @brief   4-Microphone MEMS Array — TDOA Sound Source Localization
 *
 * Algorithm:
 *   1. Sample 4 MEMS mics at 8 kHz via I2S/ADC
 *   2. Compute GCC-PHAT cross-correlation for each mic pair
 *   3. Estimate TDOA from correlation peak
 *   4. Solve hyperbolic equations → 2D bearing to sound source
 *   5. Bayesian filter to smooth estimate over time
 *   6. Threshold detection → wake signal to flight controller
 *
 * Mic layout (top view, 50mm spacing):
 *         M0 (front)
 *          |
 *  M3 ----+---- M1
 *          |
 *         M2 (back)
 *
 * Reference: Shi et al. (2020) IEEE TVT
 */

#include "acoustic_tdoa.h"
#include "main.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ── Configuration ───────────────────────────────────────────── */
#define NUM_MICS            4
#define SAMPLE_RATE_HZ      8000
#define FRAME_SIZE          256       /* samples per frame    */
#define SPEED_OF_SOUND      343.0f    /* m/s @ 20°C           */
#define MIC_SPACING_M       0.050f    /* 50 mm                */
#define MAX_TDOA_SAMPLES    ((int)(MIC_SPACING_M / SPEED_OF_SOUND * SAMPLE_RATE_HZ) + 2)

/* Detection thresholds */
#define ENERGY_THRESHOLD    800.0f    /* RMS threshold         */
#define CONFIDENCE_MIN      0.65f     /* min GCC-PHAT peak     */

/* Bayesian filter */
#define BAYES_ALPHA         0.3f      /* smoothing factor      */

/* ── Private types ───────────────────────────────────────────── */
typedef struct {
    float re, im;
} Complex_t;

/* ── Private state ───────────────────────────────────────────── */
static int16_t  s_mic_buf[NUM_MICS][FRAME_SIZE];
static float    s_bearing_estimate = 0.0f;
static float    s_elevation_estimate = 0.0f;
static float    s_confidence = 0.0f;
static uint8_t  s_threat_detected = 0;
static AcousticEvent_t s_last_event = {0};

/* ── DFT (no external FFT lib — radix-2 for 256 pts) ────────── */
static void dft(const int16_t *x, Complex_t *X, int N)
{
    for (int k = 0; k < N; k++) {
        X[k].re = 0.0f;
        X[k].im = 0.0f;
        for (int n = 0; n < N; n++) {
            float angle = -2.0f * (float)M_PI * k * n / N;
            X[k].re += (float)x[n] * cosf(angle);
            X[k].im += (float)x[n] * sinf(angle);
        }
    }
}

/* ── GCC-PHAT cross-correlation ─────────────────────────────── */
static float gcc_phat(const int16_t *x1, const int16_t *x2,
                       int N, int *peak_idx)
{
    static Complex_t X1[FRAME_SIZE], X2[FRAME_SIZE], G[FRAME_SIZE];

    dft(x1, X1, N);
    dft(x2, X2, N);

    /* Cross-spectrum with PHAT weighting: G = X1*conj(X2) / |X1*conj(X2)| */
    for (int k = 0; k < N; k++) {
        float re = X1[k].re * X2[k].re + X1[k].im * X2[k].im;
        float im = X1[k].im * X2[k].re - X1[k].re * X2[k].im;
        float mag = sqrtf(re*re + im*im);
        if (mag < 1e-6f) mag = 1e-6f;
        G[k].re = re / mag;
        G[k].im = im / mag;
    }

    /* IDFT of G → correlation function */
    float corr[FRAME_SIZE];
    float max_val = 0.0f;
    int   max_k   = 0;
    for (int n = 0; n < N; n++) {
        float sum = 0.0f;
        for (int k = 0; k < N; k++) {
            float angle = 2.0f * (float)M_PI * k * n / N;
            sum += G[k].re * cosf(angle) - G[k].im * sinf(angle);
        }
        corr[n] = sum / N;
        if (corr[n] > max_val) {
            max_val = corr[n];
            max_k   = n;
        }
    }

    /* Convert index to lag (handle wrap-around) */
    if (max_k > N/2) max_k -= N;
    *peak_idx = max_k;

    return max_val;
}

/* ── RMS energy ──────────────────────────────────────────────── */
static float compute_rms(const int16_t *buf, int N)
{
    float sum = 0.0f;
    for (int i = 0; i < N; i++) sum += (float)buf[i] * buf[i];
    return sqrtf(sum / N);
}

/* ── TDOA → bearing ──────────────────────────────────────────── */
/*
 * Using mic pair M0-M2 (front-back, Y axis) and M1-M3 (right-left, X axis)
 * τ_y = TDOA(M0,M2) * speed_of_sound / sample_rate  [metres]
 * τ_x = TDOA(M1,M3) * speed_of_sound / sample_rate
 * bearing = atan2(τ_x, τ_y) in degrees
 */
static float tdoa_to_bearing(int tdoa_01, int tdoa_23)
{
    float dy = (float)tdoa_01 * SPEED_OF_SOUND / SAMPLE_RATE_HZ;
    float dx = (float)tdoa_23 * SPEED_OF_SOUND / SAMPLE_RATE_HZ;
    float bearing = atan2f(dx, dy) * 180.0f / (float)M_PI;
    return fmodf(bearing + 360.0f, 360.0f);
}

/* ═══════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ═══════════════════════════════════════════════════════════════ */
void ACOUSTIC_Init(void)
{
    memset(s_mic_buf, 0, sizeof(s_mic_buf));
    s_bearing_estimate   = 0.0f;
    s_elevation_estimate = 0.0f;
    s_confidence         = 0.0f;
    s_threat_detected    = 0;
}

/**
 * @brief  Process one frame from all 4 microphones.
 *         Called from vAcousticTask at ~SAMPLE_RATE_HZ / FRAME_SIZE Hz.
 * @param  mic_data  [NUM_MICS][FRAME_SIZE] PCM samples
 * @retval 1 if acoustic event detected, 0 otherwise
 */
uint8_t ACOUSTIC_ProcessFrame(const int16_t mic_data[NUM_MICS][FRAME_SIZE])
{
    /* ── Energy check ── */
    float rms = 0.0f;
    for (int m = 0; m < NUM_MICS; m++)
        rms += compute_rms(mic_data[m], FRAME_SIZE);
    rms /= NUM_MICS;

    if (rms < ENERGY_THRESHOLD) {
        s_threat_detected = 0;
        return 0;
    }

    /* ── GCC-PHAT for two orthogonal mic pairs ── */
    int tdoa_02, tdoa_13;
    float conf_02 = gcc_phat(mic_data[0], mic_data[2],
                              FRAME_SIZE, &tdoa_02);
    float conf_13 = gcc_phat(mic_data[1], mic_data[3],
                              FRAME_SIZE, &tdoa_13);

    float conf = (conf_02 + conf_13) / 2.0f;
    if (conf < CONFIDENCE_MIN) {
        s_threat_detected = 0;
        return 0;
    }

    /* ── Bearing estimation ── */
    float raw_bearing = tdoa_to_bearing(tdoa_02, tdoa_13);

    /* Bayesian (exponential) smoothing */
    s_bearing_estimate = BAYES_ALPHA * raw_bearing
                       + (1.0f - BAYES_ALPHA) * s_bearing_estimate;
    s_confidence       = conf;

    /* ── Populate event ── */
    s_last_event.bearing    = s_bearing_estimate;
    s_last_event.confidence = s_confidence;
    s_last_event.rms_energy = rms;
    s_last_event.timestamp  = HAL_GetTick();
    s_last_event.valid      = 1;

    s_threat_detected = 1;
    return 1;
}

uint8_t ACOUSTIC_ThreatDetected(void)      { return s_threat_detected; }
float   ACOUSTIC_GetBearing(void)          { return s_bearing_estimate; }
float   ACOUSTIC_GetConfidence(void)       { return s_confidence; }
const AcousticEvent_t *ACOUSTIC_GetEvent(void) { return &s_last_event; }

/* ── FreeRTOS task ───────────────────────────────────────────── */
void vAcousticTask(void *pvParameters)
{
    (void)pvParameters;
    static int16_t frame[NUM_MICS][FRAME_SIZE];
    DroneCommand_t cmd;

    for (;;) {
        /* In real HW: fill frame via I2S DMA callback */
        /* Here: simulate with placeholder              */
        vTaskDelay(pdMS_TO_TICKS(32));  /* ~31 Hz frame rate */

        uint8_t detected = ACOUSTIC_ProcessFrame(
            (const int16_t (*)[FRAME_SIZE])frame);

        if (detected) {
            /* Wake up cameras → send command to flight controller */
            cmd.type    = CMD_TRACK_TARGET;
            cmd.param1  = s_last_event.bearing;
            cmd.param2  = 0.0f;
            cmd.drone_id = 1;
            cmd.timestamp_ms = HAL_GetTick();
            xQueueSend(xCommandQueue, &cmd, 0);
        }
    }
}
