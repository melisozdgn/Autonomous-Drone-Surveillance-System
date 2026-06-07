/**
 * @file    acoustic_tdoa.h
 */
#ifndef __ACOUSTIC_TDOA_H
#define __ACOUSTIC_TDOA_H
#include <stdint.h>
#define ACOUSTIC_NUM_MICS   4
#define ACOUSTIC_FRAME_SIZE 256
typedef struct {
    float    bearing;
    float    confidence;
    float    rms_energy;
    uint32_t timestamp;
    uint8_t  valid;
} AcousticEvent_t;
void    ACOUSTIC_Init(void);
uint8_t ACOUSTIC_ProcessFrame(const int16_t mic_data[ACOUSTIC_NUM_MICS][ACOUSTIC_FRAME_SIZE]);
uint8_t ACOUSTIC_ThreatDetected(void);
float   ACOUSTIC_GetBearing(void);
float   ACOUSTIC_GetConfidence(void);
const AcousticEvent_t *ACOUSTIC_GetEvent(void);
void    vAcousticTask(void *pvParameters);
#endif
