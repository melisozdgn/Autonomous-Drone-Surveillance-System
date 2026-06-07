/**
 * @file  flir_lepton.c
 * @brief FLIR Lepton 3.5 Thermal Camera — VoSPI Driver
 *
 * VoSPI: 164-byte packets (4 header + 160 data), 60 packets/segment, 4 segments
 * CCI  : I2C config for AGC, radiometry, telemetry
 * Output: 160x120 14-bit raw frame buffer
 */
#include "flir_lepton.h"
#include <string.h>

#define LEPTON_CS_LOW()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define LEPTON_CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define PKT_SIZE   164
#define PKTS_SEG   60
#define SEGS       4

static uint16_t s_frame[FLIR_ROWS][FLIR_COLS];
static uint8_t  s_pkt[PKT_SIZE];
static uint8_t  s_ready = 0;
static SPI_HandleTypeDef *s_spi = NULL;

static void cci_write(I2C_HandleTypeDef *hi2c, uint16_t reg, uint16_t val) {
    uint8_t b[4] = {reg>>8, reg&0xFF, val>>8, val&0xFF};
    HAL_I2C_Master_Transmit(hi2c, 0x2A<<1, b, 4, HAL_MAX_DELAY);
}

void FLIR_Init(SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c) {
    s_spi = hspi;
    GPIO_InitTypeDef g = {GPIO_PIN_12, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_Init(GPIOB, &g);
    LEPTON_CS_HIGH();
    HAL_Delay(1000);
    cci_write(hi2c, 0x0100, 0x0001); /* AGC enable */
    cci_write(hi2c, 0x0E00, 0x0000); /* Telemetry off */
}

uint8_t FLIR_CaptureFrame(void) {
    for (uint8_t seg = 0; seg < SEGS; seg++) {
        for (uint8_t p = 0; p < PKTS_SEG; p++) {
            uint32_t t = 5000;
            do {
                LEPTON_CS_LOW();
                HAL_SPI_Receive(s_spi, s_pkt, PKT_SIZE, HAL_MAX_DELAY);
                LEPTON_CS_HIGH();
                if (!t--) return 0;
            } while ((s_pkt[0] & 0x0F) == 0x0F);
            uint8_t row = s_pkt[1];
            if (row >= FLIR_ROWS) continue;
            for (uint8_t c = 0; c < FLIR_COLS; c++)
                s_frame[row][c] = ((uint16_t)s_pkt[4+c*2]<<8)|s_pkt[5+c*2];
        }
    }
    s_ready = 1;
    return 1;
}

uint8_t  FLIR_FrameReady(void)                    { return s_ready; }
uint16_t FLIR_GetPixel(uint8_t r, uint8_t c)      { return s_frame[r][c]; }
const uint16_t (*FLIR_GetFrame(void))[FLIR_COLS]  { return s_frame; }

uint8_t FLIR_FindHotspot(uint16_t thr, uint8_t *cx, uint8_t *cy, uint16_t *peak) {
    uint32_t sx=0, sy=0, cnt=0; *peak=0;
    for (uint8_t r=0;r<FLIR_ROWS;r++)
        for (uint8_t c=0;c<FLIR_COLS;c++) {
            if (s_frame[r][c]>thr) {sx+=c;sy+=r;cnt++;if(s_frame[r][c]>*peak)*peak=s_frame[r][c];}
        }
    if (!cnt) return 0;
    *cx=(uint8_t)(sx/cnt); *cy=(uint8_t)(sy/cnt);
    return 1;
}
