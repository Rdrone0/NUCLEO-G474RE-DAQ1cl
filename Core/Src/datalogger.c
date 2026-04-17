#include "datalogger.h"
#include "mpu9250.h"
#include "timestamp.h"
#include "main.h"
#include "spi.h"
#include "app_fatfs.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/* Ping-pong: two 512-byte sectors, each holds 16 × 32-byte samples */
#define BUF_SIZE    512U
#define SAMPLE_SIZE  32U  /* 4 timestamp + 14 IMU1 + 14 IMU2 */

static uint8_t ping_pong[2][BUF_SIZE];
static volatile uint8_t  active_buf;     /* buffer being filled  */
static volatile uint16_t buf_pos;        /* byte offset in active */
static volatile uint8_t  write_pending;  /* 1 when a sector is ready */
static volatile uint8_t  write_buf_idx;  /* index of sector to write */

/* DMA I/O buffers (must persist across DMA transfer) */
static uint8_t imu_tx[15]; /* [0]=reg|0x80, [1..14]=0xFF */
static uint8_t imu1_rx[15];
static uint8_t imu2_rx[15];

/* Sequential read state */
typedef enum { DL_IDLE = 0, DL_IMU1, DL_IMU2 } DL_SpiState_t;
static volatile DL_SpiState_t spi_state;

static volatile uint32_t timestamp_raw;  /* captured at EXTI1 */
static volatile uint32_t sample_count;
static volatile uint32_t overflow_count;

/* ------------------------------------------------------------------ */

void DataLogger_Start(void)
{
    /* Zero all state */
    active_buf    = 0;
    buf_pos       = 0;
    write_pending = 0;
    write_buf_idx = 0;
    spi_state     = DL_IDLE;
    sample_count  = 0;
    overflow_count = 0;

    /* Build constant TX buffer for 15-byte burst read from 0x3B */
    imu_tx[0] = 0x80 | 0x3B;  /* ACCEL_XOUT_H, read flag */
    memset(&imu_tx[1], 0xFF, 14);

    /* Enable EXTI1 (INT_IMU1, PB1) — priority 1 */
    HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void DataLogger_Stop(void)
{
    /* Disable further data-ready triggers */
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);

    /* Wait for any in-flight DMA to finish */
    uint32_t t0 = HAL_GetTick();
    while (spi_state != DL_IDLE && (HAL_GetTick() - t0) < 50) { /* spin */ }

    /* Flush partial buffer if it has data */
    if (buf_pos > 0) {
        /* Zero-pad remaining bytes so the sector is clean */
        memset(&ping_pong[active_buf][buf_pos], 0, BUF_SIZE - buf_pos);
        UINT bw;
        f_write(&USERFile, ping_pong[active_buf], BUF_SIZE, &bw);
    }

    /* Write any sector that was waiting */
    if (write_pending) {
        UINT bw;
        f_write(&USERFile, ping_pong[write_buf_idx], BUF_SIZE, &bw);
        write_pending = 0;
    }
}

/* ------------------------------------------------------------------ */
/* Called from EXTI1 ISR — priority 1, timing critical */
void DataLogger_OnDataReady(void)
{
    if (spi_state != DL_IDLE) {
        overflow_count++;
        return;
    }

    timestamp_raw = Timestamp_Get();
    spi_state = DL_IMU1;

    MPU9250_AssertCS(MPU9250_IMU1);
    HAL_SPI_TransmitReceive_DMA(&hspi1, imu_tx, imu1_rx, 15);
}

/* ------------------------------------------------------------------ */
/* Called from HAL_SPI_TxRxCpltCallback when hspi1 completes — ISR */
void DataLogger_SPI1TxRxCplt(void)
{
    if (spi_state == DL_IMU1) {
        MPU9250_DeassertCS(MPU9250_IMU1);
        spi_state = DL_IMU2;
        MPU9250_AssertCS(MPU9250_IMU2);
        HAL_SPI_TransmitReceive_DMA(&hspi1, imu_tx, imu2_rx, 15);
        return;
    }

    if (spi_state == DL_IMU2) {
        MPU9250_DeassertCS(MPU9250_IMU2);
        spi_state = DL_IDLE;

        /* Pack 32-byte sample into active ping-pong buffer */
        uint8_t *dst = &ping_pong[active_buf][buf_pos];

        /* 4-byte timestamp (little-endian) */
        uint32_t ts = timestamp_raw;
        dst[0] = (uint8_t)(ts      );
        dst[1] = (uint8_t)(ts >>  8);
        dst[2] = (uint8_t)(ts >> 16);
        dst[3] = (uint8_t)(ts >> 24);

        /* 14 bytes from each IMU (rx[1..14], skip dummy rx[0]) */
        memcpy(&dst[4],  &imu1_rx[1], 14);
        memcpy(&dst[18], &imu2_rx[1], 14);

        buf_pos += SAMPLE_SIZE;
        sample_count++;

        if (buf_pos >= BUF_SIZE) {
            if (!write_pending) {
                write_buf_idx = active_buf;
                write_pending = 1;
            } else {
                overflow_count++; /* main loop too slow */
            }
            active_buf ^= 1;
            buf_pos = 0;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Called from main loop */
void DataLogger_MainLoopProcess(void)
{
    if (!write_pending) return;

    uint8_t idx = write_buf_idx;
    write_pending = 0;  /* clear before f_write so ISR can set it again */

    UINT bw;
    f_write(&USERFile, ping_pong[idx], BUF_SIZE, &bw);
    if (bw != BUF_SIZE) overflow_count++;
}

void DataLogger_GetStats(uint32_t *samples, uint32_t *overflows)
{
    *samples   = sample_count;
    *overflows = overflow_count;
}

/* ------------------------------------------------------------------ */
/* HAL SPI callbacks — override weak defaults */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        DataLogger_SPI1TxRxCplt();
    }
    /* SPI2 (SD card) uses polling mode, no DMA callback needed */
}
