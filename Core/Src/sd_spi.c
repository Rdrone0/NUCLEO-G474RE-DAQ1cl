#include "sd_spi.h"
#include "main.h"
#include "spi.h"
#include "stm32g4xx_hal.h"

/* ------------------------------------------------------------------ */
/* SD command indices */
#define CMD0   0
#define CMD8   8
#define CMD9   9
#define CMD16  16
#define CMD17  17
#define CMD24  24
#define CMD55  55
#define CMD58  58
#define ACMD41 41

/* Data tokens */
#define SD_TOKEN_START_BLOCK  0xFEU
#define SD_DATA_ACCEPTED      0x05U

/* Timeouts (ms) */
#define SD_INIT_TIMEOUT_MS    1000U
#define SD_READ_TIMEOUT_MS    200U
#define SD_WRITE_TIMEOUT_MS   500U

#define SPI_TIMEOUT           100U

/* ------------------------------------------------------------------ */
static int   sd_ready   = 0;    /* 0 = not init, 1 = ready */
static int   sd_sdhc    = 0;    /* 1 = block addressing (SDHC/SDXC) */
static uint32_t sd_sectors = 0;

/* ------------------------------------------------------------------ */

static inline void CS_Assert(void)
{
    HAL_GPIO_WritePin(CS_SD_GPIO_Port, CS_SD_Pin, GPIO_PIN_RESET);
}

static inline void CS_Deassert(void)
{
    HAL_GPIO_WritePin(CS_SD_GPIO_Port, CS_SD_Pin, GPIO_PIN_SET);
}

static void SpiSend(uint8_t b)
{
    HAL_SPI_Transmit(&hspi2, &b, 1, SPI_TIMEOUT);
}

static uint8_t SpiRecv(void)
{
    uint8_t b = 0xFF;
    HAL_SPI_TransmitReceive(&hspi2, &b, &b, 1, SPI_TIMEOUT);
    return b;
}

/* Send a complete SD command frame (6 bytes) + up to 8 wait bytes for R1 */
static uint8_t SendCmd(uint8_t cmd, uint32_t arg)
{
    /* Precede with at least one idle byte while CS is high for CMD0 / CMD8 */
    SpiSend(0xFF);
    CS_Assert();
    SpiSend(0xFF);

    SpiSend(0x40 | cmd);
    SpiSend((uint8_t)(arg >> 24));
    SpiSend((uint8_t)(arg >> 16));
    SpiSend((uint8_t)(arg >>  8));
    SpiSend((uint8_t)(arg      ));

    /* CRC: 0x95 for CMD0, 0x87 for CMD8, 0x77 dummy for others */
    uint8_t crc = 0x01;
    if (cmd == CMD0)  crc = 0x95;
    if (cmd == CMD8)  crc = 0x87;
    SpiSend(crc);

    /* Wait for valid response (bit7 = 0), up to 8 bytes */
    uint8_t r = 0xFF;
    for (int i = 0; i < 8; i++) {
        r = SpiRecv();
        if (!(r & 0x80)) break;
    }
    return r;
}

static void EndCmd(void)
{
    CS_Deassert();
    SpiSend(0xFF); /* 8 extra clocks to complete card internal operation */
}

/* Wait for a non-0xFF byte (data token or error token) */
static uint8_t WaitToken(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t b;
    do {
        b = SpiRecv();
        if (b != 0xFF) return b;
    } while ((HAL_GetTick() - t0) < timeout_ms);
    return 0xFF;
}

/* Wait for card to leave busy state (returns 0 on success) */
static int WaitNotBusy(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        if (SpiRecv() == 0xFF) return 0;
    }
    return -1;
}

/* ------------------------------------------------------------------ */

/* Change SPI2 baud-rate prescaler at runtime (SPI must be idle) */
static void SetPrescaler(uint32_t prescaler_hal)
{
    /* STM32G4 classic SPI: baud rate is CR1[5:3] (BR[2:0]) */
    __HAL_SPI_DISABLE(&hspi2);
    MODIFY_REG(hspi2.Instance->CR1, SPI_CR1_BR, prescaler_hal);
    hspi2.Init.BaudRatePrescaler = prescaler_hal;
    __HAL_SPI_ENABLE(&hspi2);
}

/* ------------------------------------------------------------------ */

int SdSpi_Init(void)
{
    sd_ready = 0;

    /* Low-speed: prescaler 256 → 85 MHz / 256 = 332 kHz */
    SetPrescaler(SPI_BAUDRATEPRESCALER_256);

    CS_Deassert();

    /* ≥74 dummy clocks with CS high */
    for (int i = 0; i < 10; i++) SpiSend(0xFF);

    /* CMD0 — software reset */
    uint8_t r1 = SendCmd(CMD0, 0);
    EndCmd();
    if (r1 != 0x01) return -1;

    /* CMD8 — send interface condition (VHS=1, check=0xAA) */
    int v2 = 0;
    r1 = SendCmd(CMD8, 0x000001AA);
    if (r1 == 0x01) {
        /* Read remaining 4 bytes of R7 */
        uint8_t r7[4];
        r7[0] = SpiRecv(); r7[1] = SpiRecv();
        r7[2] = SpiRecv(); r7[3] = SpiRecv();
        EndCmd();
        if ((r7[2] & 0x0F) == 0x01 && r7[3] == 0xAA) v2 = 1;
    } else {
        EndCmd();
        /* v1 card or MMC — continue without v2 features */
    }

    /* ACMD41 — initialise, set HCS=1 for SDHC support */
    uint32_t t0 = HAL_GetTick();
    do {
        SendCmd(CMD55, 0); EndCmd();
        r1 = SendCmd(ACMD41, v2 ? 0x40000000 : 0);
        EndCmd();
        if (r1 == 0x00) break;
        HAL_Delay(1);
    } while ((HAL_GetTick() - t0) < SD_INIT_TIMEOUT_MS);

    if (r1 != 0x00) return -2;

    /* CMD58 — read OCR, check CCS bit */
    r1 = SendCmd(CMD58, 0);
    uint8_t ocr[4];
    ocr[0] = SpiRecv(); ocr[1] = SpiRecv();
    ocr[2] = SpiRecv(); ocr[3] = SpiRecv();
    EndCmd();
    sd_sdhc = (v2 && (ocr[0] & 0x40)) ? 1 : 0;

    /* SDSC: force 512-byte block length */
    if (!sd_sdhc) {
        r1 = SendCmd(CMD16, 512);
        EndCmd();
        if (r1 != 0x00) return -3;
    }

    /* Read CSD to get sector count */
    r1 = SendCmd(CMD9, 0);
    if (r1 == 0x00) {
        uint8_t tok = WaitToken(200);
        if (tok == SD_TOKEN_START_BLOCK) {
            uint8_t csd[16];
            for (int i = 0; i < 16; i++) csd[i] = SpiRecv();
            SpiRecv(); SpiRecv(); /* CRC */
            EndCmd();

            /* CSD v2 (SDHC): C_SIZE at bits [69:48] of 128-bit CSD */
            if ((csd[0] >> 6) == 1) {
                uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16)
                                | ((uint32_t)csd[8] << 8)
                                |  (uint32_t)csd[9];
                sd_sectors = (c_size + 1) * 1024;
            } else {
                /* CSD v1 */
                uint32_t c_size = ((uint32_t)(csd[6] & 0x03) << 10)
                                | ((uint32_t)csd[7] << 2)
                                | ((uint32_t)(csd[8] >> 6) & 0x03);
                uint32_t c_size_mult = ((csd[9] & 0x03) << 1) | (csd[10] >> 7);
                uint32_t blk_len = 1U << (csd[5] & 0x0F);
                sd_sectors = (c_size + 1) * (1U << (c_size_mult + 2)) * (blk_len / 512);
            }
        } else {
            EndCmd();
        }
    } else {
        EndCmd();
    }

    /* Switch to high speed: prescaler 4 → 85 MHz / 4 = 21.25 MHz */
    SetPrescaler(SPI_BAUDRATEPRESCALER_4);

    sd_ready = 1;
    return 0;
}

/* ------------------------------------------------------------------ */

int SdSpi_ReadBlock(uint32_t sector, uint8_t *buf)
{
    if (!sd_ready) return -1;

    uint32_t addr = sd_sdhc ? sector : sector * 512;
    uint8_t r1 = SendCmd(CMD17, addr);
    if (r1 != 0x00) { EndCmd(); return -2; }

    uint8_t tok = WaitToken(SD_READ_TIMEOUT_MS);
    if (tok != SD_TOKEN_START_BLOCK) { EndCmd(); return -3; }

    /* Read 512 data bytes */
    for (int i = 0; i < 512; i++) buf[i] = SpiRecv();

    SpiRecv(); SpiRecv(); /* CRC (discard) */
    EndCmd();
    return 0;
}

int SdSpi_WriteBlock(uint32_t sector, const uint8_t *buf)
{
    if (!sd_ready) return -1;

    uint32_t addr = sd_sdhc ? sector : sector * 512;
    uint8_t r1 = SendCmd(CMD24, addr);
    if (r1 != 0x00) { EndCmd(); return -2; }

    CS_Assert(); /* keep CS for data phase */

    SpiSend(0xFF);                   /* one idle byte before token */
    SpiSend(SD_TOKEN_START_BLOCK);   /* start token */

    for (int i = 0; i < 512; i++) SpiSend(buf[i]);

    SpiSend(0xFF); SpiSend(0xFF);    /* dummy CRC */

    /* Read data response */
    uint8_t resp = SpiRecv();
    if ((resp & 0x1F) != SD_DATA_ACCEPTED) {
        EndCmd();
        return -3;
    }

    /* Wait for write completion */
    if (WaitNotBusy(SD_WRITE_TIMEOUT_MS) != 0) {
        EndCmd();
        return -4;
    }

    EndCmd();
    return 0;
}

int SdSpi_Sync(void)
{
    if (!sd_ready) return 0;
    CS_Assert();
    int ret = WaitNotBusy(SD_WRITE_TIMEOUT_MS);
    CS_Deassert();
    SpiSend(0xFF);
    return ret;
}

uint32_t SdSpi_GetSectorCount(void)
{
    return sd_sectors;
}

int SdSpi_Status(void)
{
    return sd_ready ? 0 : 1;
}
