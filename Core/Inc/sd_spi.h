#ifndef __SD_SPI_H
#define __SD_SPI_H

#include <stdint.h>

/* Returns 0 on success, non-zero on error */
int  SdSpi_Init(void);
int  SdSpi_ReadBlock(uint32_t sector, uint8_t *buf);
int  SdSpi_WriteBlock(uint32_t sector, const uint8_t *buf);
int  SdSpi_Sync(void);
uint32_t SdSpi_GetSectorCount(void);

/* Current card status (0 = ready, non-zero = not initialised) */
int  SdSpi_Status(void);

#endif /* __SD_SPI_H */
