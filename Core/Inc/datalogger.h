#ifndef __DATALOGGER_H
#define __DATALOGGER_H

#include <stdint.h>

/* Called once when entering Stage 2 */
void DataLogger_Start(void);

/* Called once when leaving Stage 2; flushes the partial buffer. */
void DataLogger_Stop(void);

/* Called from main loop — writes completed 512-byte buffer sectors to SD */
void DataLogger_MainLoopProcess(void);

/* Called from EXTI1 ISR (data-ready) */
void DataLogger_OnDataReady(void);

/* Called from HAL_SPI_TxRxCpltCallback when hspi1 completes */
void DataLogger_SPI1TxRxCplt(void);

/* Stats — safe to call from any context */
void DataLogger_GetStats(uint32_t *samples, uint32_t *overflows);

#endif /* __DATALOGGER_H */
