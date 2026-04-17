#ifndef __MPU9250_H
#define __MPU9250_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MPU9250_IMU1 = 0,
    MPU9250_IMU2 = 1,
} MPU9250_Id_t;

/* Returns true if WHO_AM_I == 0x71 and all registers written OK */
bool MPU9250_Init(MPU9250_Id_t id);

/* Assert / deassert chip-select (used by datalogger to bracket DMA) */
void MPU9250_AssertCS(MPU9250_Id_t id);
void MPU9250_DeassertCS(MPU9250_Id_t id);

#endif /* __MPU9250_H */
