#include "mpu9250.h"
#include "main.h"
#include "spi.h"

/* MPU-9250 register map (subset) */
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_INT_PIN_CFG  0x37
#define REG_INT_ENABLE   0x38
#define REG_USER_CTRL    0x6A
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

#define WHO_AM_I_VAL     0x71
#define SPI_TIMEOUT_MS   10U

/* ------------------------------------------------------------------ */

void MPU9250_AssertCS(MPU9250_Id_t id)
{
    if (id == MPU9250_IMU1)
        HAL_GPIO_WritePin(CS_IMU1_GPIO_Port, CS_IMU1_Pin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(CS_IMU2_GPIO_Port, CS_IMU2_Pin, GPIO_PIN_RESET);
}

void MPU9250_DeassertCS(MPU9250_Id_t id)
{
    if (id == MPU9250_IMU1)
        HAL_GPIO_WritePin(CS_IMU1_GPIO_Port, CS_IMU1_Pin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(CS_IMU2_GPIO_Port, CS_IMU2_Pin, GPIO_PIN_SET);
}

/* Blocking 2-byte register write */
static void WriteReg(MPU9250_Id_t id, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg & 0x7F, val };
    MPU9250_AssertCS(id);
    HAL_SPI_Transmit(&hspi1, buf, 2, SPI_TIMEOUT_MS);
    MPU9250_DeassertCS(id);
}

/* Blocking single-byte register read */
static uint8_t ReadReg(MPU9250_Id_t id, uint8_t reg)
{
    uint8_t tx[2] = { reg | 0x80, 0xFF };
    uint8_t rx[2] = { 0, 0 };
    MPU9250_AssertCS(id);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, SPI_TIMEOUT_MS);
    MPU9250_DeassertCS(id);
    return rx[1];
}

/* ------------------------------------------------------------------ */

bool MPU9250_Init(MPU9250_Id_t id)
{
    /* Hardware reset */
    WriteReg(id, REG_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    /* Auto-select best available clock */
    WriteReg(id, REG_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    /* Disable I2C (SPI-only mode) */
    WriteReg(id, REG_USER_CTRL, 0x10);

    /* 1 kHz output data rate: SMPLRT_DIV = 0 (ODR = 1000 / (1+0) = 1000 Hz) */
    WriteReg(id, REG_SMPLRT_DIV, 0x00);

    /* DLPF_CFG = 1 (184 Hz BW), EXT_SYNC_SET = 1 (FSYNC latched into TEMP LSB) */
    WriteReg(id, REG_CONFIG, 0x09);

    /* Gyro: ±250 °/s */
    WriteReg(id, REG_GYRO_CONFIG, 0x00);

    /* Accel: ±2 g */
    WriteReg(id, REG_ACCEL_CONFIG, 0x00);

    /* INT pin: active high, push-pull, 50µs pulse, cleared on any read */
    WriteReg(id, REG_INT_PIN_CFG, 0x10);

    /* Enable data-ready interrupt */
    WriteReg(id, REG_INT_ENABLE, 0x01);

    /* Verify identity */
    uint8_t who = ReadReg(id, REG_WHO_AM_I);
    return (who == WHO_AM_I_VAL);
}
