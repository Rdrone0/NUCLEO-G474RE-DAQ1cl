#include "state_machine.h"
#include "datalogger.h"
#include "mpu9250.h"
#include "timestamp.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "app_fatfs.h"
#include "stm32g4xx_nucleo.h"
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */

typedef enum {
    STATE_INIT = 0,
    STATE_READY,      /* Stage 1 done, slow blink, waiting for B1 */
    STATE_LOGGING,    /* Stage 2: acquiring + writing */
    STATE_FLUSHING,   /* Stage 3a: draining buffers */
    STATE_EJECTED,    /* Stage 3b: SD unmounted, safe to remove */
    STATE_ERROR,      /* Fatal: LED off, error on VCP */
} AppState_t;

static volatile AppState_t app_state = STATE_INIT;

/* Button debounce */
static volatile uint32_t last_press_tick = 0;
#define DEBOUNCE_MS  300U

/* Stage 3 flush request flag (set by button ISR, handled in main loop) */
static volatile uint8_t flush_request = 0;

/* VCP status print throttle */
static uint32_t last_status_tick = 0;
#define STATUS_INTERVAL_MS  1000U

/* ------------------------------------------------------------------ */
/* Low-level helpers */

static void VCP_Print(const char *msg)
{
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, (uint16_t)strlen(msg), 200);
}

static void VCP_Printf(const char *fmt, uint32_t val)
{
    char buf[64];
    snprintf(buf, sizeof(buf), fmt, val);
    VCP_Print(buf);
}

static void LED_SolidOn(void)
{
    HAL_TIM_Base_Stop_IT(&htim6);
    BSP_LED_On(LED_GREEN);
}

static void LED_SolidOff(void)
{
    HAL_TIM_Base_Stop_IT(&htim6);
    BSP_LED_Off(LED_GREEN);
}

static void LED_SlowBlink(void)
{
    /* ARR=4999, PSC=16999 → 170 MHz / 17000 / 5000 = 2 Hz toggle → 500 ms period */
    __HAL_TIM_SET_AUTORELOAD(&htim6, 4999);
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    BSP_LED_Off(LED_GREEN);
    HAL_TIM_Base_Start_IT(&htim6);
}

static void LED_FastBlink(void)
{
    /* ARR=999 → 170 MHz / 17000 / 1000 = 10 Hz toggle → 100 ms period */
    __HAL_TIM_SET_AUTORELOAD(&htim6, 999);
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    BSP_LED_Off(LED_GREEN);
    HAL_TIM_Base_Start_IT(&htim6);
}

/* ------------------------------------------------------------------ */

static int Stage1_Run(void)
{
    LED_SolidOff();
    VCP_Print("\r\nINIT...\r\n");

    /* Enable DWT timestamp counter */
    Timestamp_Init();

    /* Mount FatFs — triggers USER_initialize → SdSpi_Init() internally */
    FRESULT fr = f_mount(&USERFatFs, USERPath, 1);
    if (fr != FR_OK) {
        VCP_Printf("ERR: f_mount %lu\r\n", (uint32_t)fr);
        return -2;
    }

    /* Find a unique filename LOG_0000.BIN … LOG_9999.BIN */
    char fname[32];
    for (uint32_t n = 0; n < 10000; n++) {
        snprintf(fname, sizeof(fname), "%sLOG_%04lu.BIN", USERPath, n);
        FILINFO fi;
        if (f_stat(fname, &fi) == FR_NO_FILE) break;
    }
    fr = f_open(&USERFile, fname, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        VCP_Printf("ERR: f_open %lu\r\n", (uint32_t)fr);
        return -3;
    }
    VCP_Print("Log: ");
    VCP_Print(fname);
    VCP_Print("\r\n");

    /* Configure both IMUs (MPU9250_Init issues software reset internally) */
    if (!MPU9250_Init(MPU9250_IMU1)) {
        VCP_Print("ERR: IMU1 WHO_AM_I mismatch\r\n");
        return -4;
    }
    if (!MPU9250_Init(MPU9250_IMU2)) {
        VCP_Print("ERR: IMU2 WHO_AM_I mismatch\r\n");
        return -5;
    }
    VCP_Print("IMUs OK\r\n");

    VCP_Print("READY: Press B1 to start logging\r\n");
    LED_SlowBlink();
    return 0;
}

/* ------------------------------------------------------------------ */

void StateMachine_Init(void)
{
    app_state = STATE_INIT;
    if (Stage1_Run() != 0) {
        app_state = STATE_ERROR;
        LED_SolidOff();
        return;
    }
    app_state = STATE_READY;
}

void StateMachine_Process(void)
{
    /* Handle pending flush (set by button ISR while in LOGGING) */
    if (flush_request && app_state == STATE_FLUSHING) {
        flush_request = 0;

        LED_FastBlink();
        VCP_Print("Flushing...\r\n");

        /* Stop PWM (FSYNC) */
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

        /* DataLogger_Stop writes remaining partial buffer and pending sector */
        DataLogger_Stop();

        /* Sync and close file */
        f_sync(&USERFile);
        f_close(&USERFile);
        f_mount(NULL, USERPath, 0);

        LED_SolidOff();
        app_state = STATE_EJECTED;
        VCP_Print("Safe to remove SD. Press B1 to re-init.\r\n");
        return;
    }

    /* Periodic VCP status during logging */
    if (app_state == STATE_LOGGING) {
        uint32_t now = HAL_GetTick();
        if ((now - last_status_tick) >= STATUS_INTERVAL_MS) {
            last_status_tick = now;
            uint32_t n, ov;
            DataLogger_GetStats(&n, &ov);
            VCP_Printf("Samples: %lu", n);
            VCP_Printf("  OVF: %lu\r\n", ov);
        }
    }

    /* Re-init after eject */
    if (app_state == STATE_INIT) {
        if (Stage1_Run() != 0) {
            app_state = STATE_ERROR;
            LED_SolidOff();
        } else {
            app_state = STATE_READY;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Called from HAL_GPIO_EXTI_Callback (ISR context) */
void StateMachine_OnButton(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - last_press_tick) < DEBOUNCE_MS) return;
    last_press_tick = now;

    switch (app_state) {
        case STATE_READY:
            app_state = STATE_LOGGING;
            LED_SolidOn();
            last_status_tick = HAL_GetTick();
            DataLogger_Start();
            /* Start 1 kHz FSYNC PWM */
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
            VCP_Print("LOGGING\r\n");
            break;

        case STATE_LOGGING:
            app_state = STATE_FLUSHING;
            flush_request = 1;
            /* Actual flush happens in StateMachine_Process() */
            break;

        case STATE_EJECTED:
            app_state = STATE_INIT;
            /* Stage1_Run() will be called from StateMachine_Process() */
            break;

        default:
            break;
    }
}

/* ------------------------------------------------------------------ */
/* HAL callbacks — override weak defaults */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_1) {
        /* PB1 — INT_IMU1 data-ready */
        if (app_state == STATE_LOGGING) {
            DataLogger_OnDataReady();
        }
    } else if (GPIO_Pin == GPIO_PIN_13) {
        /* PC13 — B1 user button */
        StateMachine_OnButton();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        BSP_LED_Toggle(LED_GREEN);
    }
}
