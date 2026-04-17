#include "timestamp.h"
#include "stm32g4xx_hal.h"

void Timestamp_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t Timestamp_Get(void)
{
    return DWT->CYCCNT;
}

/* Handles 32-bit wraparound (wraps every ~25.2 s at 170 MHz).
   Returns elapsed microseconds (unsigned, max ~25.2 s). */
uint32_t Timestamp_DeltaUs(uint32_t t_start, uint32_t t_end)
{
    return (uint32_t)((uint32_t)(t_end - t_start) / 170U);
}
