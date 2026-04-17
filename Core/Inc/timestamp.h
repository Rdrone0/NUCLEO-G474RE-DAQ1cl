#ifndef __TIMESTAMP_H
#define __TIMESTAMP_H

#include <stdint.h>

void     Timestamp_Init(void);
uint32_t Timestamp_Get(void);
uint32_t Timestamp_DeltaUs(uint32_t t_start, uint32_t t_end);

#endif /* __TIMESTAMP_H */
