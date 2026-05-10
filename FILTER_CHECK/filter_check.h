#ifndef FILTER_CHECK_H
#define FILTER_CHECK_H

#include <stdint.h>

#ifdef USE_FILTER_CHECK
typedef enum
{
    FILTER_NULL = 0,
    FILTER_FULLPASS = 1,
    FILTER_LOWPASS = 2,
    FILTER_HIGHPASS = 3,
    FILTER_BANDPASS = 4,
    FILTER_BANDSTOP = 5
} FILTER_TYPE;

typedef struct
{
    uint16_t pRais;
    uint16_t pFall;
    FILTER_TYPE type;
} FILTER_INFO;

FILTER_INFO filter_check(uint32_t addr, uint8_t Msize, uint16_t Length, float threshold, uint8_t BitWidth, float ADC_Ref);

#endif

#endif // FILTER_CHECK_H
