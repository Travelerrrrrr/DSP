#ifndef FILTER_CHECK_H
#define FILTER_CHECK_H

#include "dsp_config.h"
#include <stdint.h>

#ifndef USE_FILTER_CHECK
#define USE_FILTER_CHECK 0
#endif

#ifndef FILTER_CHECK_FLOAT
#define FILTER_CHECK_FLOAT 0
#endif

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
} _filter_info;

#if USE_FILTER_CHECK
_filter_info filter_check(uint32_t addr, uint8_t Msize, uint16_t Length, float threshold);
#endif

#endif // FILTER_CHECK_H
