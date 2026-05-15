#ifndef DSP_CONFIG_H
#define DSP_CONFIG_H

#include "arm_math.h"

#define USE_ARM_FFT 1
#define FFT_LENGTH 1024U
#define ADC_RESOLUTION_BITS 12U
#define ADC_FULL_SCALE_VOLTAGE 3.3f

#define USE_CONVERT 1

#define USE_FILTER_CHECK 1
#define FILTER_CHECK_FLOAT 1

#endif // DSP_CONFIG_H