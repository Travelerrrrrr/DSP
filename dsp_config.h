#ifndef DSP_CONFIG_H
#define DSP_CONFIG_H

#include "arm_math.h"

#define USE_ARM_FFT 1
#define FFT_LENGTH 2048U
#define ADC_RESOLUTION_BITS 12U
#define ADC_FULL_SCALE_VOLTAGE 3.3f

#define USE_CONVERT 1

#define USE_FILTER_CHECK 0
#define FILTER_CHECK_FLOAT 0

#endif // DSP_CONFIG_H
