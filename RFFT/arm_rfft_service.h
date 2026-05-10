#ifndef __ARM_RFFT_SERVICE_H
#define __ARM_RFFT_SERVICE_H

#include <stdint.h>
#include "arm_math.h"

#if USE_ARM_FFT

#ifndef FFT_LENGTH
#define FFT_LENGTH 2048U
#endif

#define RFFT_RESULT_LENGTH (FFT_LENGTH / 2U)
#define ADC_RESOLUTION_BITS 12U
#define ADC_FULL_SCALE_VOLTAGE 3.3f
#define ADC_FULL_SCALE_CODE ((1UL << ADC_RESOLUTION_BITS) - 1UL)
#define ADC_LSB_VOLTAGE (ADC_FULL_SCALE_VOLTAGE / (float32_t)ADC_FULL_SCALE_CODE)

/* 1 Vrms 对应的 ADC 码值，用作 dBV 的 0 dB 参考。 */
#define RFFT_DBV_REFERENCE_ADC_CODE (1.41421356237309f / ADC_LSB_VOLTAGE)
#define RFFT_DBFS_REFERENCE_V ADC_FULL_SCALE_VOLTAGE

typedef enum
{
	RFFT_WINDOW_RECTANGULAR = 0,
	RFFT_WINDOW_HANNING,
	RFFT_WINDOW_HAMMING,
	RFFT_WINDOW_BARTLETT,
	RFFT_WINDOW_WELCH,
	RFFT_WINDOW_NUTTALL3,
	RFFT_WINDOW_NUTTALL4,
	RFFT_WINDOW_BLACKMAN_HARRIS_92DB
} rfft_window_t;

typedef enum
{
	RFFT_UNIT_RAW = 0,
	RFFT_UNIT_DBV,
	RFFT_UNIT_DBFS
} rfft_unit_t;

typedef arm_status (*rfft_dma_transfer_func_t)(void *dma_handle,
											   uint32_t src_addr,
											   uint32_t dst_addr,
											   uint32_t size,
											   uint32_t timeout);

typedef struct
{
	void *handle;
	rfft_dma_transfer_func_t transfer;
} rfft_dma_t;

typedef struct
{
	arm_rfft_fast_instance_f32 fft_instance;
	rfft_dma_t dma;
	float32_t work_buffer[FFT_LENGTH];
	float32_t window_buffer[FFT_LENGTH];
	rfft_window_t cached_window;
	uint8_t window_ready;
	float32_t window_gain;
} rfft_handle_t;

arm_status rfft_handle_init(rfft_handle_t *hrfft, const rfft_dma_t *dma);

void rfft_start_af(rfft_handle_t *hrfft,
				   uint32_t adc_data_addr,
				   float32_t *result_data,
				   rfft_window_t window,
				   rfft_unit_t unit);

void rfft_start_pf(rfft_handle_t *hrfft,
				   uint32_t adc_data_addr,
				   float32_t *result_data,
				   rfft_window_t window);

void rfft_start_af_pf(rfft_handle_t *hrfft,
					  uint32_t adc_data_addr,
					  float32_t *result_data,
					  rfft_window_t window,
					  rfft_unit_t unit);

void RFFT_ERROR_HANDLER(void);

#endif // USE_ARM_FFT
#endif // __ARM_RFFT_SERVICE_H
