/**
 * @file arm_rfft_service.c
 * @brief 基于 CMSIS-DSP 的实数 FFT 服务层实现。
 * @author Analog
 * @version 1.5
 * @date 2026-05-11
 * @note 本文件负责完成 ADC 原始数据搬运、数值格式转换、窗函数处理、
 *       RFFT 计算，以及单边幅频/相频结果提取。
 */

#include "../User/DSP/arm_rfft_service.h"
#include <string.h>

#define RFFT_DMA_TIMEOUT 0xFFFFU
#define RFFT_LOG_FLOOR 1.0e-20f
#define RFFT_LOG_CEILING 1.0e20f
#define RFFT_LN_TO_20LOG10 8.68588963806503f
#define RFFT_INV_SQRT2 0.70710678118654f
#define RFFT_PI 3.14159265358979f
#define RFFT_TWO_PI 6.28318530717959f
#define RFFT_INV_TWO_PI 0.15915494309190f
#define RFFT_RAD_TO_DEG 57.29577951308232f
#define RFFT_DEG_TO_RAD 0.01745329251994f

static float32_t rfft_wrap_phase_rad(float32_t phase)
{
	int32_t turns = (int32_t)(phase * RFFT_INV_TWO_PI);

	phase -= (float32_t)turns * RFFT_TWO_PI;

	if (phase > RFFT_PI)
	{
		phase -= RFFT_TWO_PI;
	}
	else if (phase < -RFFT_PI)
	{
		phase += RFFT_TWO_PI;
	}

	return phase;
}

/**
 * @brief 将工作缓冲区中的 unsigned 32-bit 原始采样原地转换为 float32 数据。
 * @param[in,out] Src 工作缓冲区指针；转换前按 uint32_t 解释，转换后按 float32_t 解释。
 * @param[in] length 需要转换的数据点数。
 * @note 源数据和目标数据均为 32 bit 宽度，因此可以安全地正序原地转换。
 */
static void convert_unsigned_to_float(float32_t *Src, uint32_t length)
{
	uint16_t i;
	uint32_t *buf = (uint32_t *)Src;
	for (i = 0; i < length; i++)
	{
		Src[i] = (float32_t)buf[i];
	}
}

/**
 * @brief 生成并缓存 RFFT 窗函数及其幅值校正增益。
 * @param[in,out] hrfft RFFT 句柄。
 * @param[in] window 需要使用的窗函数类型。
 * @note 若当前缓存的窗函数类型未变化，则直接复用已有 window_buffer 和 window_gain。
 */
static void rfft_generate_window(rfft_handle_t *hrfft, rfft_window_t window)
{
	if ((hrfft->window_ready != 0U) && (hrfft->cached_window == window))
	{
		return;
	}

	switch (window)
	{
	case RFFT_WINDOW_HANNING:
		arm_hanning_f32(hrfft->window_buffer, FFT_LENGTH);
		break;
	case RFFT_WINDOW_HAMMING:
		arm_hamming_f32(hrfft->window_buffer, FFT_LENGTH);
		break;
	case RFFT_WINDOW_BARTLETT:
		arm_bartlett_f32(hrfft->window_buffer, FFT_LENGTH);
		break;
	case RFFT_WINDOW_WELCH:
		arm_welch_f32(hrfft->window_buffer, FFT_LENGTH);
		break;
	case RFFT_WINDOW_NUTTALL3:
		arm_nuttall3_f32(hrfft->window_buffer, FFT_LENGTH);
		break;
	case RFFT_WINDOW_NUTTALL4:
		arm_nuttall4_f32(hrfft->window_buffer, FFT_LENGTH);
		break;
	case RFFT_WINDOW_BLACKMAN_HARRIS_92DB:
		arm_blackman_harris_92db_f32(hrfft->window_buffer, FFT_LENGTH);
		break;
	case RFFT_WINDOW_RECTANGULAR:
	default:
		arm_fill_f32(1.0f, hrfft->window_buffer, FFT_LENGTH);
		break;
	}

	arm_mean_f32(hrfft->window_buffer, FFT_LENGTH, &hrfft->window_gain);
	if (hrfft->window_gain <= 0.0f)
	{
		hrfft->window_gain = 1.0f;
	}

	hrfft->cached_window = window;
	hrfft->window_ready = 1U;
}

/**
 * @brief 将单边幅频结果转换为指定单位。
 * @param[in,out] result_data 幅频结果数组，长度至少为 RFFT_RESULT_LENGTH。
 * @param[in] unit 目标单位，可选择 RAW、dBV 或 dBFS。
 * @note RAW 模式不做转换；dB 模式下执行 20log10(x/reference)，并对输入下限做裁剪以避免 log(0)。
 */
static void rfft_convert_unit(float32_t *result_data, rfft_unit_t unit)
{
	float32_t reference = 1.0f;

	switch (unit)
	{
	case RFFT_UNIT_DBV:
		arm_scale_f32(&result_data[1], RFFT_INV_SQRT2, &result_data[1], RFFT_RESULT_LENGTH - 1U);
		reference = RFFT_DBV_REFERENCE_ADC_CODE;
		break;
	case RFFT_UNIT_DBFS:
		reference = RFFT_DBFS_REFERENCE_V;
		break;
	case RFFT_UNIT_RAW:
	default:
		// 线性电压单位不需要做 dB 转换
		return;
	}

	arm_scale_f32(result_data, 1.0f / reference, result_data, RFFT_RESULT_LENGTH);				  // 20log10(x/reference)
	arm_clip_f32(result_data, result_data, RFFT_LOG_FLOOR, RFFT_LOG_CEILING, RFFT_RESULT_LENGTH); // 仅为O(N)，为保证系统稳定，必须保留
	arm_vlog_f32(result_data, result_data, RFFT_RESULT_LENGTH);
	arm_scale_f32(result_data, RFFT_LN_TO_20LOG10, result_data, RFFT_RESULT_LENGTH); // 20log10(x) = ln(x) * 20/ln(10)
}

/**
 * @brief 从 RFFT 复数结果中提取并归一化单边幅频结果。
 * @param[in] hrfft RFFT 句柄。
 * @param[out] result_data 输出幅频结果，长度至少为 RFFT_RESULT_LENGTH。
 * @note 非直流频点按单边谱幅值缩放，直流分量额外乘以 0.5 以避免被双边合并系数放大。
 */
static void rfft_make_single_sided_amplitude(rfft_handle_t *hrfft, float32_t *result_data)
{
	float32_t scale = 2.0f / ((float32_t)FFT_LENGTH * hrfft->window_gain);

	arm_abs_f32(&hrfft->work_buffer[0], &result_data[0], 1U);
	arm_cmplx_mag_f32(&hrfft->work_buffer[2], &result_data[1], RFFT_RESULT_LENGTH - 1U);
	arm_scale_f32(result_data, scale, result_data, RFFT_RESULT_LENGTH);
	// arm_scale_f32(&result_data[0], 0.5f, &result_data[0], 1U);
	result_data[0] *= 0.5f;
}

/**
 * @brief 从 RFFT 复数结果中提取单边相频结果。
 * @param[in] hrfft RFFT 句柄。
 * @param[out] result_data 输出相频结果，长度至少为 RFFT_RESULT_LENGTH。
 * @param[in] unit 相位输出单位选择；0 表示弧度，非 0 表示角度。
 * @note 直流相位固定为 0，其余频点由 arm_atan2_f32 计算。
 */
static void rfft_make_single_sided_phase(rfft_handle_t *hrfft, float32_t *result_data, uint8_t unit)
{
	uint32_t i;
	const float32_t *p = &hrfft->work_buffer[2];

	result_data[0] = 0.0f;

	if (unit)
	{
		// 弧度->角度缩放融入循环，省一次 arm_scale_f32 的 O(N) 遍历
		float32_t phase;
		for (i = 1U; i < RFFT_RESULT_LENGTH; i++)
		{
			arm_atan2_f32(p[1], p[0], &phase);
			result_data[i] = phase * RFFT_RAD_TO_DEG;
			p += 2;
		}
	}
	else
	{
		for (i = 1U; i < RFFT_RESULT_LENGTH; i++)
		{
			arm_atan2_f32(p[1], p[0], &result_data[i]);
			p += 2;
		}
	}
}

/**
 * @brief 同时从 RFFT 复数结果中提取单边幅频和相频结果。
 * @param[in] hrfft RFFT 句柄。
 * @param[out] result_data 输出数组，长度至少为 2 * RFFT_RESULT_LENGTH。
 * @param[in] unit 相位输出单位选择；0 表示弧度，非 0 表示角度。
 * @note result_data 前半段保存归一化幅值，后半段保存相位；直流相位固定为 0。
 */
static void rfft_make_single_sided_amplitude_phase(rfft_handle_t *hrfft, float32_t *result_data, uint8_t unit)
{
	uint32_t i;
	const float32_t *p = &hrfft->work_buffer[2];
	const float32_t scale = 2.0f / ((float32_t)FFT_LENGTH * hrfft->window_gain);
	float32_t mag;
	float32_t phase;

	result_data[RFFT_RESULT_LENGTH] = 0.0f;

	// DC 幅值：单边谱 DC 不翻倍，scale*0.5 一次性应用
	arm_abs_f32(&hrfft->work_buffer[0], &result_data[0], 1U);
	result_data[0] *= scale * 0.5f;

	if (unit)
	{
		// 幅值缩放与弧度->角度缩放都融入循环，省两次 arm_scale_f32 的 O(N) 遍历
		for (i = 1U; i < RFFT_RESULT_LENGTH; i++)
		{
			float32_t real = *p++;
			float32_t imag = *p++;

			arm_sqrt_f32((real * real) + (imag * imag), &mag);
			result_data[i] = mag * scale;

			arm_atan2_f32(imag, real, &phase);
			result_data[RFFT_RESULT_LENGTH + i] = phase * RFFT_RAD_TO_DEG;
		}
	}
	else
	{
		for (i = 1U; i < RFFT_RESULT_LENGTH; i++)
		{
			float32_t real = *p++;
			float32_t imag = *p++;

			arm_sqrt_f32((real * real) + (imag * imag), &mag);
			result_data[i] = mag * scale;

			arm_atan2_f32(imag, real, &result_data[RFFT_RESULT_LENGTH + i]);
		}
	}
}

/**
 * @brief 将输入采样数据复制到 RFFT 工作缓冲区。
 * @param[in,out] hrfft RFFT 句柄。
 * @param[in] adc_data_addr 输入采样缓冲区地址。
 * @return ARM_MATH_SUCCESS 表示复制成功，其他返回值表示 DMA 搬运失败。
 * @note 当前输入按 uint32_t 采样数据处理；若配置了 DMA 适配器，则优先使用 DMA 搬运。
 */
static arm_status rfft_copy_input(rfft_handle_t *hrfft, uint32_t adc_data_addr)
{
	uint32_t size = FFT_LENGTH * sizeof(uint32_t);

	if (hrfft->dma.transfer != NULL)
	{
		return hrfft->dma.transfer(hrfft->dma.handle,
								   adc_data_addr,
								   (uint32_t)hrfft->work_buffer,
								   size,
								   RFFT_DMA_TIMEOUT);
	}

	memcpy(hrfft->work_buffer, (const void *)(uintptr_t)adc_data_addr, size);

	return ARM_MATH_SUCCESS;
}

/**
 * @brief 准备输入数据并执行 RFFT 变换。
 * @param[in,out] hrfft RFFT 句柄。
 * @param[in] adc_data_addr 输入采样缓冲区地址。
 * @param[in] window 窗函数类型。
 * @return ARM_MATH_SUCCESS 表示 RFFT 前处理和计算成功，其他返回值来自输入搬运过程。
 * @note 处理流程为：复制输入、unsigned 转 float、生成并应用窗函数、执行 arm_rfft_fast_f32。
 */
static arm_status rfft_prepare_transform(rfft_handle_t *hrfft, uint32_t adc_data_addr, rfft_window_t window)
{
	arm_status status = rfft_copy_input(hrfft, adc_data_addr);

	if (status != ARM_MATH_SUCCESS)
	{
		return status;
	}

	convert_unsigned_to_float(hrfft->work_buffer, FFT_LENGTH);
	if (window != RFFT_WINDOW_RECTANGULAR)
	{
		rfft_generate_window(hrfft, window);
		arm_mult_f32(hrfft->work_buffer, hrfft->window_buffer, hrfft->work_buffer, FFT_LENGTH);
	}
	arm_rfft_fast_f32(&hrfft->fft_instance, hrfft->work_buffer, hrfft->work_buffer, 0);

	return ARM_MATH_SUCCESS;
}

/**
 * @brief 初始化 RFFT 句柄。
 * @param[out] hrfft RFFT 句柄。
 * @param[in] dma DMA 适配器指针；传入 NULL 时使用 memcpy 复制输入。
 * @return ARM_MATH_SUCCESS 表示初始化成功，ARM_MATH_ARGUMENT_ERROR 表示参数或 FFT_LENGTH 不支持。
 * @note 该函数会重置 DMA 配置和窗函数缓存，并根据 FFT_LENGTH 选择对应长度的 CMSIS-DSP RFFT 初始化函数。
 */
arm_status rfft_handle_init(rfft_handle_t *hrfft, const rfft_dma_t *dma)
{
#if USE_ARM_FFT
	arm_status status = ARM_MATH_SUCCESS;

	if (hrfft == NULL)
	{
		return ARM_MATH_ARGUMENT_ERROR;
	}

	hrfft->dma.handle = NULL;
	hrfft->dma.transfer = NULL;
	if (dma != NULL)
	{
		hrfft->dma = *dma;
	}

	hrfft->cached_window = RFFT_WINDOW_RECTANGULAR;
	hrfft->window_ready = 0U;
	hrfft->window_gain = 1.0f;

#if (FFT_LENGTH == 512)
	status = arm_rfft_fast_init_512_f32(&hrfft->fft_instance);
#elif (FFT_LENGTH == 1024)
	status = arm_rfft_fast_init_1024_f32(&hrfft->fft_instance);
#elif (FFT_LENGTH == 2048)
	status = arm_rfft_fast_init_2048_f32(&hrfft->fft_instance);
#elif (FFT_LENGTH == 4096)
	status = arm_rfft_fast_init_4096_f32(&hrfft->fft_instance);
#else
	status = ARM_MATH_ARGUMENT_ERROR;
#endif
	return status;
#else
	return ARM_MATH_ARGUMENT_ERROR;
#endif
}

void rfft_start(rfft_handle_t *hrfft,
				uint32_t adc_data_addr,
				float32_t *result_data,
				rfft_window_t window)
{
#if USE_ARM_FFT
	if ((hrfft == NULL) || (result_data == NULL))
	{
		RFFT_ERROR_HANDLER();
		return;
	}

	if (rfft_prepare_transform(hrfft, adc_data_addr, window) != ARM_MATH_SUCCESS)
	{
		RFFT_ERROR_HANDLER();
		return;
	}
#endif
}

/**
 * @brief 启动单边幅频计算。
 * @param[in,out] hrfft RFFT 句柄。
 * @param[in] adc_data_addr 输入采样缓冲区地址。
 * @param[out] result_data 输出幅频结果，长度至少为 RFFT_RESULT_LENGTH。
 * @param[in] window 窗函数类型。
 * @param[in] unit 幅值输出单位。
 * @note 参数非法或 RFFT 前处理失败时调用 RFFT_ERROR_HANDLER。
 */
void rfft_start_af(rfft_handle_t *hrfft,
				   uint32_t adc_data_addr,
				   float32_t *result_data,
				   rfft_window_t window,
				   rfft_unit_t unit)
{
#if USE_ARM_FFT
	if ((hrfft == NULL) || (result_data == NULL))
	{
		RFFT_ERROR_HANDLER();
		return;
	}

	if (rfft_prepare_transform(hrfft, adc_data_addr, window) != ARM_MATH_SUCCESS)
	{
		RFFT_ERROR_HANDLER();
		return;
	}

	rfft_make_single_sided_amplitude(hrfft, result_data);
	rfft_convert_unit(result_data, unit);
#endif
}

/**
 * @brief 启动单边相频计算。
 * @param[in,out] hrfft RFFT 句柄。
 * @param[in] adc_data_addr 输入采样缓冲区地址。
 * @param[out] result_data 输出相频结果，长度至少为 RFFT_RESULT_LENGTH。
 * @param[in] window 窗函数类型。
 * @param[in] p_unit 相位输出单位选择；0 表示弧度，非 0 表示角度。
 * @note 参数非法或 RFFT 前处理失败时调用 RFFT_ERROR_HANDLER。
 */
void rfft_start_pf(rfft_handle_t *hrfft,
				   uint32_t adc_data_addr,
				   float32_t *result_data,
				   rfft_window_t window,
				   uint8_t p_unit)
{
#if USE_ARM_FFT
	if ((hrfft == NULL) || (result_data == NULL))
	{
		RFFT_ERROR_HANDLER();
		return;
	}

	if (rfft_prepare_transform(hrfft, adc_data_addr, window) != ARM_MATH_SUCCESS)
	{
		RFFT_ERROR_HANDLER();
		return;
	}

	rfft_make_single_sided_phase(hrfft, result_data, p_unit);
#endif
}

/**
 * @brief 启动单边幅频和相频联合计算。
 * @param[in,out] hrfft RFFT 句柄。
 * @param[in] adc_data_addr 输入采样缓冲区地址。
 * @param[out] result_data 输出数组，长度至少为 2 * RFFT_RESULT_LENGTH。
 * @param[in] window 窗函数类型。
 * @param[in] unit 幅值输出单位。
 * @param[in] p_unit 相位输出单位选择；0 表示弧度，非 0 表示角度。
 * @note result_data 前半段保存幅值，后半段保存相位；参数非法或 RFFT 前处理失败时调用 RFFT_ERROR_HANDLER。
 */
void rfft_start_af_pf(rfft_handle_t *hrfft,
					  uint32_t adc_data_addr,
					  float32_t *result_data,
					  rfft_window_t window,
					  rfft_unit_t unit,
					  uint8_t p_unit)
{
#if USE_ARM_FFT
	if ((hrfft == NULL) || (result_data == NULL))
	{
		RFFT_ERROR_HANDLER();
		return;
	}

	if (rfft_prepare_transform(hrfft, adc_data_addr, window) != ARM_MATH_SUCCESS)
	{
		RFFT_ERROR_HANDLER();
		return;
	}

	rfft_make_single_sided_amplitude_phase(hrfft, result_data, p_unit);
	rfft_convert_unit(result_data, unit);
#endif
}

void irfft_start(rfft_handle_t *hrfft,
				 const float32_t *rfft_af,
				 const float32_t *rfft_pf,
				 float32_t *irfft,
				 const float32_t *bode,
				 const float32_t *PF,
				 uint8_t p_unit)
{
#if USE_ARM_FFT
	if ((hrfft == NULL) || (rfft_af == NULL) || (rfft_pf == NULL) || (irfft == NULL) || (bode == NULL) || (PF == NULL))
	{
		RFFT_ERROR_HANDLER();
		return;
	}

	const float32_t dc_scale = (float32_t)FFT_LENGTH;
	const float32_t ac_scale = 0.5f * (float32_t)FFT_LENGTH;

	/* CMSIS RIFFT expects unnormalized packed DFT bins, not single-sided amplitude. */
	irfft[0] = rfft_af[0] * bode[0] * dc_scale;
	irfft[1] = 0.0f;

	if (p_unit)
	{
		for (uint16_t i = 1; i < RFFT_RESULT_LENGTH; i++)
		{
			float32_t mag = rfft_af[i] * bode[i] * ac_scale;
			float32_t phase = rfft_pf[i];

			/* rfft_pf is radians; p_unit describes only the PF array. */
			phase += PF[i] * RFFT_DEG_TO_RAD;
			phase = rfft_wrap_phase_rad(phase);

			irfft[i * 2] = mag * cosf(phase);
			irfft[i * 2 + 1] = mag * sinf(phase);
		}
	}
	else
	{
		for (uint16_t i = 1; i < RFFT_RESULT_LENGTH; i++)
		{
			float32_t mag = rfft_af[i] * bode[i] * ac_scale;
			float32_t phase = rfft_pf[i];

			/* rfft_pf is radians; p_unit describes only the PF array. */
			phase += PF[i];
			phase = rfft_wrap_phase_rad(phase);

			irfft[i * 2] = mag * cosf(phase);
			irfft[i * 2 + 1] = mag * sinf(phase);
		}
	}

	arm_rfft_fast_f32(&hrfft->fft_instance, irfft, irfft, 1);

#endif
}

/**
 * @brief RFFT 默认错误处理函数。
 * @note 该函数为弱定义；用户层可重新实现同名函数，以覆盖默认死循环行为。
 */
__WEAK void RFFT_ERROR_HANDLER(void)
{
	while (1)
	{
	}
}
