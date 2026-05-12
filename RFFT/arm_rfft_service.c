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
 * @brief 将工作缓冲区中的 unsigned 32-bit 原始采样原地转换为 float32 数据，并可选叠加偏置。
 * @param[in,out] Src 工作缓冲区指针；转换前按 uint32_t 解释，转换后按 float32_t 解释。
 * @param[in] length 需要转换的数据点数。
 * @param[in] offset 转换为 float32 后叠加到每个采样点的偏置值；0.0f 表示不调整。
 * @note 源数据和目标数据均为 32 bit 宽度，可安全地正序原地转换。
 *       offset 非 0.0f 时借助 CMSIS-DSP 的 arm_offset_f32（SIMD/VFP 加速）完成逐点偏置叠加。
 */
static void convert_unsigned_to_float(float32_t *Src, uint32_t length, float32_t offset)
{
	uint32_t i;
	uint32_t *buf = (uint32_t *)Src;

	if (offset != 0.0f)
	{
		for (i = 0; i < length; i++)
		{
			Src[i] = (float32_t)buf[i];
		}

		arm_offset_f32(Src, offset, Src, length);
	}
	else
	{
		for (i = 0; i < length; i++)
		{
			Src[i] = (float32_t)buf[i];
		}
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
 * @param[in,out] result_data 幅频结果数组，长度至少为 RFFT_HALF_LENGTH。
 * @param[in] unit 目标单位，可选择 RAW、dBV 或 dBFS。
 * @note RAW 模式不做转换；dB 模式下执行 20log10(x/reference)，并对输入下限做裁剪以避免 log(0)。
 */
static void rfft_convert_unit(float32_t *result_data, rfft_unit_t unit)
{
	float32_t reference = 1.0f;

	switch (unit)
	{
	case RFFT_UNIT_DBV:
		reference = RFFT_DBV_REFERENCE_ADC_CODE;
		break;
	case RFFT_UNIT_DBFS:
		reference = RFFT_DBFS_REFERENCE_CODE;
		break;
	case RFFT_UNIT_RAW:
	default:
		// 线性电压单位不需要做 dB 转换
		return;
	}

	arm_scale_f32(result_data, 1.0f / reference, result_data, RFFT_HALF_LENGTH);				// 20log10(x/reference)
	arm_clip_f32(result_data, result_data, RFFT_LOG_FLOOR, RFFT_LOG_CEILING, RFFT_HALF_LENGTH); // 仅为O(N)，为保证系统稳定，必须保留
	arm_vlog_f32(result_data, result_data, RFFT_HALF_LENGTH);
	arm_scale_f32(result_data, RFFT_LN_TO_20LOG10, result_data, RFFT_HALF_LENGTH); // 20log10(x) = ln(x) * 20/ln(10)
}

/**
 * @brief 从 RFFT 复数结果中提取并归一化单边幅频结果。
 * @param[in] hrfft RFFT 句柄。
 * @param[in,out] result_data 输入为 packed RFFT 复数（长度 FFT_LENGTH），原地压缩为单边幅频结果（长度 RFFT_HALF_LENGTH+1）。
 * @note 非直流频点按单边谱幅值缩放，直流分量额外乘以 0.5 以避免被双边合并系数放大。
 *       原地压缩安全性：arm_cmplx_mag_f32 写 result_data[k] 读 result_data[2k..2k+1]，k 顺序递增时写下标永远小于读下标，无冲突。
 *       Nyquist 必须先读后写，否则 result_data[1] 会被 DC 的 abs 覆盖。
 */
static void rfft_make_single_sided_amplitude(rfft_handle_t *hrfft, float32_t *result_data)
{
	float32_t scale = 2.0f / ((float32_t)FFT_LENGTH * hrfft->window_gain);
	float32_t nyquist = result_data[1];

	arm_cmplx_mag_f32(&result_data[2], &result_data[1], RFFT_HALF_LENGTH - 1U);
	result_data[0] = fabsf(result_data[0]);
	result_data[RFFT_HALF_LENGTH] = fabsf(nyquist);
	arm_scale_f32(result_data, scale, result_data, RFFT_HALF_LENGTH + 1U);
	result_data[0] *= 0.5f;
	result_data[RFFT_HALF_LENGTH] *= 0.5f;
}

/**
 * @brief 从 RFFT 复数结果中提取单边相频结果。
 * @param[in] hrfft RFFT 句柄（未使用）。
 * @param[in,out] result_data 输入为 packed RFFT 复数（长度 FFT_LENGTH），原地压缩为单边相频结果（长度 RFFT_HALF_LENGTH+1）。
 * @param[in] unit 相位输出单位选择；0 表示弧度，非 0 表示角度。
 * @note 原地压缩：循环顺序 k=1..N/2-1 写 result_data[k] 读 result_data[2k..2k+1]，写下标永远小于读下标，无冲突。
 *       DC/Nyquist 实部先暂存到局部变量，Nyquist 相位必须循环结束后再写入，避免在 k=N/4 时覆盖 X[N/4].real。
 */
static void rfft_make_single_sided_phase(rfft_handle_t *hrfft, float32_t *result_data, uint8_t unit)
{
	(void)hrfft;
	uint32_t i;
	float32_t dc_real = result_data[0];
	float32_t nyquist_real = result_data[1];
	float32_t dc_phase, nyquist_phase;

	arm_atan2_f32(0.0f, dc_real, &dc_phase);
	arm_atan2_f32(0.0f, nyquist_real, &nyquist_phase);

	if (unit)
	{
		// 弧度->角度缩放融入循环，省一次 arm_scale_f32 的 O(N) 遍历
		float32_t phase;
		for (i = 1U; i < RFFT_HALF_LENGTH; i++)
		{
			arm_atan2_f32(result_data[2 * i + 1], result_data[2 * i], &phase);
			result_data[i] = phase * RFFT_RAD_TO_DEG;
		}
		result_data[0] = dc_phase * RFFT_RAD_TO_DEG;
		result_data[RFFT_HALF_LENGTH] = nyquist_phase * RFFT_RAD_TO_DEG;
	}
	else
	{
		for (i = 1U; i < RFFT_HALF_LENGTH; i++)
		{
			arm_atan2_f32(result_data[2 * i + 1], result_data[2 * i], &result_data[i]);
		}
		result_data[0] = dc_phase;
		result_data[RFFT_HALF_LENGTH] = nyquist_phase;
	}
}

/**
 * @brief 同时从 RFFT 复数结果中提取单边幅频和相频结果。
 * @param[in,out] hrfft RFFT 句柄；scratch_buffer 在本函数内被借用为相位暂存。
 * @param[in,out] result_data 输入为 packed RFFT 复数（长度 FFT_LENGTH），原地写出 [幅值 N/2+1 | 相位 N/2+1]，总长度 FFT_LENGTH+2。
 * @param[in] unit 相位输出单位选择；0 表示弧度，非 0 表示角度。
 * @note 幅值原地压缩到 result_data[0..N/2] 是安全的（写下标 k 总小于读下标 2k）。但若把相位直接写入 result_data[N/2+1+k]，
 *       当 k > N/4 时读取的 result_data[2k] 已被先前迭代的相位覆盖。所以相位先暂存到 scratch_buffer，循环结束后再一次性 memcpy 到相位区。
 */
static void rfft_make_single_sided_amplitude_phase(rfft_handle_t *hrfft, float32_t *result_data, uint8_t unit)
{
	uint32_t i;
	const float32_t scale = 2.0f / ((float32_t)FFT_LENGTH * hrfft->window_gain);
	const float32_t phase_unit_scale = unit ? RFFT_RAD_TO_DEG : 1.0f;
	float32_t mag, phase;
	float32_t dc_phase, nyquist_phase;
	float32_t dc_real = result_data[0];
	float32_t nyquist_real = result_data[1];

	arm_atan2_f32(0.0f, dc_real, &dc_phase);
	arm_atan2_f32(0.0f, nyquist_real, &nyquist_phase);

	// 幅值原地、相位暂存到 scratch_buffer
	for (i = 1U; i < RFFT_HALF_LENGTH; i++)
	{
		float32_t real = result_data[2 * i];
		float32_t imag = result_data[2 * i + 1];

		arm_sqrt_f32((real * real) + (imag * imag), &mag);
		result_data[i] = mag * scale;

		arm_atan2_f32(imag, real, &phase);
		hrfft->scratch_buffer[i] = phase * phase_unit_scale;
	}

	// DC / Nyquist 幅值（单边谱 DC 不翻倍，scale*0.5 一次性应用）
	result_data[0] = fabsf(dc_real) * scale * 0.5f;
	result_data[RFFT_HALF_LENGTH] = fabsf(nyquist_real) * scale * 0.5f;

	// 写相位区：DC、Nyquist 单独写，其余从 scratch_buffer 整段搬运
	result_data[RFFT_HALF_LENGTH + 1] = dc_phase * phase_unit_scale;
	memcpy(&result_data[RFFT_HALF_LENGTH + 2],
		   &hrfft->scratch_buffer[1],
		   (RFFT_HALF_LENGTH - 1U) * sizeof(float32_t));
	result_data[RFFT_HALF_LENGTH * 2 + 1] = nyquist_phase * phase_unit_scale;
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
								   (uint32_t)hrfft->scratch_buffer,
								   size,
								   RFFT_DMA_TIMEOUT);
	}

	memcpy(hrfft->scratch_buffer, (const void *)(uintptr_t)adc_data_addr, size);

	return ARM_MATH_SUCCESS;
}

/**
 * @brief 准备输入数据并执行 RFFT 变换。
 * @param[in,out] hrfft RFFT 句柄。
 * @param[in] adc_data_addr 输入采样缓冲区地址。
 * @param[out] result_data packed RFFT 复数输出缓冲区，长度至少为 FFT_LENGTH。
 * @param[in] window 窗函数类型。
 * @param[in] offset 转换为 float32 后叠加到每个采样点的偏置值；0.0f 表示不调整。
 * @return ARM_MATH_SUCCESS 表示 RFFT 前处理和计算成功，其他返回值来自输入搬运过程。
 * @note 处理流程为：复制输入、unsigned 转 float（可选叠加 offset）、生成并应用窗函数、执行 arm_rfft_fast_f32。
 *       scratch_buffer 作为时域输入，result_data 作为频域 packed 输出，两者必须为不同缓冲区（arm_rfft_fast_f32 不支持 in-place）。
 */
static arm_status rfft_prepare_transform(rfft_handle_t *hrfft, uint32_t adc_data_addr, float32_t *result_data, rfft_window_t window, float32_t offset)
{
	arm_status status = rfft_copy_input(hrfft, adc_data_addr);

	if (status != ARM_MATH_SUCCESS)
	{
		return status;
	}

	convert_unsigned_to_float(hrfft->scratch_buffer, FFT_LENGTH, offset);

	if (window != RFFT_WINDOW_RECTANGULAR)
	{
		rfft_generate_window(hrfft, window);
		arm_mult_f32(hrfft->scratch_buffer, hrfft->window_buffer, hrfft->scratch_buffer, FFT_LENGTH);
	}

	arm_rfft_fast_f32(&hrfft->fft_instance, hrfft->scratch_buffer, result_data, 0);

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
				rfft_window_t window,
				float32_t offset)
{
#if USE_ARM_FFT
	if ((hrfft == NULL) || (result_data == NULL))
	{
		RFFT_ERROR_HANDLER();
		return;
	}

	if (rfft_prepare_transform(hrfft, adc_data_addr, result_data, window, offset) != ARM_MATH_SUCCESS)
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
 * @param[out] result_data 输出幅频结果，长度至少为 RFFT_HALF_LENGTH。
 * @param[in] window 窗函数类型。
 * @param[in] unit 幅值输出单位。
 * @param[in] offset 转换为 float32 后叠加到每个采样点的偏置值；0.0f 表示不调整。
 * @note 参数非法或 RFFT 前处理失败时调用 RFFT_ERROR_HANDLER。
 */
void rfft_start_af(rfft_handle_t *hrfft,
				   uint32_t adc_data_addr,
				   float32_t *result_data,
				   rfft_window_t window,
				   rfft_unit_t unit,
				   float32_t offset)
{
#if USE_ARM_FFT
	if ((hrfft == NULL) || (result_data == NULL))
	{
		RFFT_ERROR_HANDLER();
		return;
	}

	if (rfft_prepare_transform(hrfft, adc_data_addr, result_data, window, offset) != ARM_MATH_SUCCESS)
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
 * @param[out] result_data 输出相频结果，长度至少为 RFFT_HALF_LENGTH。
 * @param[in] window 窗函数类型。
 * @param[in] p_unit 相位输出单位选择；0 表示弧度，非 0 表示角度。
 * @param[in] offset 转换为 float32 后叠加到每个采样点的偏置值；0.0f 表示不调整。
 * @note 参数非法或 RFFT 前处理失败时调用 RFFT_ERROR_HANDLER。
 */
void rfft_start_pf(rfft_handle_t *hrfft,
				   uint32_t adc_data_addr,
				   float32_t *result_data,
				   rfft_window_t window,
				   uint8_t p_unit,
				   float32_t offset)
{
#if USE_ARM_FFT
	if ((hrfft == NULL) || (result_data == NULL))
	{
		RFFT_ERROR_HANDLER();
		return;
	}

	if (rfft_prepare_transform(hrfft, adc_data_addr, result_data, window, offset) != ARM_MATH_SUCCESS)
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
 * @param[out] result_data 输出数组，长度至少为 2 * RFFT_HALF_LENGTH。
 * @param[in] window 窗函数类型。
 * @param[in] unit 幅值输出单位。
 * @param[in] p_unit 相位输出单位选择；0 表示弧度，非 0 表示角度。
 * @param[in] offset 转换为 float32 后叠加到每个采样点的偏置值；0.0f 表示不调整。
 * @note result_data 前半段保存幅值，后半段保存相位；参数非法或 RFFT 前处理失败时调用 RFFT_ERROR_HANDLER。
 */
void rfft_start_af_pf(rfft_handle_t *hrfft,
					  uint32_t adc_data_addr,
					  float32_t *result_data,
					  rfft_window_t window,
					  rfft_unit_t unit,
					  uint8_t p_unit,
					  float32_t offset)
{
#if USE_ARM_FFT
	if ((hrfft == NULL) || (result_data == NULL))
	{
		RFFT_ERROR_HANDLER();
		return;
	}

	if (rfft_prepare_transform(hrfft, adc_data_addr, result_data, window, offset) != ARM_MATH_SUCCESS)
	{
		RFFT_ERROR_HANDLER();
		return;
	}

	rfft_make_single_sided_amplitude_phase(hrfft, result_data, p_unit);
	rfft_convert_unit(result_data, unit);
#endif
}

/**
 * @brief 根据幅频、相频和补偿曲线执行逆 RFFT。
 * @param[in,out] hrfft RFFT 服务句柄指针。
 * @param[in,out] rfft_complex CMSIS packed RFFT 复数域数组，长度至少为 FFT_LENGTH，函数会原地写入补偿后的频域数据。
 * @param[out] irfft 逆变换输出缓冲区，长度至少为 FFT_LENGTH。
 * @param[in] bode 幅值补偿数组，长度至少为 RFFT_UNIQUE_RESULT_LENGTH。
 * @param[in] PF 相位补偿数组，长度至少为 RFFT_UNIQUE_RESULT_LENGTH。
 * @param[in] p_unit PF 的相位单位；0 表示弧度，非 0 表示角度。
 * @note bode/PF 的下标 0 表示 DC，1..RFFT_HALF_LENGTH-1 表示普通正频率点，RFFT_HALF_LENGTH 表示 Nyquist 点。
 *       DC 和 Nyquist 在 packed RFFT 中必须保持纯实数，因此仅使用补偿相位的 cos 分量。
 *       参数非法时调用 RFFT_ERROR_HANDLER。
 */
void irfft_start(rfft_handle_t *hrfft,
				 float32_t *rfft_complex,
				 float32_t *irfft,
				 const float32_t *bode,
				 const float32_t *PF,
				 uint8_t p_unit)
{
#if USE_ARM_FFT
	if ((hrfft == NULL) || (rfft_complex == NULL) || (irfft == NULL) || (bode == NULL) || (PF == NULL))
	{
		RFFT_ERROR_HANDLER();
		return;
	}

	const float32_t phase_scale = p_unit ? RFFT_DEG_TO_RAD : 1.0f;

	// DC/Nyquist 在 packed 格式下是纯实数，只取补偿相位的 cos 分量
	rfft_complex[0] *= bode[0] * cosf(PF[0] * phase_scale);
	rfft_complex[1] *= bode[RFFT_HALF_LENGTH] * cosf(PF[RFFT_HALF_LENGTH] * phase_scale);

	for (uint16_t i = 1; i < RFFT_HALF_LENGTH; i++)
	{
		float32_t phase = PF[i] * phase_scale;
		float32_t Hr = bode[i] * cosf(phase);
		float32_t Hi = bode[i] * sinf(phase);
		float32_t Xr = rfft_complex[i * 2];
		float32_t Xi = rfft_complex[i * 2 + 1];

		rfft_complex[i * 2] = Xr * Hr - Xi * Hi;
		rfft_complex[i * 2 + 1] = Xr * Hi + Xi * Hr;
	}

	// arm_rfft_fast_f32 inverse 模式内部 merge_rfft_f32 的访问模式不支持 in-place，
	// 输入/输出必须是不同 buffer
	arm_rfft_fast_f32(&hrfft->fft_instance, rfft_complex, irfft, 1);

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
