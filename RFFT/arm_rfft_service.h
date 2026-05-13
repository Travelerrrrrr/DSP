#ifndef __ARM_RFFT_SERVICE_H
#define __ARM_RFFT_SERVICE_H

#include <stdint.h>
#include "arm_math.h"

#if USE_ARM_FFT

#ifndef FFT_LENGTH
#define FFT_LENGTH 1024U
#endif

#define RFFT_HALF_LENGTH (FFT_LENGTH / 2U)
#define FFT_RESULT_LENGTH (FFT_LENGTH + 2U)
#define ADC_RESOLUTION_BITS 12U
#define ADC_FULL_SCALE_VOLTAGE 3.3f
#define ADC_FULL_SCALE_CODE ((1UL << ADC_RESOLUTION_BITS) - 1UL)
#define ADC_LSB_VOLTAGE (ADC_FULL_SCALE_VOLTAGE / (float32_t)ADC_FULL_SCALE_CODE)

/* 1 Vrms 对应的 ADC 码值，用作 dBV 的 0 dB 参考。 */
#define RFFT_DBV_REFERENCE_ADC_CODE (1.41421356237309f / ADC_LSB_VOLTAGE)
#define RFFT_DBFS_REFERENCE_CODE ((ADC_FULL_SCALE_VOLTAGE / 2) / ADC_LSB_VOLTAGE)

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
	float32_t scratch_buffer[FFT_LENGTH]; // 时域输入暂存，避开 arm_rfft_fast_f32 不支持 in-place
	float32_t window_buffer[FFT_LENGTH];
	rfft_window_t cached_window;
	uint8_t window_ready;
	float32_t window_gain;
} rfft_handle_t;

/**
 * @brief 初始化 RFFT 服务句柄。
 * @param hrfft RFFT 服务句柄指针。
 * @param dma DMA 搬运接口配置；传入 NULL 时使用 memcpy 拷贝输入数据。
 * @return ARM_MATH_SUCCESS 表示初始化成功，ARM_MATH_ARGUMENT_ERROR 表示参数或 FFT_LENGTH 不支持。
 * @note 该函数会初始化 CMSIS-DSP RFFT 实例，并重置窗口缓存和 DMA 配置。
 */
arm_status rfft_handle_init(rfft_handle_t *hrfft, const rfft_dma_t *dma);

/**
 * @brief 执行 RFFT 前处理和变换，输出 packed 复数频域。
 * @param hrfft RFFT 服务句柄指针。
 * @param adc_data_addr 输入采样缓冲区地址。
 * @param result_data packed RFFT 复数输出，长度至少为 FFT_LENGTH（CMSIS packed 格式：[DC, Nyquist, X1.re, X1.im, ...]）。
 * @param window 窗函数类型。
 * @param offset 转换为 float32 后叠加到每个采样点的偏置值；0.0f 表示不调整。
 * @note 该 packed 频域结果可直接传给 irfft_start_compensation 作为 rfft_complex 输入。参数非法或 RFFT 前处理失败时调用 RFFT_ERROR_HANDLER。
 */
void rfft_start(rfft_handle_t *hrfft,
				uint32_t adc_data_addr,
				float32_t *result_data,
				rfft_window_t window,
				float32_t offset);

/**
 * @brief 执行 RFFT 并输出单边幅频结果。
 * @param hrfft RFFT 服务句柄指针。
 * @param result_data 工作/输出缓冲区，长度至少为 FFT_LENGTH；最终输出 result_data[0..RFFT_HALF_LENGTH]。
 * @param window 窗函数类型。
 * @param unit 幅值输出单位。
 * @param offset 转换为 float32 后叠加到每个采样点的偏置值；0.0f 表示不调整。
 * @note 输出包含 DC、普通正频率点和 Nyquist 点，共 RFFT_HALF_LENGTH + 1 个单边幅值；参数非法或 RFFT 前处理失败时调用 RFFT_ERROR_HANDLER。
 */
void rfft_start_af(rfft_handle_t *hrfft,
				   uint32_t adc_data_addr,
				   float32_t *result_data,
				   rfft_window_t window,
				   rfft_unit_t unit,
				   float32_t offset);

/**
 * @brief 执行 RFFT 并输出单边相频结果。
 * @param hrfft RFFT 服务句柄指针。
 * @param result_data 工作/输出缓冲区，长度至少为 FFT_LENGTH；最终输出 result_data[0..RFFT_HALF_LENGTH]。
 * @param window 窗函数类型。
 * @param p_unit 相位输出单位；0 表示弧度，非 0 表示角度。
 * @param offset 转换为 float32 后叠加到每个采样点的偏置值；0.0f 表示不调整。
 * @note 输出包含 DC、普通正频率点和 Nyquist 点，共 RFFT_HALF_LENGTH + 1 个单边相位；参数非法或 RFFT 前处理失败时调用 RFFT_ERROR_HANDLER。
 */
void rfft_start_pf(rfft_handle_t *hrfft,
				   uint32_t adc_data_addr,
				   float32_t *result_data,
				   rfft_window_t window,
				   uint8_t p_unit,
				   float32_t offset);

/**
 * @brief 执行 RFFT 并输出单边幅频和相频结果。
 * @param hrfft RFFT 服务句柄指针。
 * @param result_data 工作/输出缓冲区，长度至少为 FFT_LENGTH + 2；最终按 [幅值0, 相位0, 幅值1, 相位1, ...] 交错输出。
 * @param window 窗函数类型。
 * @param unit 幅值输出单位。
 * @param p_unit 相位输出单位；0 表示弧度，非 0 表示角度。
 * @param offset 转换为 float32 后叠加到每个采样点的偏置值；0.0f 表示不调整。
 * @note 输出覆盖 DC、普通正频率点和 Nyquist 点，共 RFFT_HALF_LENGTH + 1 组幅相结果；参数非法或 RFFT 前处理失败时调用 RFFT_ERROR_HANDLER。
 */
void rfft_start_af_pf(rfft_handle_t *hrfft,
					  uint32_t adc_data_addr,
					  float32_t *result_data,
					  rfft_window_t window,
					  rfft_unit_t unit,
					  uint8_t p_unit,
					  float32_t offset);

/**
 * @brief 根据幅频、相频和补偿曲线执行逆 RFFT。
 * @param hrfft RFFT 服务句柄指针。
 * @param rfft_complex CMSIS packed RFFT 复数域数组，长度至少为 FFT_LENGTH，函数会原地写入补偿后的频域数据。
 * @param irfft 逆变换输出缓冲区，长度至少为 FFT_LENGTH。
 * @param AF 幅值补偿数组，长度至少为 RFFT_HALF_LENGTH + 1。
 * @param PF 相位补偿数组，长度至少为 RFFT_HALF_LENGTH + 1。
 * @param p_unit PF 的相位单位；0 表示弧度，非 0 表示角度。
 * @note AF/PF 的下标 0 表示 DC，1..RFFT_HALF_LENGTH-1 表示普通正频率点，RFFT_HALF_LENGTH 表示 Nyquist 点。
 *       DC 和 Nyquist 在 packed RFFT 中必须保持纯实数，因此仅使用补偿相位的 cos 分量。
 *       参数非法时调用 RFFT_ERROR_HANDLER。
 */
void irfft_start_compensation(rfft_handle_t *hrfft,
							  float32_t *rfft_complex,
							  float32_t *irfft,
							  const float32_t *AF,
							  const float32_t *PF,
							  uint8_t p_unit);

/**
 * @brief 使用幅度和相位信息构建复数频域数据，并执行反向RFFT变换
 * @param hrfft RFFT句柄，包含实例配置和临时缓冲区
 * @param irfft 输出缓冲区，存放反向变换结果（时域数据）
 * @param AF 幅度频谱数组，长度为 RFFT_HALF_LENGTH + 1
 * @param PF 相位频谱数组，长度为 RFFT_HALF_LENGTH + 1
 * @param p_unit 相位单位标志：1 表示度数（转换为弧度），0 表示弧度
 * @note 输入/输出缓冲区必须不同（不支持原地变换）；内部使用scratch_buffer进行数据重组
 */
void irfft_start_construction(rfft_handle_t *hrfft,
							  float32_t *irfft,
							  const float32_t *AF,
							  const float32_t *PF,
							  uint8_t p_unit);

/**
 * @brief RFFT 服务错误处理回调。
 * @note 默认实现为弱定义，用户可重新实现同名函数覆盖默认死循环行为。
 */
void RFFT_ERROR_HANDLER(void);

#endif // USE_ARM_FFT
#endif // __ARM_RFFT_SERVICE_H
