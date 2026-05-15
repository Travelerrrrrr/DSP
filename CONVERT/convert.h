#ifndef __CONVERT_H
#define __CONVERT_H

#include "../User/DSP/dsp_config.h"
#include <stdint.h>

#ifndef USE_CONVERT
#define USE_CONVERT 0
#endif

#define CONVERT_SWITCH (USE_CONVERT || USE_ARM_FFT)

#if CONVERT_SWITCH

/**
 * @brief 将 unsigned 32-bit 原始采样转换为 float32 数据，并可选叠加偏置。
 * @param Dst 目标 float32_t 缓冲区。
 * @param Src 源 uint32_t 采样缓冲区。
 * @param length 需要转换的数据点数。
 * @param offset 转换为 float32 后叠加到每个采样点的偏置值；0.0f 表示不调整。
 * @note 源数据和目标数据均为 32 bit 宽度，Dst 与 Src 指向同一缓冲区时可安全地正序原地转换。
 */
void convert_uint32_to_float(float32_t *Dst, const uint32_t *Src, uint32_t length, float32_t offset);

/**
 * @brief 将 float32 数据转换为 unsigned 32-bit 整数，可选在转换前叠加偏置。
 * @param Dst 目标 uint32_t 缓冲区。
 * @param Src 源 float32_t 数据缓冲区。
 * @param length 需要转换的数据点数。
 * @param offset 转换前叠加到每个 float 采样点的偏置值；0.0f 表示不调整。
 * @note 源数据和目标数据均为 32 bit 宽度，Dst 与 Src 指向同一缓冲区时可安全地正序原地转换。
 */
void convert_float_to_uint32(uint32_t *Dst, const float32_t *Src, uint32_t length, float32_t offset);

/**
 * @brief 将 float32 数据转换为 Q1.31 定点数，可选在转换前叠加偏置。
 * @param Dst 目标 q31_t 缓冲区。
 * @param Src 源 float32_t 数据缓冲区。
 * @param length 需要转换的数据点数。
 * @param offset 转换前叠加到每个 float 采样点的偏置值；0.0f 表示不调整。
 * @note 转换公式 q31 = (float * 2^31)，超出 [-1.0, 1.0) 范围的输入会饱和到 Q31 边界。
 *       源数据和目标数据均为 32 bit 宽度，Dst 与 Src 指向同一缓冲区时可安全地正序原地转换。
 *       LSB = 2^-31 ≈ 4.6566129e-10
 */
void convert_float_to_q31(q31_t *Dst, const float32_t *Src, uint32_t length, float32_t offset);

/**
 * @brief 将 float32 数据转换为 Q1.15 定点数，可选在转换前叠加偏置。
 * @param Dst 目标 q15_t 缓冲区。
 * @param Src 源 float32_t 数据缓冲区。
 * @param length 需要转换的数据点数。
 * @param offset 转换前叠加到每个 float 采样点的偏置值；0.0f 表示不调整。
 * @note 转换公式 q15 = (float * 2^15)，超出 [-1.0, 1.0) 范围的输入会饱和到 Q15 边界。
 *       Dst 与 Src 指向同一缓冲区时可安全地正序原地转换。
 *       LSB = 2^-15 ≈ 3.05e-5
 */
void convert_float_to_q15(q15_t *Dst, const float32_t *Src, uint32_t length, float32_t offset);

/**
 * @brief 将 float32 数据转换为 Q1.7 定点数，可选在转换前叠加偏置。
 * @param Dst 目标 q7_t 缓冲区。
 * @param Src 源 float32_t 数据缓冲区。
 * @param length 需要转换的数据点数。
 * @param offset 转换前叠加到每个 float 采样点的偏置值；0.0f 表示不调整。
 * @note 转换公式 q7 = (float * 2^7)，超出 [-1.0, 1.0) 范围的输入会饱和到 Q7 边界。
 *       Dst 与 Src 指向同一缓冲区时可安全地正序原地转换。
 *       LSB = 2^-7 ≈ 0.0078125，输入数据的绝对值小于该值时会被量化为 0。
 */
void convert_float_to_q7(q7_t *Dst, const float32_t *Src, uint32_t length, float32_t offset);

#endif // USE_CONVERT

#endif // __CONVERT_H
