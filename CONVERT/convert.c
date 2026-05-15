/**
 * @file convert.c
 * @brief 数值格式转换实现。
 * @author Analog
 * @version 1.2
 * @date 2026-05-13
 * @note 本文件实现了 unsigned 32-bit 原始采样与 float32 数据之间的转换函数，支持可选的偏置叠加。
 */
#include "../User/DSP/convert.h"

/**
 * @brief 将 unsigned 32-bit 原始采样转换为 float32 数据，并可选叠加偏置。
 * @param Dst 目标 float32_t 缓冲区。
 * @param Src 源 uint32_t 采样缓冲区。
 * @param length 需要转换的数据点数。
 * @param offset 转换为 float32 后叠加到每个采样点的偏置值；0.0f 表示不调整。
 * @note 源数据和目标数据均为 32 bit 宽度，Dst 与 Src 指向同一缓冲区时可安全地正序原地转换。
 */
void convert_uint32_to_float(float32_t *Dst, const uint32_t *Src, uint32_t length, float32_t offset)
{
#if CONVERT_SWITCH
    uint32_t blkCnt = length >> 2U; // 块计数 4x 展开主循环 后续要做剩余元素的处理

    if (offset != 0.0f)
    {
        while (blkCnt > 0U)
        {
            Dst[0] = (float32_t)Src[0] + offset;
            Dst[1] = (float32_t)Src[1] + offset;
            Dst[2] = (float32_t)Src[2] + offset;
            Dst[3] = (float32_t)Src[3] + offset;
            Dst += 4U;
            Src += 4U;
            blkCnt--;
        }

        blkCnt = length & 0x3U;
        while (blkCnt > 0U)
        {
            *Dst++ = (float32_t)(*Src++) + offset;
            blkCnt--;
        }
    }
    else
    {
        while (blkCnt > 0U)
        {
            Dst[0] = (float32_t)Src[0];
            Dst[1] = (float32_t)Src[1];
            Dst[2] = (float32_t)Src[2];
            Dst[3] = (float32_t)Src[3];
            Dst += 4U;
            Src += 4U;
            blkCnt--;
        }

        blkCnt = length & 0x3U;
        while (blkCnt > 0U)
        {
            *Dst++ = (float32_t)(*Src++);
            blkCnt--;
        }
    }
#endif
}

/**
 * @brief 将 float32 数据转换为 unsigned 32-bit 整数，可选在转换前叠加偏置。
 * @param Dst 目标 uint32_t 缓冲区。
 * @param Src 源 float32_t 数据缓冲区。
 * @param length 需要转换的数据点数。
 * @param offset 转换前叠加到每个 float 采样点的偏置值；0.0f 表示不调整。
 * @note 源数据和目标数据均为 32 bit 宽度，Dst 与 Src 指向同一缓冲区时可安全地正序原地转换。
 */
void convert_float_to_uint32(uint32_t *Dst, const float32_t *Src, uint32_t length, float32_t offset)
{
#if CONVERT_SWITCH
    uint32_t blkCnt = length >> 2U; // 块计数 4x 展开主循环 后续要做剩余元素的处理

    if (offset != 0.0f)
    {
        while (blkCnt > 0U)
        {
            Dst[0] = (uint32_t)(Src[0] + offset);
            Dst[1] = (uint32_t)(Src[1] + offset);
            Dst[2] = (uint32_t)(Src[2] + offset);
            Dst[3] = (uint32_t)(Src[3] + offset);
            Dst += 4U;
            Src += 4U;
            blkCnt--;
        }

        blkCnt = length & 0x3U;
        while (blkCnt > 0U)
        {
            *Dst++ = (uint32_t)(*Src++ + offset);
            blkCnt--;
        }
    }
    else
    {
        while (blkCnt > 0U)
        {
            Dst[0] = (uint32_t)Src[0];
            Dst[1] = (uint32_t)Src[1];
            Dst[2] = (uint32_t)Src[2];
            Dst[3] = (uint32_t)Src[3];
            Dst += 4U;
            Src += 4U;
            blkCnt--;
        }

        blkCnt = length & 0x3U;
        while (blkCnt > 0U)
        {
            *Dst++ = (uint32_t)(*Src++);
            blkCnt--;
        }
    }
#endif
}

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
void convert_float_to_q31(q31_t *Dst, const float32_t *Src, uint32_t length, float32_t offset)
{
#if CONVERT_SWITCH
    if (offset == 0.0f)
    {
        // 无偏置时直接调用 CMSIS-DSP 内部已 SIMD 优化的实现
        arm_float_to_q31(Src, Dst, length);
        return;
    }

    uint32_t blkCnt = length >> 2U; // 块计数 4x 展开主循环 后续要做剩余元素的处理
    float32_t in;

    while (blkCnt > 0U)
    {
        in = (Src[0] + offset) * 2147483648.0f;
        Dst[0] = clip_q63_to_q31((q63_t)in);
        in = (Src[1] + offset) * 2147483648.0f;
        Dst[1] = clip_q63_to_q31((q63_t)in);
        in = (Src[2] + offset) * 2147483648.0f;
        Dst[2] = clip_q63_to_q31((q63_t)in);
        in = (Src[3] + offset) * 2147483648.0f;
        Dst[3] = clip_q63_to_q31((q63_t)in);
        Dst += 4U;
        Src += 4U;
        blkCnt--;
    }

    blkCnt = length & 0x3U;
    while (blkCnt > 0U)
    {
        in = (*Src++ + offset) * 2147483648.0f;
        *Dst++ = clip_q63_to_q31((q63_t)in);
        blkCnt--;
    }
#endif
}

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
void convert_float_to_q15(q15_t *Dst, const float32_t *Src, uint32_t length, float32_t offset)
{
#if CONVERT_SWITCH
    if (offset == 0.0f)
    {
        // 无偏置时直接调用 CMSIS-DSP 内部已 SIMD 优化的实现
        arm_float_to_q15(Src, Dst, length);
        return;
    }

    uint32_t blkCnt = length >> 2U; // 块计数 4x 展开主循环 后续要做剩余元素的处理
    float32_t in;

    while (blkCnt > 0U)
    {
        in = (Src[0] + offset) * 32768.0f;
        Dst[0] = clip_q31_to_q15((q31_t)in);
        in = (Src[1] + offset) * 32768.0f;
        Dst[1] = clip_q31_to_q15((q31_t)in);
        in = (Src[2] + offset) * 32768.0f;
        Dst[2] = clip_q31_to_q15((q31_t)in);
        in = (Src[3] + offset) * 32768.0f;
        Dst[3] = clip_q31_to_q15((q31_t)in);
        Dst += 4U;
        Src += 4U;
        blkCnt--;
    }

    blkCnt = length & 0x3U;
    while (blkCnt > 0U)
    {
        in = (*Src++ + offset) * 32768.0f;
        *Dst++ = clip_q31_to_q15((q31_t)in);
        blkCnt--;
    }
#endif
}

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
void convert_float_to_q7(q7_t *Dst, const float32_t *Src, uint32_t length, float32_t offset)
{
#if CONVERT_SWITCH
    if (offset == 0.0f)
    {
        // 无偏置时直接调用 CMSIS-DSP 内部已 SIMD 优化的实现
        arm_float_to_q7(Src, Dst, length);
        return;
    }

    uint32_t blkCnt = length >> 2U; // 块计数 4x 展开主循环 后续要做剩余元素的处理
    float32_t in;

    while (blkCnt > 0U)
    {
        in = (Src[0] + offset) * 128.0f;
        Dst[0] = clip_q31_to_q7((q31_t)in);
        in = (Src[1] + offset) * 128.0f;
        Dst[1] = clip_q31_to_q7((q31_t)in);
        in = (Src[2] + offset) * 128.0f;
        Dst[2] = clip_q31_to_q7((q31_t)in);
        in = (Src[3] + offset) * 128.0f;
        Dst[3] = clip_q31_to_q7((q31_t)in);
        Dst += 4U;
        Src += 4U;
        blkCnt--;
    }

    blkCnt = length & 0x3U;
    while (blkCnt > 0U)
    {
        in = (*Src++ + offset) * 128.0f;
        *Dst++ = clip_q31_to_q7((q31_t)in);
        blkCnt--;
    }
#endif
}
