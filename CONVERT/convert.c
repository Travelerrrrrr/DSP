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
#if USE_CONVERT
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
#if USE_CONVERT
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
