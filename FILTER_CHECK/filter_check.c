/******************************************************************************
 * @file filter_check.c
 * @author Analog
 * @date 2026/5/15
 * @version 2.2
 * @note 判断滤波器类型。
 ******************************************************************************/

#include "../User/DSP/filter_check.h"
#include <stdint.h>

#define assertFILTER_CHECK(x)   \
    do                          \
    {                           \
        if (!(x))               \
        {                       \
            return filter_info; \
        }                       \
    } while (0)

// 为了方便维护和避免代码重复，采用宏来定义函数
#define DEFINE_FILTER_SCAN(NAME, TYPE)                                    \
    static _filter_info NAME(uint32_t addr, uint16_t Length, float thr)   \
    {                                                                     \
        _filter_info info = {0, 0, FILTER_NULL};                          \
        const TYPE *p = (const TYPE *)(uintptr_t)addr;                    \
        uint16_t k;                                                       \
                                                                          \
        /* 1) 全通检测：遇到首个低于阈值的点立即退出 */                   \
        for (k = 0; k < Length; k++)                                      \
        {                                                                 \
            if ((float)p[k] < thr)                                        \
                break;                                                    \
        }                                                                 \
        if (k == Length)                                                  \
        {                                                                 \
            info.type = FILTER_FULLPASS;                                  \
            return info;                                                  \
        }                                                                 \
                                                                          \
        /* 2) 滑窗扫描 rise/fall，仅当 Length >= 3 才有意义 */            \
        if (Length >= 3)                                                  \
        {                                                                 \
            uint16_t first_rise = 0, first_fall = 0, fall_after_rise = 0; \
            uint8_t has_rise = 0, has_fall = 0, has_far = 0;              \
            uint8_t bin_bef = ((float)p[0] < thr);                        \
            uint8_t bin_curr = ((float)p[1] < thr);                       \
            uint16_t last = (uint16_t)(Length - 1);                       \
                                                                          \
            for (k = 1; k < last; k++)                                    \
            {                                                             \
                uint8_t bin_aft = ((float)p[k + 1] < thr);                \
                if (bin_bef && !bin_aft)                                  \
                {                                                         \
                    if (!has_rise)                                        \
                    {                                                     \
                        first_rise = k;                                   \
                        has_rise = 1;                                     \
                    }                                                     \
                }                                                         \
                else if (!bin_bef && bin_aft)                             \
                {                                                         \
                    if (!has_fall)                                        \
                    {                                                     \
                        first_fall = k;                                   \
                        has_fall = 1;                                     \
                    }                                                     \
                    if (has_rise && k >= (uint16_t)(first_rise + 2))      \
                    {                                                     \
                        fall_after_rise = k;                              \
                        has_far = 1;                                      \
                        break;                                            \
                    }                                                     \
                }                                                         \
                bin_bef = bin_curr;                                       \
                bin_curr = bin_aft;                                       \
            }                                                             \
                                                                          \
            /* 3) 类型判定 */                                             \
            if (has_far)                                                  \
            {                                                             \
                info.pRais = first_rise;                                  \
                info.pFall = fall_after_rise;                             \
                info.type = FILTER_BANDPASS;                              \
            }                                                             \
            else if (has_rise && has_fall)                                \
            {                                                             \
                info.pRais = first_rise;                                  \
                info.pFall = first_fall;                                  \
                info.type = FILTER_BANDSTOP;                              \
            }                                                             \
            else if (has_rise)                                            \
            {                                                             \
                info.pRais = first_rise;                                  \
                info.type = FILTER_HIGHPASS;                              \
            }                                                             \
            else if (has_fall)                                            \
            {                                                             \
                info.pFall = first_fall;                                  \
                info.type = FILTER_LOWPASS;                               \
            }                                                             \
        }                                                                 \
        return info;                                                      \
    }

DEFINE_FILTER_SCAN(filter_scan_u8, uint8_t)
DEFINE_FILTER_SCAN(filter_scan_u16, uint16_t)
DEFINE_FILTER_SCAN(filter_scan_u32, uint32_t)
#ifdef FILTER_CHECK_FLOAT
DEFINE_FILTER_SCAN(filter_scan_f32, float)
#endif

/**
 * @brief 判断滤波器类型
 * @param addr 数据地址
 * @param Msize 数据位宽，必须是8的倍数, 0表示为float类型
 * @param Length 数据长度，必须大于0
 * @param threshold 阈值
 * @return _filter_info 滤波器信息结构体
 */
_filter_info filter_check(uint32_t addr, uint8_t Msize, uint16_t Length, float threshold)
{
#ifdef USE_FILTER_CHECK
    _filter_info filter_info = {0, 0, FILTER_NULL};

    assertFILTER_CHECK(addr != 0);
    assertFILTER_CHECK((Msize % 8) == 0);
    assertFILTER_CHECK(Length > 0);

    switch (Msize)
    {
    case 8:
        return filter_scan_u8(addr, Length, threshold);
    case 16:
        return filter_scan_u16(addr, Length, threshold);
    case 32:
        return filter_scan_u32(addr, Length, threshold);
#ifdef FILTER_CHECK_FLOAT
    case 0:
        return filter_scan_f32(addr, Length, threshold);
#endif
    default:
        return filter_info;
    }
#endif
}
