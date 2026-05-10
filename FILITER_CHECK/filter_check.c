/******************************************************************************
 * @file filter_check.chis file contains the implementation of the filter_check function, which is used to determine the type of filter based on the provided parameters.
 * @author Analog__
 * @date 2026/4/19 V1.0
 * @date 2026/4/22 V1.2
 * @date 2026/4/30 V1.8
 * @version 1.8
 * @note 判断滤波器类型，需要根据项目实际情况进行调整堆大小，以避免Stack Overflow，此函数暂未用到堆，如需
 *       精确的阈值判断，必须使用arm c library，不可使用任何microlib，若不在意速度以及阈值精度，推荐打开
 *       microlib。
 ******************************************************************************/

// #include "includes.h"

#include "filter_check.h"
// #include "arm_math.h"
#include "math.h"
#include <stdint.h>
#include <stdlib.h>

#define assertFILTER_CHECK(x)   \
    do                          \
    {                           \
        if (!(x))               \
        {                       \
            return filter_info; \
        }                       \
    } while (0)

static uint32_t read_u8(void *p) { return *(uint8_t *)p; }
static uint32_t read_u16(void *p) { return *(uint16_t *)p; }
static uint32_t read_u32(void *p) { return *(uint32_t *)p; }
static uint32_t automatic_read(void *p, uint8_t Msize)
{
    switch (Msize)
    {
    case 8:
        return read_u8(p);
    case 16:
        return read_u16(p);
    case 32:
        return read_u32(p);
    default:
        return 0;
    }
}
#ifdef FILTER_CHECK_FLOAT
static float read_f32(void *p) { return *(float *)p; }
#endif
/**
 * @brief 判断滤波器类型
 * @param addr 数据地址
 * @param Msize 数据位宽，必须是8的倍数, 0表示为float类型
 * @param Length 数据长度，必须大于0
 * @param threshold dBV阈值
 * @param BitWidth 位宽，默认为12
 * @param ADC_Ref ADC参考电压，默认为3.3V
 * @return FILTER_INFO 滤波器信息结构体
 */
#ifdef FILTER_CHECK_INFO
FILTER_INFO filter_check(uint32_t addr, uint8_t Msize, uint16_t Length, float threshold, uint8_t BitWidth, float ADC_Ref)
{
    FILTER_INFO filter_info = {0, 0, FILTER_NULL};

    assertFILTER_CHECK(addr != 0);
    assertFILTER_CHECK((Msize % 8) == 0);
    assertFILTER_CHECK(Length > 0);

    if (BitWidth == 0)
        BitWidth = 12;

    if (ADC_Ref == 0)
        ADC_Ref = 3.3f;

    float vFFTDATAThreshold = expf(threshold * 0.11512925464970229f) * 1.4142135623730950488016f / (ADC_Ref / (1 << BitWidth));

    uint8_t _addr_Inc = Msize / 8, find = 0;

    uint16_t i, j;

    uint16_t fullpass = 1;

    if (_addr_Inc != 0)
    {
        for (i = 0; i < Length; i++)
        {
            if (automatic_read(addr + i * _addr_Inc, Msize) < vFFTDATAThreshold)
            {
                fullpass = 0;
                break;
            }
        }

        if (fullpass)
        {
            find = 1;
            filter_info.pRais = 0;
            filter_info.pFall = 0;
            filter_info.type = FILTER_FULLPASS;
            return filter_info;
        }

        for (i = 1; i < Length - 1; i++)
        {
            if (automatic_read(addr + i * _addr_Inc - _addr_Inc, Msize) < vFFTDATAThreshold && automatic_read(addr + i * _addr_Inc + _addr_Inc, Msize) >= vFFTDATAThreshold)
            {
                filter_info.pRais = i;
                for (j = i + 2; j < Length - 1; j++)
                {
                    if (automatic_read(addr + j * _addr_Inc - _addr_Inc, Msize) >= vFFTDATAThreshold && automatic_read(addr + j * _addr_Inc + _addr_Inc, Msize) < vFFTDATAThreshold)
                    {
                        filter_info.pFall = j;
                        find = 1;
                        filter_info.type = FILTER_BANDPASS;
                        break;
                    }
                }
                if (!find)
                {
                    for (i = 1; i < Length - 1; i++)
                    {
                        if (automatic_read(addr + i * _addr_Inc - _addr_Inc, Msize) >= vFFTDATAThreshold && automatic_read(addr + i * _addr_Inc + _addr_Inc, Msize) < vFFTDATAThreshold)
                        {
                            filter_info.pFall = i;
                            find = 1;
                            filter_info.type = FILTER_BANDSTOP;
                            break;
                        }
                    }
                    if (!find)
                    {
                        find = 1;
                        filter_info.type = FILTER_HIGHPASS;
                        break;
                    }
                }
            }
        }

        if (filter_info.type == FILTER_NULL)
        {
            for (i = 1; i < Length - 1; i++)
            {
                if (automatic_read(addr + i * _addr_Inc - _addr_Inc, Msize) >= vFFTDATAThreshold && automatic_read(addr + i * _addr_Inc + _addr_Inc, Msize) < vFFTDATAThreshold)
                {
                    filter_info.pFall = i;
                    find = 1;
                    filter_info.type = FILTER_LOWPASS;
                    break;
                }
            }
        }
    }
    else
    {
#ifdef FILTER_CHECK_FLOAT
        for (i = 0; i < Length; i++)
        {
            if (read_f32(addr + i * 4) < vFFTDATAThreshold)
            {
                fullpass = 0;
                break;
            }
        }

        if (fullpass)
        {
            find = 1;
            filter_info.pRais = 0;
            filter_info.pFall = 0;
            filter_info.type = FILTER_FULLPASS;
            return filter_info;
        }

        for (i = 1; i < Length - 1; i++)
        {
            if (read_f32(addr + i * 4 - 4) < vFFTDATAThreshold && read_f32(addr + i * 4 + 4) >= vFFTDATAThreshold)
            {
                filter_info.pRais = i;
                for (j = i + 2; j < Length - 1; j++)
                {
                    if (read_f32(addr + j * 4 - 4) >= vFFTDATAThreshold && read_f32(addr + j * 4 + 4) < vFFTDATAThreshold)
                    {
                        filter_info.pFall = j;
                        find = 1;
                        filter_info.type = FILTER_BANDPASS;
                        break;
                    }
                }
                if (!find)
                {
                    for (i = 1; i < Length - 1; i++)
                    {
                        if (read_f32(addr + i * 4 - 4) >= vFFTDATAThreshold && read_f32(addr + i * 4 + 4) < vFFTDATAThreshold)
                        {
                            filter_info.pFall = i;
                            find = 1;
                            filter_info.type = FILTER_BANDSTOP;
                            break;
                        }
                    }
                    if (!find)
                    {
                        find = 1;
                        filter_info.type = FILTER_HIGHPASS;
                        break;
                    }
                }
            }
        }
#else
        filter_info.type = FILTER_NULL;
#endif
    }

    return filter_info;
}
#endif
