# DSP

作者在做毕设过程中写的代码，后续会持续更新

面向 Cortex-M 工程的轻量 DSP 工具代码，主要基于 ARM CMSIS-DSP，当前包含三个模块：

- `RFFT`: 实数 FFT 服务层，支持 ADC 原始数据到频域结果的常用处理流程。
- `CONVERT`: 数值格式转换工具，支持 `uint32_t`、`float32_t`、Q31、Q15、Q7 等格式。
- `FILTER_CHECK`: 幅频曲线类型识别工具，用于判断全通、低通、高通、带通、带阻等滤波器特性。

## 目录结构

```text
DSP/
├── dsp_config.h
├── CONVERT/
│   ├── convert.c
│   └── convert.h
├── FILTER_CHECK/
│   ├── filter_check.c
│   └── filter_check.h
└── RFFT/
    ├── arm_rfft_service.c
    └── arm_rfft_service.h
```

## 依赖

- ARM CMSIS-DSP。
- 工程中需要能找到 `arm_math.h`。
- 使用 RFFT 时需要链接 CMSIS-DSP 对应目标平台的库文件，或把 CMSIS-DSP 源码加入工程。
- 需要启用 FPU 或确认 CMSIS-DSP 的浮点实现与目标芯片配置匹配。

## 接入工程

1. 将本目录加入工程，例如放到 `User/DSP` 或其他固定目录。
2. 将需要的 `.c` 文件加入编译：
   - 使用 RFFT: `RFFT/arm_rfft_service.c` 和 `CONVERT/convert.c`
   - 只使用转换工具: `CONVERT/convert.c`
   - 使用滤波器识别: `FILTER_CHECK/filter_check.c`
3. 将本目录、`RFFT`、`CONVERT`、`FILTER_CHECK` 以及 CMSIS-DSP 头文件目录加入 include path。
4. 按实际目录检查源码里的 `#include "../User/DSP/..."` 路径。如果你的工程没有使用 `User/DSP` 这个目录结构，需要统一调整 include 路径。
5. 在 `dsp_config.h` 中配置模块开关、FFT 长度和 ADC 参数。

## 全局配置

`dsp_config.h` 是全局配置入口：

```c
#define USE_ARM_FFT 1
#define FFT_LENGTH 2048U
#define ADC_RESOLUTION_BITS 12U
#define ADC_FULL_SCALE_VOLTAGE 3.3f

#define USE_CONVERT 1

#define USE_FILTER_CHECK 0
#define FILTER_CHECK_FLOAT 0
```

配置说明：

| 配置项                   | 说明                                                           |
| ------------------------ | -------------------------------------------------------------- |
| `USE_ARM_FFT`            | 是否启用 RFFT 模块。                                           |
| `FFT_LENGTH`             | FFT 点数，当前 RFFT 初始化支持 `512`、`1024`、`2048`、`4096`。 |
| `ADC_RESOLUTION_BITS`    | ADC 位宽，用于 dBV/dBFS 换算。                                 |
| `ADC_FULL_SCALE_VOLTAGE` | ADC 满量程电压，用于 dBV/dBFS 换算。                           |
| `USE_CONVERT`            | 是否启用转换工具。RFFT 启用时转换工具也会被使用。              |
| `USE_FILTER_CHECK`       | 是否启用滤波器类型识别模块。                                   |
| `FILTER_CHECK_FLOAT`     | 是否启用浮点数组扫描分支。                                     |

## RFFT 使用说明

### 功能概览

RFFT 模块封装了以下流程：

1. 从 ADC 采样缓冲区读取 `uint32_t` 数据。
2. 转换为 `float32_t`，可叠加 `offset` 做偏置修正。
3. 可选加窗。
4. 调用 CMSIS-DSP `arm_rfft_fast_f32`。
5. 输出 packed 复数频域、单边幅频、单边相频，或幅频/相频交错结果。

支持的窗函数：

| 枚举                               | 说明                     |
| ---------------------------------- | ------------------------ |
| `RFFT_WINDOW_RECTANGULAR`          | 矩形窗                   |
| `RFFT_WINDOW_HANNING`              | Hanning 窗               |
| `RFFT_WINDOW_HAMMING`              | Hamming 窗               |
| `RFFT_WINDOW_BARTLETT`             | Bartlett 窗              |
| `RFFT_WINDOW_WELCH`                | Welch 窗                 |
| `RFFT_WINDOW_NUTTALL3`             | Nuttall 3 项窗           |
| `RFFT_WINDOW_NUTTALL4`             | Nuttall 4 项窗           |
| `RFFT_WINDOW_BLACKMAN_HARRIS_92DB` | Blackman-Harris 92 dB 窗 |

幅值单位：

| 枚举             | 说明                     |
| ---------------- | ------------------------ |
| `RFFT_UNIT_RAW`  | 线性幅值，不做 dB 换算。 |
| `RFFT_UNIT_DBV`  | 转换为 dBV。             |
| `RFFT_UNIT_DBFS` | 转换为 dBFS。            |

相位单位：

| 参数          | 说明             |
| ------------- | ---------------- |
| `p_unit = 0`  | 输出或输入弧度。 |
| `p_unit != 0` | 输出或输入角度。 |

### 初始化

```c
#include "RFFT/arm_rfft_service.h"

static rfft_handle_t hrfft;

void dsp_init(void)
{
    if (rfft_handle_init(&hrfft, NULL) != ARM_MATH_SUCCESS)
    {
        /* 处理 FFT_LENGTH 不支持等初始化错误 */
    }
}
```

第二个参数传入 `NULL` 时，内部直接从 ADC 缓冲区转换数据，不使用 DMA。

如果需要使用 DMA 搬运输入数据，需要提供 `rfft_dma_t`：

```c
static arm_status my_dma_transfer(void *dma_handle,
                                  uint32_t src_addr,
                                  uint32_t dst_addr,
                                  uint32_t size,
                                  uint32_t timeout)
{
    /* 在这里调用平台 DMA 接口，并等待搬运完成 */
    return ARM_MATH_SUCCESS;
}

static rfft_dma_t rfft_dma = {
    .handle = &hdma_memtomem,
    .transfer = my_dma_transfer,
};

void dsp_init_with_dma(void)
{
    (void)rfft_handle_init(&hrfft, &rfft_dma);
}
```

### 获取 packed 复数频域

```c
uint32_t adc_buffer[FFT_LENGTH];
float32_t fft_complex[FFT_LENGTH];

rfft_start(&hrfft,
           (uint32_t)adc_buffer,
           fft_complex,
           RFFT_WINDOW_HANNING,
           0.0f);
```

输出格式为 CMSIS-DSP packed RFFT 格式：

```text
[DC, Nyquist, X1.real, X1.imag, X2.real, X2.imag, ...]
```

### 获取单边幅频

```c
uint32_t adc_buffer[FFT_LENGTH];
float32_t amplitude[FFT_LENGTH];

rfft_start_af(&hrfft,
              (uint32_t)adc_buffer,
              amplitude,
              RFFT_WINDOW_HANNING,
              RFFT_UNIT_DBV,
              0.0f);
```

有效输出范围：

```text
amplitude[0 .. RFFT_HALF_LENGTH]
```

共 `RFFT_HALF_LENGTH + 1` 个点，依次对应 DC、正频率点和 Nyquist 点。

### 获取单边相频

```c
uint32_t adc_buffer[FFT_LENGTH];
float32_t phase[FFT_LENGTH];

rfft_start_pf(&hrfft,
              (uint32_t)adc_buffer,
              phase,
              RFFT_WINDOW_HANNING,
              1,
              0.0f);
```

有效输出范围：

```text
phase[0 .. RFFT_HALF_LENGTH]
```

上例中 `p_unit` 为 `1`，相位单位是角度；传 `0` 时单位是弧度。

### 同时获取幅频和相频

```c
uint32_t adc_buffer[FFT_LENGTH];
float32_t amp_phase[FFT_RESULT_LENGTH];

rfft_start_af_pf(&hrfft,
                 (uint32_t)adc_buffer,
                 amp_phase,
                 RFFT_WINDOW_HAMMING,
                 RFFT_UNIT_DBFS,
                 1,
                 0.0f);
```

输出格式：

```text
[amp0, phase0, amp1, phase1, ..., ampN, phaseN]
```

其中 `N = RFFT_HALF_LENGTH`，所以输出缓冲区长度至少为 `FFT_LENGTH + 2`，也可以直接使用宏 `FFT_RESULT_LENGTH`。

### 频域补偿后逆变换

适合先做正向 RFFT，再根据幅度补偿曲线 `AF` 和相位补偿曲线 `PF` 修正频域结果，最后回到时域。

```c
float32_t fft_complex[FFT_LENGTH];
float32_t irfft_output[FFT_LENGTH];
float32_t AF[RFFT_HALF_LENGTH + 1];
float32_t PF[RFFT_HALF_LENGTH + 1];

rfft_start(&hrfft,
           (uint32_t)adc_buffer,
           fft_complex,
           RFFT_WINDOW_RECTANGULAR,
           0.0f);

irfft_start_compensation(&hrfft,
                         fft_complex,
                         irfft_output,
                         AF,
                         PF,
                         0);
```

`AF` 和 `PF` 的下标含义：

| 下标                        | 含义         |
| --------------------------- | ------------ |
| `0`                         | DC           |
| `1 .. RFFT_HALF_LENGTH - 1` | 普通正频率点 |
| `RFFT_HALF_LENGTH`          | Nyquist 点   |

### 从幅度和相位重建时域信号

如果已经有单边幅度 `AF` 和相位 `PF`，可以直接构建 packed 频域数据并执行逆 RFFT：

```c
float32_t AF[RFFT_HALF_LENGTH + 1];
float32_t PF[RFFT_HALF_LENGTH + 1];
float32_t irfft_output[FFT_LENGTH];

irfft_start_construction(&hrfft,
                         irfft_output,
                         AF,
                         PF,
                         0);
```

注意：`irfft_output` 不能和输入/内部工作缓冲区共用同一块内存。

## CONVERT 使用说明

转换模块用于采样数据和 CMSIS-DSP 常用格式之间的转换。

```c
#include "CONVERT/convert.h"

uint32_t raw[FFT_LENGTH];
float32_t f32[FFT_LENGTH];
q15_t q15[FFT_LENGTH];

convert_uint32_to_float(f32, raw, FFT_LENGTH, 0.0f);
convert_float_to_q15(q15, f32, FFT_LENGTH, 0.0f);
```

可用接口：

| 函数                      | 说明                        |
| ------------------------- | --------------------------- |
| `convert_uint32_to_float` | `uint32_t` 转 `float32_t`。 |
| `convert_float_to_uint32` | `float32_t` 转 `uint32_t`。 |
| `convert_float_to_q31`    | `float32_t` 转 Q31。        |
| `convert_float_to_q15`    | `float32_t` 转 Q15。        |
| `convert_float_to_q7`     | `float32_t` 转 Q7。         |

`offset` 参数会在转换时叠加到输入值上。常见用途是去除 ADC 中点偏置，例如 12 bit ADC 的中点可传入 `-2048.0f`。

## FILTER_CHECK 使用说明

该模块用于根据一段幅频曲线和阈值判断滤波器类型，返回：

- `FILTER_FULLPASS`
- `FILTER_LOWPASS`
- `FILTER_HIGHPASS`
- `FILTER_BANDPASS`
- `FILTER_BANDSTOP`
- `FILTER_NULL`

`FILTER_INFO` 中的字段含义：

| 字段    | 说明                 |
| ------- | -------------------- |
| `pRais` | 检测到的上升沿位置。 |
| `pFall` | 检测到的下降沿位置。 |
| `type`  | 识别出的滤波器类型。 |

当前 `dsp_config.h` 中默认关闭：

```c
#define USE_FILTER_CHECK 0
```

启用前建议先检查 `FILTER_CHECK/filter_check.h` 和 `FILTER_CHECK/filter_check.c` 的 `filter_check` 函数声明是否一致，再将 `USE_FILTER_CHECK` 改为 `1`。

## 常见注意事项

- RFFT 输入采样缓冲区当前按 `uint32_t` 处理，长度必须至少为 `FFT_LENGTH`。
- `rfft_start_af` 和 `rfft_start_pf` 的输出缓冲区长度至少为 `FFT_LENGTH`，虽然有效结果只有 `RFFT_HALF_LENGTH + 1` 个点。
- `rfft_start_af_pf` 的输出缓冲区长度至少为 `FFT_LENGTH + 2`。
- 使用 dBV/dBFS 时，请确认 `ADC_RESOLUTION_BITS` 和 `ADC_FULL_SCALE_VOLTAGE` 与硬件一致。
- 使用加窗时，幅值结果已按窗函数平均增益做补偿。
- 默认 `RFFT_ERROR_HANDLER` 是弱定义死循环函数，实际项目中建议重写同名函数，改为日志、断言或错误上报。
- 如果编译器提示找不到头文件，优先检查 include path 和源码里的相对 include 路径是否与工程目录一致。
