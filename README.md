# [DSP](https://github.com/Travelerrrrrr/DSP)

## [RFFT](https://github.com/Travelerrrrrr/DSP/tree/main/RFFT)

基于ARM CMSIS-DSP的实数FFT（RFFT）服务层，提供高效的频域分析和逆变换功能。

### 核心功能

- **正向变换**
  - `rfft_start` - 输出packed复数频域数据
  - `rfft_start_af` - 输出单边幅频结果（支持原始值/dBV/dBFS单位）
  - `rfft_start_pf` - 输出单边相频结果（支持弧度/角度）
  - `rfft_start_af_pf` - 同时输出幅频和相频结果

- **逆向变换**
  - `irfft_start_compensation` - 根据幅频、相频和补偿曲线执行逆RFFT
  - `irfft_start_construction` - 从幅度和相位信息构建频域数据后执行逆变换

### 特性

- 支持8种窗函数（矩形窗、汉宁窗、汉明窗、Bartlett、Welch、Nuttall3/4、Blackman-Harris）
- ADC数据直接处理，支持偏置补偿
- 可选DMA搬运或memcpy数据拷贝
- 优化的对数下限/上限定义用于幅值计算
- 高效的相位和幅值计算

### 使用说明

#### 1. 初始化句柄
```c
rfft_handle_t hrfft;
// 不使用DMA，使用memcpy拷贝数据
rfft_handle_init(&hrfft, NULL); // 数据量小不推荐使用DMA
```

#### 2. 获取幅频结果
```c
float32_t result_buffer[FFT_LENGTH];   // FFT结果
uint32_t adc_buffer_addr[FFT_LENGTH];  // ADC数据地址

// 获取dBV单位的幅频数据
rfft_start_af(&hrfft, 
    (uint32_t)adc_buffer_addr,
              result_buffer,
              RFFT_WINDOW_HANNING,  // 使用汉宁窗
              RFFT_UNIT_DBV,        // 输出单位：dBV
              0.0f);                // 无偏置
// 结果存储在 result_buffer[0..RFFT_HALF_LENGTH]
```

#### 3. 同时获取幅频和相频
```c
float32_t af_pf_buffer[FFT_LENGTH + 2];

rfft_start_af_pf(&hrfft,
       (uint32_t)adc_buffer_addr,
                 af_pf_buffer,
                 RFFT_WINDOW_HAMMING,  // 使用汉明窗
                 RFFT_UNIT_DBV,         // 幅值单位：dBV
                 1,                     // 相位单位：1=角度，0=弧度
                 0.0f);
// 结果交错存储：[amp0, phase0, amp1, phase1, ...]
```

#### 4. 逆变换 - 从幅度和相位重建时域信号
```c
float32_t AF[RFFT_HALF_LENGTH + 1];  // 幅度数组
float32_t PF[RFFT_HALF_LENGTH + 1];  // 相位数组
float32_t irfft_result[FFT_LENGTH];  // 时域输出

irfft_start_construction(&hrfft,
                         irfft_result,
                         AF,
                         PF,
                         0);  // 0表示PF单位为弧度，1表示为角度
```

#### 参数说明

| 参数 | 说明 |
|------|------|
| `adc_data_addr` | ADC采样数据缓冲区地址 |
| `offset` | ADC数据转换为float32后叠加的偏置值，0.0f表示不调整 |
| `window` | 窗函数类型，推荐使用RFFT_WINDOW_HANNING或RFFT_WINDOW_HAMMING |
| `unit` | 幅值输出单位：RFFT_UNIT_RAW（原始值）、RFFT_UNIT_DBV、RFFT_UNIT_DBFS |
| `p_unit` | 相位单位：0=弧度，非0=角度 |

## [FILTER_CHECK](https://github.com/Travelerrrrrr/DSP/tree/main/FILTER_CHECK)

滤波器特性自动检测模块，通过分析幅频曲线自动识别滤波器类型。

### 支持的滤波器类型

- **FullPass** - 全通滤波器
- **Lowpass** - 低通滤波器
- **Highpass** - 高通滤波器
- **BandPass** - 带通滤波器
- **BandStop** - 带阻滤波器

### 功能特性

- 支持多种数据格式：8/16/32位无符号整数、单精度浮点数
- 支持多种幅值单位：原始ADC数据、dBV、dBFS
- 自动识别滤波器的通带和阻带特性
- 返回上升沿和下降沿位置信息
- 优化的阈值计算公式
- 支持任意数据长度分析

### 使用场景

用于滤波器设计验证、频率响应测试以及信号处理管道中的自动化特性检测

