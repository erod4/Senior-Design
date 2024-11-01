
# STM32 ADC with DMA and TIM7 Timer Configuration for Real-Time FFT Analysis

This documentation provides a step-by-step guide for configuring the STM32 ADC (Analog-to-Digital Converter) for real-time FFT (Fast Fourier Transform) analysis using TIM7 as a trigger source and DMA (Direct Memory Access) for efficient data transfer. This setup allows for high-frequency data sampling and processing, essential in real-time signal processing applications.

---

## Overview

- **ADC**: Analog-to-Digital Converter, sampling data at a consistent rate.
- **TIM7**: Timer that triggers the ADC, allowing precise control over the sampling rate.
- **DMA**: Direct Memory Access, transferring sampled data to memory without CPU involvement.
- **FFT**: Fast Fourier Transform, processes sampled data for real-time analysis.

---

## Table of Contents

1. [ADC Initialization and Configuration](#adc-initialization-and-configuration)
2. [TIM7 Timer Configuration](#tim7-timer-configuration)
3. [DMA Initialization](#dma-initialization)
4. [ADC-DMA Data Flow and Processing](#adc-dma-data-flow-and-processing)
5. [DMA Callback Functions](#dma-callback-functions)

---

## 1. ADC Initialization and Configuration

The `MX_ADC1_Init` function configures ADC1 for precise sampling with DMA. Here’s the initialization code and an explanation of the configuration:

```c
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;   // Clock prescaler to sync with PCLK
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;             // 12-bit resolution
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;             // Right-aligned data
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;             // Single channel mode
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;          // End of Conversion selection
  hadc1.Init.ContinuousConvMode = DISABLE;                // Single conversion mode
  hadc1.Init.NbrOfConversion = 1;                         // Single conversion per trigger
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T7_TRGO; // Trigger by TIM7
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING; // Trigger on rising edge
  hadc1.Init.DMAContinuousRequests = ENABLE;              // Continuous DMA requests
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;            // Preserve data on overrun
  hadc1.Init.OversamplingMode = DISABLE;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  // Configure ADC channel 9 with specified sampling time
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
```

### Key Configuration Details

- **Clock Prescaler**: `ADC_CLOCK_SYNC_PCLK_DIV4` divides the ADC clock to balance sampling rate.
- **Resolution**: 12-bit resolution, providing sufficient data detail for FFT processing.
- **Trigger Source**: Configured to trigger via TIM7 on a rising edge for precise control.
- **DMA Requests**: Enables continuous data transfer, reducing CPU load.
- **Channel Configuration**: Sets Channel 9 with a quick sampling time to minimize delay.

---

## 2. TIM7 Timer Configuration

The `MX_TIM7_Init` function configures TIM7 to trigger ADC conversions at regular intervals, ensuring a consistent sampling rate for accurate FFT analysis.

```c
void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 16 - 1;                       // Clock prescaler to divide system clock by 16
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;         // Count upwards
  htim7.Init.Period = 24 - 1;                          // Period for desired sampling rate
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE; // Set TRGO on update to trigger ADC
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
```

### Key Configuration Details

- **Prescaler and Period**: Configures TIM7’s output frequency to the desired ADC sampling rate.
- **Master Output Trigger**: Sends a TRGO signal on every counter update, which initiates ADC conversion.

---

## 3. DMA Initialization

The `MX_DMA_Init` function configures DMA channels for smooth data transfer from ADC to memory, enabling efficient buffer management without CPU involvement.

```c
void MX_DMA_Init(void)
{
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  // Initialize DMA interrupts for handling half and full buffer events
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}
```

### Key Configuration Details

- **Clock Enable**: Activates DMA clocks, allowing the DMA controller to function.
- **Interrupts**: Configures interrupts for handling half-buffer and full-buffer events to avoid data loss.

---

## 4. ADC-DMA Data Flow and Processing

The DMA transfers data from the ADC to an `adc_buffer` in two halves, flagged by `HALF_BUFFER_FULL_FLAG` and `FULL_BUFFER_FULL_FLAG`. These flags help monitor data availability for efficient processing without overwriting. 

### Main Processing Loop

In the main loop, flags indicate when each half of `adc_buffer` is ready. When set, the corresponding buffer is processed, with ADC values centered around 0 before FFT processing.

```c
while (1)
{
  if (HALF_BUFFER_FULL_FLAG)
  {
    for (int n = 0; n < HALF_BUFFER_SIZE; n++)
    {
      int16_t centered_value = (int16_t)adc_buffer[n] - 2048;
      Q15_fft_buffer_in1[n] = (q15_t)(centered_value << 4);
    }
    HALF_BUFFER_FULL_FLAG = 0;

    perform_fft(Q15_fft_buffer_in1, Q15_fft_buffer_out1);
  }

  if (FULL_BUFFER_FULL_FLAG)
  {
    for (int n = HALF_BUFFER_SIZE; n < BUFFER_SIZE; n++)
    {
      int16_t centered_value = (int16_t)adc_buffer[n] - 2048;
      Q15_fft_buffer_in2[n - HALF_BUFFER_SIZE] = (q15_t)(centered_value << 4);
    }
    FULL_BUFFER_FULL_FLAG = 0;

    perform_fft(Q15_fft_buffer_in2, Q15_fft_buffer_out2);
  }
}
```

---

## 5. DMA Callback Functions

The `HAL_ADC_ConvHalfCpltCallback` and `HAL_ADC_ConvCpltCallback` functions handle half-buffer and full-buffer completion interrupts, setting flags to signal data availability.

```c
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (!HALF_BUFFER_FULL_FLAG)
  {
    HALF_BUFFER_FULL_FLAG = 1;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (!FULL_BUFFER_FULL_FLAG)
  {
    FULL_BUFFER_FULL_FLAG = 1;
  }
}
```

---

With this configuration, the STM32 ADC, DMA, and TIM7 Timer operate seamlessly to enable real-time FFT analysis with minimal CPU load. This setup is ideal for applications requiring continuous signal monitoring and frequency analysis.

