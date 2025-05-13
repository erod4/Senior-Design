

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with bar drawing and LCD functions
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include <stdlib.h>
#include <math.h>
#include "arm_math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "UART.h"
#include <stdio.h>
#include "lcd.h"
#include "fft.h"
#include "iir.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch2;

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
// Bar parameters for testing the drawing functions.

#define BUFFER_SIZE			128
#define HALF_BUFFER_SIZE 	64
#define FFT_BUFFER_SIZE 	64

#define LCD_SCALE_FACTOR	30

static long filter_registers[FFT_BUFFER_SIZE/2];



uint32_t adc_buffer				[BUFFER_SIZE];				//Holds DMA data [0...HALF_BUFFER_SIZE-1,HALF_BUFFER_SIZE...BUFFER_SIZE-1]
uint32_t dac_buffer				[BUFFER_SIZE];				//ignore (this buffer is to output data via the DAC, which we dont use)

float32_t fft_buffer_in_1		[FFT_BUFFER_SIZE];			//Holds data from 1st half of DMA buffer which will have an FFT performed on it
float32_t fft_buffer_out_1		[FFT_BUFFER_SIZE];			//Holds output of the FFT in {real[0],img[0],real[1],img[1],...,real[FFT_BUFFER_SIZE-1],img[FFT_BUFFER_SIZE-1]


float32_t fft_buffer_out_2		[FFT_BUFFER_SIZE];			//Holds data from 2nd half of DMA buffer which will have an FFT performed on it
float32_t fft_buffer_in_2		[FFT_BUFFER_SIZE];			//Holds output of the FFT in {real[0],img[0],real[1],img[1],...,real[FFT_BUFFER_SIZE-1],img[FFT_BUFFER_SIZE-1]


float32_t iir_buffer_in_1		[FFT_BUFFER_SIZE/2];		//After the FFT magntiude is computed (sqrt(real^2+img^2)) the magntiudes are stored inside buffer for the LPF
float32_t iir_buffer_in_2		[FFT_BUFFER_SIZE/2];		//Size is FFT_BUFFER_SIZE/2 since the magnitude will give us N/2 values


int iir_buffer_out_1			[FFT_BUFFER_SIZE/2];		//Output of IIR filter is stored in this buffer. I think this can be used as the LCD buffer!!
int iir_buffer_out_2			[FFT_BUFFER_SIZE/2];

int32_t lcd_buffer_1			[FFT_BUFFER_SIZE/2];
int32_t lcd_buffer_2			[FFT_BUFFER_SIZE/2];

volatile uint8_t HALF_BUFFER_FULL_FLAG							=	0;		//Flag is set via DMA ISR which signals the first half of DMA buffer is ready to be processed
volatile uint8_t FULL_BUFFER_FULL_FLAG							=	0;		//Flag is set via DMA ISR which signals the second half of DMA buffer is ready to be processed
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM7_Init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1,adc_buffer,BUFFER_SIZE);
  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_2,dac_buffer,BUFFER_SIZE,DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim7);
  uart_init();
  SPI_init();
  LCD_reset();
  HAL_Delay(100);
  LCD_init();
  init_fft(FFT_BUFFER_SIZE);

  LCD_Fill_Color(BLACK);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(HALF_BUFFER_FULL_FLAG)
	  	{


	  		//Once converted to floating point the FFT is performed on the first half of buffer
//	  		perform_fft(fft_buffer_in_1, fft_buffer_out_1);
	  		perform_fft(fft_buffer_in_1, fft_buffer_out_1);

	  		//Take absolute magnitude of FFT output
	  		arm_cmplx_mag_f32(fft_buffer_out_1,iir_buffer_in_1,(uint32_t)(FFT_BUFFER_SIZE/2));

	  		//Do IIR filtering for each FFT value
	  		for(int i=0;i<FFT_BUFFER_SIZE/2;i++)
	  				{
	  					iir_buffer_out_1[i]=lowpass_IIR_filter((int)(round(iir_buffer_in_1[i])),&filter_registers[i]);
	  				}
	  		//Lastly store in a LCD screen buffer
	  		int temp=31;
	  		for(int i=0; i<FFT_BUFFER_SIZE/2;i++)
	  		{
	  			//Values inside update bar graph function should be reversed (dont touch)
	  			lcd_buffer_1[temp]=(int32_t)(iir_buffer_out_1[i]*(LCD_SCALE_FACTOR));
				temp--;

	  		}

  			updateBarGraph(lcd_buffer_1);
	  		HALF_BUFFER_FULL_FLAG=0;		//Reset the flag letting know ISR it can fill this half of the buffer now


	  	}

	  	if(FULL_BUFFER_FULL_FLAG)
	  	{

		  	//Once converted to floating point the FFT is performed on the second half of buffer
//	  		perform_fft(fft_buffer_in_2, fft_buffer_out_2);
	  		perform_fft(fft_buffer_in_2, fft_buffer_out_2);

	  		//Take absolute magnitude of FFT output
	  		arm_cmplx_mag_f32(fft_buffer_out_2,iir_buffer_in_2,(uint32_t)(FFT_BUFFER_SIZE/2));



	  		//Do IIR filtering for each FFT value
	  		for(int i=0;i<FFT_BUFFER_SIZE/2;i++)
	  		{
  					iir_buffer_out_2[i]=lowpass_IIR_filter((int)(round(iir_buffer_in_2[i])),&filter_registers[i]);

	  		}

	  		//Lastly store in a LCD screen buffer
	  		int temp=31;
	  		for(int i=0; i<FFT_BUFFER_SIZE/2;i++)
	  		{
				lcd_buffer_2[temp]=(int32_t)(iir_buffer_out_2[i]*(LCD_SCALE_FACTOR));
				temp--;
	  		}
  			updateBarGraph(lcd_buffer_2);
	  		FULL_BUFFER_FULL_FLAG=0;

	  	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T7_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
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
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 160-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin|LCD_RST_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RS_Pin LCD_RST_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_RST_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(!HALF_BUFFER_FULL_FLAG)
	{
		float sum = 0.0f;

		for(int i = 0; i < HALF_BUFFER_SIZE; i++)
		{
			fft_buffer_in_1[i] = ((float)adc_buffer[i] / 4095.0f) * 3.3f;
			sum += fft_buffer_in_1[i];
		}

		float mean = sum / HALF_BUFFER_SIZE;

		// Subtract mean to remove DC offset
		for(int i = 0; i < HALF_BUFFER_SIZE; i++)
		{
			fft_buffer_in_1[i] -= mean;
		}

		HALF_BUFFER_FULL_FLAG = 1;
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(!FULL_BUFFER_FULL_FLAG)
	{
		float sum = 0.0f;

		for(int i = HALF_BUFFER_SIZE; i < BUFFER_SIZE; i++)
		{
			fft_buffer_in_2[i - HALF_BUFFER_SIZE] = ((float)adc_buffer[i] / 4095.0f) * 3.3f;
			sum += fft_buffer_in_2[i - HALF_BUFFER_SIZE];
		}

		float mean = sum / HALF_BUFFER_SIZE;

		for(int i = 0; i < HALF_BUFFER_SIZE; i++)
		{
			fft_buffer_in_2[i] -= mean;
		}

		FULL_BUFFER_FULL_FLAG = 1;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
































