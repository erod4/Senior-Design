#include "dac.h"
#include "stm32g4xx_hal.h"
#include "main.h"

//DAC_HandleTypeDef hdac1;
//DMA_HandleTypeDef hdma_dac1_ch1;


/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
//void MX_DAC1_Init(void)
//{
//
//  /* USER CODE BEGIN DAC1_Init 0 */
//
//  /* USER CODE END DAC1_Init 0 */
//
//  DAC_ChannelConfTypeDef sConfig = {0};
//
//  /* USER CODE BEGIN DAC1_Init 1 */
//
//  /* USER CODE END DAC1_Init 1 */
//
//  /** DAC Initialization
//  */
//  hdac1.Instance = DAC1;
//  if (HAL_DAC_Init(&hdac1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** DAC channel OUT1 config
//  */
//  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
//  sConfig.DAC_DMADoubleDataMode = DISABLE;
//  sConfig.DAC_SignedFormat = DISABLE;
//  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
//  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
//  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
//  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
//  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
//  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
//  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN DAC1_Init 2 */
//
//  /* USER CODE END DAC1_Init 2 */
//
//}
