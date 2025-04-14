#include <main.h>
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <timer.h>

TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim6;

void MX_TIM8_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig={0};
	__HAL_RCC_TIM8_CLK_ENABLE();

	htim8.Instance=TIM8;
	htim8.Init.Prescaler=4096-1;
	htim8.Init.CounterMode=TIM_COUNTERMODE_UP;
	htim8.Init.Period=2159-1;
	htim8.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
	if(HAL_TIM_Base_Init(&htim8)!=HAL_OK)
	{
		printf("Error Occurred with Timer 8 Init!\r\n");
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger=TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode=TIM_MASTERSLAVEMODE_DISABLE;

	 if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	  {
		printf("Error Occurred with Timer 8 Config!\r\n");
		Error_Handler();
	  }
	 HAL_NVIC_SetPriority(TIM8_UP_IRQn, 0, 0);
	 HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);
}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 65500-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 17-1;
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


}
void MX_TIM6_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig = {0};
    __HAL_RCC_TIM6_CLK_ENABLE();

	htim6.Instance=TIM6;
	htim6.Init.Prescaler=170-1;
	htim6.Init.CounterMode=TIM_COUNTERMODE_UP;
	htim6.Init.Period=1000-1;
	htim6.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
	if(HAL_TIM_Base_Init(&htim6)!=HAL_OK)
	{
		printf("Error Occurred with Timer 6 Init!\r\n");
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger=TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode=TIM_MASTERSLAVEMODE_DISABLE;

	 if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	  {
		printf("Error Occurred with Timer 6 Config!\r\n");
	    Error_Handler();
	  }

}


