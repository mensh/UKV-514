
#include "timCore.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;


/* TIM2 */
#if !defined( USB_HARDWARE_FS ) && defined( USB_HARDWARE_HS_MC_REFCLK )

TIM_HandleTypeDef htim2;

void MX_TIM2_Init(void)
	{
	TIM_OC_InitTypeDef sConfigOC;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&htim2);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
	}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
	{
	GPIO_InitTypeDef GPIO_InitStruct;
	if( htim_pwm->Instance == TIM2 )
		{
		/* Peripheral clock enable */
		__TIM2_CLK_ENABLE();

		/**TIM2 GPIO Configuration
		PA2     ------> TIM2_CH3
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		}
	}

void TIM_CORE_initTim2(void)
	{
	HAL_TIM_PWM_MspInit(&htim2);
	MX_TIM2_Init();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	}
#endif
void MX_TIM3_Init(void)
{
	
  //TIM_SlaveConfigTypeDef sSlaveConfig;
  //TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 14999;//11999;//23999; //29850;;;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);
	HAL_TIM_Base_Start_IT(&htim3);


}
/* TIM7 init function */
void MX_TIM7_Init(void)
{
	//uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / 400) - 1;
 // TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 2;//1;//2;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period =49999;//  59999;//49999;
  HAL_TIM_Base_Init(&htim7);

	HAL_TIM_Base_Start_IT(&htim7);

}
TIM_HandleTypeDef  htim;
void TIM6_Config(void)
{
	
	

  TIM_MasterConfigTypeDef sMasterConfig;
	__TIM6_CLK_ENABLE();
  /*##-1- Configure the TIM peripheral #######################################*/
  /* Time base configuration */
  htim.Instance = TIM6;

  htim.Init.Period = 0x7FF*3;
  htim.Init.Prescaler = 0;
  htim.Init.ClockDivision = 0;
  htim.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&htim);

  /* TIM6 TRGO selection */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);

  /*##-2- Enable TIM peripheral counter ######################################*/
  HAL_TIM_Base_Start(&htim);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  /* TIM6 Periph clock enable */
		
	
 if(htim_base->Instance==TIM6)
	{
		__TIM6_CLK_ENABLE();
	}
	
	
  if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __TIM3_CLK_ENABLE();

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(htim_base->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspInit 0 */

  /* USER CODE END TIM7_MspInit 0 */
    /* Peripheral clock enable */
    __TIM7_CLK_ENABLE();

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspInit 1 */

  /* USER CODE END TIM7_MspInit 1 */
  }
}
