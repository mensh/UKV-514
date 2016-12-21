#include "adcCore.h"

#if !defined( USB_HARDWARE_FS ) && !defined( USB_HARDWARE_HS_VBUS_DETECT_BY_PIN )

static ADC_HandleTypeDef AdcHandle;

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
	{
	GPIO_InitTypeDef GPIO_InitStruct;

	if( hadc->Instance == ADC1 )
		{
		/* Peripheral clock enable */
		__ADC1_CLK_ENABLE();

		/**ADC1 GPIO Configuration
		PA0-WKUP     ------> ADC1_IN0
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		}

	}

tdStatus ADC_CORE_init(void)
	{
	ADC_ChannelConfTypeDef sConfig;

  AdcHandle.Instance = ADC1;
  AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  AdcHandle.Init.Resolution = ADC_RESOLUTION12b;
  AdcHandle.Init.ScanConvMode = DISABLE;
  AdcHandle.Init.ContinuousConvMode = DISABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.NbrOfDiscConversion = 1;
  AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.NbrOfConversion = 1;
  AdcHandle.Init.DMAContinuousRequests = DISABLE;
  AdcHandle.Init.EOCSelection = EOC_SINGLE_CONV;

	if( HAL_ADC_Init(&AdcHandle) != HAL_OK )
		{
		/* Initiliazation Error */
		return Fail;
		}

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;

	if( HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK )
		{
		/* Channel Configuration Error */
		return Fail;
		}
	
	if( HAL_ADC_Start(&AdcHandle) != HAL_OK )
		{
		/* Start Conversation Error */
		return Fail;
		}

	HAL_ADC_PollForConversion(&AdcHandle, 10);

	return Ok;
	}

tdStatus ADC_CORE_startConversion(void)
	{
	if( HAL_ADC_Start(&AdcHandle) != HAL_OK )
		{
		/* Start Conversation Error */
		return Fail;
		}

	HAL_ADC_PollForConversion(&AdcHandle, 10);

	return Ok;
	}
	
tdStatus ADC_CORE_getValue(uint16_t *_pValue)
	{
	/* Check if the continous conversion of regular channel is finished */
	if( HAL_ADC_GetState(&AdcHandle) == HAL_ADC_STATE_EOC_REG )
		{
		*_pValue = HAL_ADC_GetValue(&AdcHandle);
		return Ok;
		}

	return Busy;
	}

#endif

