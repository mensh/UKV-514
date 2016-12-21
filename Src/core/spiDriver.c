#include "spiDriver.h"
#include "bhd2settings.h"

static tdSpiDriver spiDriver = { NULL, NULL, NULL };

uint8_t SPI_DRIVER_getSpiChannel(SPI_HandleTypeDef *_pSpi)
	{
	uint8_t channel = SPI_DRIVER_CHANNELS;

	if( _pSpi->Instance == SPI1 )
		{
		channel = SPI_DRIVER_CHANNEL_1;
		}
	else if( _pSpi->Instance == SPI2 )
		{
		channel = SPI_DRIVER_CHANNEL_2;
		}
	else if( _pSpi->Instance == SPI3 )
		{
		channel = SPI_DRIVER_CHANNEL_3;
		}

	return channel;
	}

tdStatus SPI_DRIVER_initCS(tdSpiCS *_pCs)
	{
	GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = _pCs->pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  HAL_GPIO_Init(_pCs->pGPIO, &GPIO_InitStruct);

	return Ok;
	}

tdStatus SPI_DRIVER_setCS(tdSpiCS *_pChipSelect, tdEnumSpiCSState _state)
	{
	if( _pChipSelect == NULL )
		return Fail;

	HAL_GPIO_WritePin( _pChipSelect->pGPIO, _pChipSelect->pin, (GPIO_PinState)_state);

	return Ok;
	}

tdStatus SPI_DRIVER_initDevice(tdSpiDevice *_pDevice, SPI_HandleTypeDef *_pSpiHandle, tdSpiCS *_pCs)
	{
	uint8_t channel;

	if( _pDevice == NULL || _pSpiHandle == NULL )
		return Fail;

	channel = SPI_DRIVER_getSpiChannel( _pSpiHandle );

	if( (channel < SPI_DRIVER_CHANNELS) == 0 )
		return Fail;

	_pDevice->pSpi = _pSpiHandle;
	_pDevice->channel = channel;
	_pDevice->chipSelect.pGPIO = _pCs->pGPIO;
	_pDevice->chipSelect.pin = _pCs->pin;

	if( SPI_DRIVER_initCS( &_pDevice->chipSelect ) == Fail )
		return Fail;

	return Ok;
	}

tdStatus SPI_DRIVER_dataTransmition(tdSpiDevice *_pDevice, uint8_t *_pTxData, uint8_t *_pRxData, uint16_t _size)
	{
	tdStatus status;
	tdSpiDevice **pSpiDevice;
	#if SPI_DRIVER_TRANSMITION_TIMEOUT_ENABLE != 0
	uint32_t timeout;
	#endif

	if( _pDevice == NULL )
		return Fail;

	pSpiDevice = &spiDriver.pChannel[ _pDevice->channel ];

	if( *pSpiDevice != NULL )
		return Busy;

	*pSpiDevice = _pDevice;

	#if SPI_DRIVER_TRANSMITION_TIMEOUT_ENABLE != 0
  timeout = HAL_GetTick() + SPI_TRANSMITION_TIMEOUT_MS;
	#endif

	SPI_DRIVER_setCS( &(*pSpiDevice)->chipSelect, Down );

	HAL_SPI_TransmitReceive_DMA( (*pSpiDevice)->pSpi, _pTxData, _pRxData, _size );

	status = Ok;

	while( HAL_SPI_GetState( (*pSpiDevice)->pSpi ) != HAL_SPI_STATE_READY )
		{
		#if SPI_DRIVER_TRANSMITION_TIMEOUT_ENABLE != 0
		if( timeout > HAL_GetTick() )
			{
			status = TimeOut;
			break;
			}
		#endif
		}

	#if SPI_DRIVER_SPI_TxRx_COMPLETE_CALLBACK_ENABLE == 0
	SPI_DRIVER_setCS( &(*pSpiDevice)->chipSelect, Up );
	#endif

	*pSpiDevice = NULL;

	return status;
	};

#if SPI_DRIVER_SPI_TxRx_COMPLETE_CALLBACK_ENABLE != 0
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
	{
	uint8_t channel;
	tdSpiDevice **pDevice;

	channel = SPI_DRIVER_getSpiChannel( hspi );

	if( (channel < SPI_DRIVER_CHANNELS) == 0 )
		return;

	pDevice = &spiDriver.pChannel[ channel ];

	if( *pDevice == NULL )
		return;

	SPI_DRIVER_setCS( &(*pDevice)->chipSelect, Up );
	};
#endif
