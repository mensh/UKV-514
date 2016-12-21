#include "holtCore.h"
#include <string.h>
#include "manipulationsWithNumbers.h"
#include "fileSystem.h"
#include "timCore.h"

tdSpiDevice HOLT3598;

void HOLT_CORE_init(tdSpiDevice *_pDevice, tdArinc *_pArinc)
	{
	uint8_t channel;
	uint16_t configuration;
	tdArincChannel *pChannel;
	tdSpiCS hot3598ChipSelect;
	SPI_HandleTypeDef *pHspi;

#ifdef USB_HARDWARE_FS
	pHspi = &hspi1;
	hot3598ChipSelect.pGPIO = GPIOA;
	hot3598ChipSelect.pin = GPIO_PIN_4;
#else
	pHspi = &hspi3;
	hot3598ChipSelect.pGPIO = GPIOC;
	hot3598ChipSelect.pin = GPIO_PIN_13;
#endif

	_pArinc->activeChannelsAmount = 0;

	SPI_DRIVER_initDevice(_pDevice, pHspi, &hot3598ChipSelect);

#ifdef BHD2_ARINC_CORE_HOLT_ENABLE_SELFTEST
	HOLT_CORE_holtSelfTest(_pDevice, &_pArinc->receiverFailedFreq);
#endif

	HOLT_CORE_generateReversedLabels(_pArinc);

	FILE_SYSTEM_readHoltConfiguration(_pArinc);

	channel = 0;
	pChannel = &_pArinc->channel[ 0 ];
	while( channel < HOLT_CORE_CHANNELS_AMOUNT )
		{
		pChannel->status.active = 0;
		pChannel->status.activeTimeEnd = 0;
		pChannel->status.state = notActiveState;
		if( pChannel->status.enable == 1 )
			{
			configuration = CORE_HOLT3598_CONFIGURATION;

			if( pChannel->freq == frequency_100_kHz )
				configuration |= HOLT_DRIVER_HI3598_CONFIGURATION_SPEED_HIGH;
			else if( pChannel->freq == frequency_12_5_kHz )
				configuration |= HOLT_DRIVER_HI3598_CONFIGURATION_SPEED_LOW;

			HOLT_CORE_configurateChannel(_pDevice, channel + 1, configuration);
			}

		pChannel++;
		channel++;
		}

	HOLT_DRIVER_masterReset( _pDevice );
	}

tdArincConfigurationStatus HOLT_CORE_handleSlotTD(tdArinc *_pArinc, uint8_t *_pData, uint8_t _slotsAmount)
	{
	tdArincChannel *pChannel;
	uint8_t channelIndex;
	uint8_t slot;
	uint8_t freq;
	tdArincConfigurationStatus status;

	if( _slotsAmount > HOLT_CORE_CHANNELS_AMOUNT )
		return configAmountError;

	pChannel = &_pArinc->channel[ 0 ];
	for( channelIndex = 0; channelIndex < HOLT_CORE_CHANNELS_AMOUNT; channelIndex++ )
		{
		pChannel->status.enable = 0;
		pChannel->freq = frequency_notSet;
		pChannel++;
		}

	status = configOk;
	for( slot = 0; slot < _slotsAmount; slot++ )
		{
		channelIndex = *(_pData + HOLT_CORE_USB_SLOT_TD_ICHN_SHIFT);
		freq = *(_pData + HOLT_CORE_USB_SLOT_TD_FREQ_SHIFT);

		if( channelIndex > HOLT_CORE_CHANNELS_AMOUNT - 1 )
			{
			status = configIndexError;
			break;
			}

		if( freq != frequency_100_kHz && freq != frequency_12_5_kHz )
			{
			status = configFreqError;
			break;
			}

		pChannel = &_pArinc->channel[ channelIndex ];

		if( pChannel->status.enable == 1 ) // we trying to configurate channel once more
			{
			status = configDuplicateChannels;
			break;
			}

		pChannel->status.enable = 1;
		pChannel->freq = (tdArincFreq)freq;

		_pData += HOLT_CORE_USB_SLOT_TD_SIZE;
		}

	if( status == configOk /*|| status == configDuplicateChannels*/ )
		{
		_pArinc->enabledChannels = 0;
		pChannel = &_pArinc->channel[ 0 ];
		for( channelIndex = 0; channelIndex < HOLT_CORE_CHANNELS_AMOUNT; channelIndex++ )
			{
			if( pChannel->status.enable == 1 )
				_pArinc->enabledChannels++;
			pChannel++;
			}

		FILE_SYSTEM_saveHoltConfiguration(_pArinc);
		}
	else
		{
		FILE_SYSTEM_readHoltConfiguration(_pArinc);
		}

	return status;
	}

tdStatus HOLT_CORE_configurateChannel(tdSpiDevice *_pDevice, uint8_t _channel, uint16_t _configuration)
	{
	uint16_t readBackConfiguration = 0x0000;

	while( HOLT_DRIVER_setConfiguration(_pDevice, _channel, _configuration) == Busy ); // Trying to set configuration until status not equal Busy (Ok - if success or TimeOut ). If Busy - try again;

	while( HOLT_DRIVER_readConfiguration(_pDevice, _channel, &readBackConfiguration) == Busy ); // Trying to read configuration until status not equal Busy (Ok - if success or TimeOut ). If Busy - try again;

	if( _configuration != readBackConfiguration )
		{
		return Fail;
		}

	return Ok;
	}

tdStatus HOLT_CORE_readWordsFormChannel(tdSpiDevice *_pDevice, tdArinc *_pArinc, uint8_t _channel, uint8_t _wordsToRead, uint8_t *_pBuffer, uint32_t *_pCurrentSize, uint32_t _maxSize)
	{
	tdArincChannel *pArincChannel;
	uint8_t channel;
	uint32_t word;
	uint8_t lable;

	channel = _channel - 1;
	pArincChannel = &_pArinc->channel[ channel ];

	if( pArincChannel->status.enable == 0  )
		return Ok;

	while( _wordsToRead )
		{
		if( HOLT_DRIVER_readFifoWord(_pDevice, _channel, &word) == Ok )
			{
			pArincChannel->okTimeEnd = HAL_GetTick() + BHD2_ARINC_CHANNEL_OK_TIMEOUT_MS;

			lable = _pArinc->reversedLabels[ word & (HOLT_CORE_LABELS_AMOUNT - 1) ]; // (word & 0xFF) - reversed lable
			pArincChannel->data[ lable ] = word;

			if( pArincChannel->status.active == 1 )
				{
				*(_pBuffer + *_pCurrentSize + 0) = channel;
				 MWN_M_setValue32ToBuffer(word, _pBuffer, *_pCurrentSize + 1);

				*_pCurrentSize += HOLT_CORE_READ_BYTES_AMOUNT;

				if( !(*_pCurrentSize < _maxSize) )
					return ProcessBreak;
				}

			_wordsToRead--;
			}
		}

	return Ok;
	}

tdStatus HOLT_CORE_handleFifoEvents(tdSpiDevice *_pDevice, tdArinc *_pArinc, uint8_t _events, uint8_t _wordsToRead, uint8_t *_pBuffer, uint32_t *_pCurrentSize, uint32_t _maxSize)
	{
	uint8_t channel;

	if( !(*_pCurrentSize < _maxSize) )
		return ProcessBreak;

	channel = 1;
	while( _events )
		{
		if( _events & 0x01 )
			{
			HOLT_CORE_readWordsFormChannel(_pDevice, _pArinc, channel, _wordsToRead, _pBuffer, _pCurrentSize, _maxSize);

			if( !(*_pCurrentSize < _maxSize) )
				return ProcessBreak;
			}
		_events >>= 1;
		channel++;
		}
	return Ok;
	}

#ifdef BHD2_ARINC_CORE_HOLT_GET_STATUS_VIA_SPI
tdStatus HOLT_CORE_ReedAvailableData(tdSpiDevice *_pDevice, tdArinc *_pArinc, uint8_t *_pBuffer, uint32_t *_pCurrentSize, uint32_t _maxSize)
	{
	tdStatus status = Ok;
	tdHoltStatus holtStatus;
	uint8_t tryes;

	if( !(*_pCurrentSize < _maxSize) )
		return ProcessBreak;

	tryes = 0;
	HOLT_DRIVER_readStatus(_pDevice, &holtStatus);
	while( holtStatus.notEmpty != 0 && tryes++ < 4 )
		{
		status = HOLT_CORE_handleFifoEvents(_pDevice, _pArinc, holtStatus.notEmpty, HOLT_DRIVER_HI3598_DATA_RECEIVER_FIFO_ONE_WORD, _pBuffer, _pCurrentSize, _maxSize);
		HOLT_DRIVER_readStatus(_pDevice, &holtStatus);
		}

	return status;
	}

#else
tdStatus HOLT_CORE_ReedAvailableData(tdSpiDevice *_pDevice, tdArinc *_pArinc, uint8_t *_pBuffer, uint32_t *_pCurrentSize, uint32_t _maxSize)
	{
	tdStatus status = Ok;
	uint8_t flagsStatus;
	uint8_t tryes;

	if( !(*_pCurrentSize < _maxSize) )
		return ProcessBreak;

	tryes = 0;
	flagsStatus = HOLT_DRIVER_CHANNELS_READY;
	while( flagsStatus != 0 && tryes++ < 4 )
		{
		status = HOLT_CORE_handleFifoEvents(_pDevice, _pArinc, flagsStatus, HOLT_DRIVER_HI3598_DATA_RECEIVER_FIFO_ONE_WORD, _pBuffer, _pCurrentSize, _maxSize);
		flagsStatus = HOLT_DRIVER_CHANNELS_READY;
		}

	return status;
	}

#endif

tdStatus HOLT_CORE_generateReversedLabels(tdArinc *_pArinc)
	{
	uint16_t lable;
	uint32_t S, T;
	for( lable = 0; lable < HOLT_CORE_LABELS_AMOUNT; lable++ )
		{
		S = (lable * 0x02020202) & 0x84422010;
		T = (lable * 8) & 0x00000420;
		_pArinc->reversedLabels[ (((S + T) % 1023) & (HOLT_CORE_LABELS_AMOUNT - 1)) ] = (uint8_t)lable;
		}

	return Ok;
	}

#define TMP_SPEED_AMOUNT 2
#define TMP_SPEED_LOW 0
#define TMP_SPEED_HIGH 1
#define TMP_TEST_DATA 0x9DB6692A
tdStatus HOLT_CORE_holtSelfTest(tdSpiDevice *_pDevice, tdArincReceiverFaild *_pFailStats)
	{
	tdStatus status;
	uint8_t channel;
	uint8_t test;
	uint32_t timeOut;
	uint8_t testSpeedHigh;
	tdHoltStatus holtStatus;
	uint16_t currentConfiguration[ HOLT_CORE_CHANNELS_AMOUNT ];
	uint32_t loopBackData;
	uint8_t failureStatsus[ TMP_SPEED_AMOUNT ] = { 0x00, 0x00 };
	uint16_t testCfg[ TMP_SPEED_AMOUNT ] = { (HOLT_DRIVER_HI3598_CONFIGURATION_SELF_TEST_ENABLE | HOLT_DRIVER_HI3598_CONFIGURATION_LABEL_ORDER_DIRECT | HOLT_DRIVER_HI3598_CONFIGURATION_SPEED_LOW),
																					 (HOLT_DRIVER_HI3598_CONFIGURATION_SELF_TEST_ENABLE | HOLT_DRIVER_HI3598_CONFIGURATION_LABEL_ORDER_DIRECT | HOLT_DRIVER_HI3598_CONFIGURATION_SPEED_HIGH) };

	if( _pDevice == NULL )
		return Fail;

	// Saving current configuration
	for( channel = 0; channel < HOLT_CORE_CHANNELS_AMOUNT; channel++ ) // Holt Configuration
		{
		while( HOLT_DRIVER_readConfiguration(_pDevice, channel + 1, &currentConfiguration[ channel ]) == Busy );
		}

	HOLT_DRIVER_masterReset( _pDevice );

	for( test = 0; test < TMP_SPEED_AMOUNT; test++ )
		{
		// test configuration
		for( channel = 0; channel < HOLT_CORE_CHANNELS_AMOUNT; channel++ )
			{
			do
				{
				status = HOLT_CORE_configurateChannel(_pDevice, channel + 1, testCfg[ test ]);
				} while( status == Busy );
			
			if( status == Fail )
				{
				failureStatsus[ test ] |= ( 1 << channel );
				}
			}

		// HOLT_DRIVER_HI3598_CONFIGURATION_SPEED_HIGH == (0 << 0), HOLT_DRIVER_HI3598_CONFIGURATION_SPEED_LOW == (1 << 0)
		testSpeedHigh = (testCfg[ test ] & HOLT_DRIVER_HI3598_CONFIGURATION_SPEED_LOW) ? 0 : 1;

		while( HOLT_DRIVER_writeLoopBackFifoWord(_pDevice, testSpeedHigh, TMP_TEST_DATA) == Busy );

		timeOut = HAL_GetTick() + 5; // 2 ms timeout

		holtStatus.notEmpty = 0;
		while( holtStatus.notEmpty == 0 && HAL_GetTick() < timeOut )
			{
			while( HOLT_DRIVER_readStatus(_pDevice, &holtStatus) == Busy );
			}

		for( channel = 0; channel < HOLT_CORE_CHANNELS_AMOUNT; channel++ )
			{
			if( holtStatus.notEmpty & ( 1 << channel ) )
				{
				while( HOLT_DRIVER_readFifoWord(_pDevice, channel + 1, &loopBackData) == Busy );

				if( loopBackData != TMP_TEST_DATA )
					{
					failureStatsus[ test ] |= ( 1 << channel );
					}
				}
			else
				{
				failureStatsus[ test ] |= ( 1 << channel );
				}
			}
		}

	// Returning previous configuration
	for( channel = 0; channel < HOLT_CORE_CHANNELS_AMOUNT; channel++ )
		{
		do
			{
			status = HOLT_CORE_configurateChannel(_pDevice, channel + 1, currentConfiguration[ channel ]);
			} while( status == Busy );

		if( status == Fail )			
			{
			failureStatsus[ TMP_SPEED_HIGH ] |= ( 1 << channel );
			failureStatsus[ TMP_SPEED_LOW ] |= ( 1 << channel );
			}
		}

	if( _pFailStats != NULL )
		{
		_pFailStats->high = failureStatsus[ TMP_SPEED_HIGH ];
		_pFailStats->low = failureStatsus[ TMP_SPEED_LOW ];
		}

 	return Ok;
	}
#undef TMP_SPEED_AMOUNT
#undef TMP_SPEED_HIGH
#undef TMP_SPEED_LOW
#undef TMP_TEST_DATA

