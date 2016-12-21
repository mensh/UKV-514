#include "holtDriver.h"
#include <string.h>
#include "bhd2settings.h"

tdStatus HOLT_DRIVER_dataTransmition(tdSpiDevice *_pDevice, uint8_t *_pTxData, uint8_t *_pRxData, uint16_t _size)
	{
	return SPI_DRIVER_dataTransmition( _pDevice, _pTxData, _pRxData, _size);
	}

tdStatus HOLT_DRIVER_setConfiguration(tdSpiDevice *_pDevice, uint8_t _channel, uint16_t _configuration)
	{
	static uint8_t txBuff[ HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_CONFIGURATION_SIZE ];
	static uint8_t rxBuff[ HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_CONFIGURATION_SIZE ];

	txBuff[ 0 ] = (uint8_t)setConfiguration;
	txBuff[ 0 ] |= ( (_channel & HOLT_DRIVER_HI3598_DATA_CHANNEL_MASK) << HOLT_DRIVER_HI3598_DATA_CHANNEL_SHIFT );
	txBuff[ 1 ] = (_configuration >> 8) & HOLT_DRIVER_HI3598_DATA_BYTE_MASK;
	txBuff[ 2 ] = (_configuration >> 0) & HOLT_DRIVER_HI3598_DATA_BYTE_MASK;

	return HOLT_DRIVER_dataTransmition(_pDevice, txBuff, rxBuff, HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_CONFIGURATION_SIZE);
	}

tdStatus HOLT_DRIVER_readConfiguration(tdSpiDevice *_pDevice, uint8_t _channel, uint16_t *_pConfiguration)
	{
	tdStatus status;
	static uint8_t txBuff[ HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_CONFIGURATION_SIZE ];
	static uint8_t rxBuff[ HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_CONFIGURATION_SIZE ];

	txBuff[ 0 ] = (uint8_t)readConfiguration;
	txBuff[ 0 ] |= ( (_channel & HOLT_DRIVER_HI3598_DATA_CHANNEL_MASK) << HOLT_DRIVER_HI3598_DATA_CHANNEL_SHIFT );
	txBuff[ 1 ] = 0x00;
	txBuff[ 2 ] = 0x00;

	status = HOLT_DRIVER_dataTransmition(_pDevice, txBuff, rxBuff, HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_CONFIGURATION_SIZE);

	if( status == Ok )
		{
		*_pConfiguration = ( (rxBuff[ 1 ] << 8) | rxBuff[ 2 ] );
		}
	else
		{
		*_pConfiguration = 0x0000;
		}

	return status;
	}

tdStatus HOLT_DRIVER_readFifoWord(tdSpiDevice *_pDevice, uint8_t _channel, uint32_t *_word)
	{
	tdStatus status;
	uint8_t txBuff[ HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_WORD_SIZE ];
	uint8_t rxBuff[ HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_WORD_SIZE ];

	memset(txBuff, 0x00, HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_WORD_SIZE);

	txBuff[ 0 ] = (uint8_t)readRxFifo;
	txBuff[ 0 ] |= ( (_channel & HOLT_DRIVER_HI3598_DATA_CHANNEL_MASK) << HOLT_DRIVER_HI3598_DATA_CHANNEL_SHIFT );

	status = HOLT_DRIVER_dataTransmition(_pDevice, txBuff, rxBuff, HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_WORD_SIZE);

	if( status == Ok )
		{
		*_word = ( (rxBuff[ 1 ] << 24) | (rxBuff[ 2 ] << 16) | (rxBuff[ 3 ] << 8) | (rxBuff[ 4 ] << 0) );
		}
	else
		{
		*_word = 0x00000000;
		}

	return status;
	}

tdStatus HOLT_DRIVER_readFifoData(tdSpiDevice *_pDevice, uint8_t _channel, uint8_t *_data)
	{
	tdStatus status;
	uint8_t txBuff[ HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_WORD_SIZE ];

	memset(txBuff, 0x00, HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_WORD_SIZE);

	txBuff[ 0 ] = (uint8_t)readRxFifo;
	txBuff[ 0 ] |= ( (_channel & HOLT_DRIVER_HI3598_DATA_CHANNEL_MASK) << HOLT_DRIVER_HI3598_DATA_CHANNEL_SHIFT );

	status = HOLT_DRIVER_dataTransmition(_pDevice, txBuff, _data, HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_WORD_SIZE);

	if( status == Ok )
		*_data = _channel - 1; // Saving channel at firs byte after we get response(to prevent information lost)

	return status;
	}

tdStatus HOLT_DRIVER_readStatus(tdSpiDevice *_pDevice, tdHoltStatus *_pStatus)
	{
	tdStatus status;

	uint8_t txBuff[ HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_STATUS_SIZE ];
	uint8_t rxBuff[ HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_STATUS_SIZE ];

	txBuff[ 0 ] = (uint8_t)readStatus;
	txBuff[ 1 ] = 0x00;
	txBuff[ 2 ] = 0x00;

	rxBuff[ 1 ] = 0x00;
	rxBuff[ 2 ] = 0x00;

	status = HOLT_DRIVER_dataTransmition(_pDevice, txBuff, rxBuff, HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_STATUS_SIZE);

	if( status == Ok )
		{
		_pStatus->notEmpty = ~rxBuff[ 2 ];	// HI3598 empty status. There is "0" in corresponding bit when some FIFO buffer not empty. So we do inversion.
		_pStatus->full = rxBuff[ 1 ];			// HI3598 full status. There is "1" in corresponding bit when some FIFO buffer full.
		}
	else
		{
		_pStatus->notEmpty = HOLT_DRIVER_FIFO_STATUS_EMPTY;
		_pStatus->full = HOLT_DRIVER_FIFO_STATUS_NOT_FULL;
		}

	return status;
	}

tdStatus HOLT_DRIVER_masterReset(tdSpiDevice *_pDevice)
	{
	uint8_t txBuff[ HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE ];
	uint8_t rxBuff[ HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE ];

	txBuff[ 0 ] = (uint8_t)reset;

	return HOLT_DRIVER_dataTransmition(_pDevice, txBuff, rxBuff, HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_STATUS_SIZE);
	}

uint32_t HOLT_DRIVER_reverseBitOrder(uint32_t _x)
	{
	_x = (((_x & 0xAAAAAAAA) >> 1) | ((_x & 0x55555555) << 1));
	_x = (((_x & 0xCCCCCCCC) >> 2) | ((_x & 0x33333333) << 2));
	_x = (((_x & 0xF0F0F0F0) >> 4) | ((_x & 0x0F0F0F0F) << 4));
	_x = (((_x & 0xFF00FF00) >> 8) | ((_x & 0x00FF00FF) << 8));
	return ((_x >> 16) | (_x << 16));
	}

tdStatus HOLT_DRIVER_writeLoopBackFifoWord(tdSpiDevice *_pDevice, uint8_t _speedHigh, uint32_t _word)
	{
	uint8_t txBuff[ HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_WORD_SIZE ];
	uint8_t rxBuff[ HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_WORD_SIZE ];

	memset(txBuff, 0x00, HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_WORD_SIZE);

	if( _speedHigh == 0 )
		txBuff[ 0 ] = (uint8_t)loopBackTxSpeedLow;
	else
		txBuff[ 0 ] = (uint8_t)loopBackTxSpeedHigh;

	/*	NOTE: The first bit shifted into the Self Test register will
			be the first bit sent to the receivers and the TX1 and TX0
			pins. In ARINC 429 protocol, this bit is the LSB.
			hi-3598_v-rev-e.pdf, page 9
			Also, as you know first bit in SPI is MSB so we must reverse bits */
	_word = HOLT_DRIVER_reverseBitOrder(_word);

	txBuff[ 1 ] = (_word >> 24) & HOLT_DRIVER_HI3598_DATA_BYTE_MASK;
	txBuff[ 2 ] = (_word >> 16) & HOLT_DRIVER_HI3598_DATA_BYTE_MASK;
	txBuff[ 3 ] = (_word >>  8) & HOLT_DRIVER_HI3598_DATA_BYTE_MASK;
	txBuff[ 4 ] = (_word >>  0) & HOLT_DRIVER_HI3598_DATA_BYTE_MASK;

	return HOLT_DRIVER_dataTransmition(_pDevice, txBuff, rxBuff, HOLT_DRIVER_HI3598_DATA_OP_INSTRUCTION_SIZE + HOLT_DRIVER_HI3598_DATA_WORD_SIZE);
	}
