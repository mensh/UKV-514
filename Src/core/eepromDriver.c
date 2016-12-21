#include "spiDriver.h"
#include "eepromDriver.h"
#include "stdlib.h"
#include "string.h"



tdStatus EEPROM_DRIVER_dataTransmition(tdSpiDevice *_pPevice, uint8_t *_pTxBuff, uint8_t *_pRxBuff, uint16_t _size)
	{
	return SPI_DRIVER_dataTransmition( _pPevice, _pTxBuff, _pRxBuff, _size);
	}

tdStatus EEPROM_DRIVER_write(tdSpiDevice *_pDevice, uint8_t *_pBuff, uint16_t _size)
	{
	tdStatus status;
	uint8_t writeEnableCommand[ EEPROM_DRIVER_FM25V05_DATA_OP_INSTRUCTION_SIZE ] = { (uint8_t)WriteEnable };
	uint8_t writeEnableReadBack[ EEPROM_DRIVER_FM25V05_DATA_OP_INSTRUCTION_SIZE ] = { 0x00 };

	#if EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE > 0
	uint8_t rxBuff[ EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE ];
	if( _size > EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE )
		return Fail;
	#else
	uint8_t *rxBuff = (uint8_t *)malloc( (unsigned int)_size );
	if( rxBuff == NULL )
		return Fail;
	#endif

	do
		{
		status = EEPROM_DRIVER_dataTransmition(_pDevice, writeEnableCommand, writeEnableReadBack, EEPROM_DRIVER_FM25V05_DATA_OP_INSTRUCTION_SIZE); // set write enable
		} while( status == Busy );	// Trying to transmit till ok

	if( status == Ok )
		{
		// write enable setted, so we can transmit data
		do
			{
			status = EEPROM_DRIVER_dataTransmition(_pDevice, _pBuff, rxBuff, _size); // write data
			} while( status == Busy );	// Trying to transmit till ok
		}

	#if EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE == 0
	free( (void *)rxBuff );
	#endif

	return status;
	}

tdStatus EEPROM_DRIVER_writeByte(tdSpiDevice *_pDevice, uint16_t _address, uint8_t _byte)
	{
	uint8_t txBuff[ EEPROM_DRIVER_FM25V05_DATA_OP_INSTRUCTION_SIZE + EEPROM_DRIVER_FM25V05_DATA_ADDRESS_SIZE + EEPROM_DRIVER_FM25V05_DATA_BYTE_SIZE ];

	txBuff[ 0 ] = (uint8_t)WriteData;
	txBuff[ 1 ] = (_address >> 8) & 0xFF;
	txBuff[ 2 ] = (_address >> 0) & 0xFF;
	txBuff[ 3 ] = _byte;

	return EEPROM_DRIVER_write(_pDevice, txBuff, EEPROM_DRIVER_FM25V05_DATA_OP_INSTRUCTION_SIZE + EEPROM_DRIVER_FM25V05_DATA_ADDRESS_SIZE + EEPROM_DRIVER_FM25V05_DATA_BYTE_SIZE );
	}

// EEPROM_DRIVER_writeBuffer slow implementation
// Overload
// _pBuffer copying to txBuff. So you have additional O(n), n = _size
tdStatus EEPROM_DRIVER_writeBuffer(tdSpiDevice *_pDevice, uint16_t _address, uint8_t *_pBuffer, uint16_t _size)
	{
	tdStatus status;
	uint8_t *pTxBuff;
	uint16_t bytesToCopy;
	uint16_t transmissionSize = EEPROM_DRIVER_FM25V05_DATA_OP_INSTRUCTION_SIZE + EEPROM_DRIVER_FM25V05_DATA_ADDRESS_SIZE + _size;

	#if EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE > 0
	uint8_t txBuff[ EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE ];
	if( transmissionSize > EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE )
		return Fail;
	#else
	uint8_t *txBuff = (uint8_t *)malloc( (unsigned int)transmissionSize );
	if( txBuff == NULL )
		return Fail;
	#endif

	txBuff[ 0 ] = (uint8_t)WriteData;
	txBuff[ 1 ] = (_address >> 8) & 0xFF;
	txBuff[ 2 ] = (_address >> 0) & 0xFF;
	pTxBuff = &txBuff[ 3 ];

	bytesToCopy = _size;
	while( bytesToCopy-- )
		{
		*pTxBuff++ = *_pBuffer++;
		}

	status = EEPROM_DRIVER_write(_pDevice, txBuff, transmissionSize);

	#if EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE == 0
	free( txBuff );
	#endif

	return status;
	}


// EEPROM_DRIVER_writeBuffer fast implementation
// Overload
// If you want to use this implementation you must fill 0, 1, 2 byts of _pBuffer yourself
// *(_pBuffer + 0) = (uint8_t)WriteData;
// *(_pBuffer + 1) = Addres_MSB;
// *(_pBuffer + 2) = Addres_LSB;
// Note that _size = 3 + sizeof(yourData)
tdStatus EEPROM_DRIVER_writeBuffer_(tdSpiDevice *_pDevice, uint8_t *_pBuffer, uint16_t _size)
	{
	return EEPROM_DRIVER_write(_pDevice, _pBuffer, _size);
	}

tdStatus EEPROM_DRIVER_read(tdSpiDevice *_pDevice, uint16_t _address, uint8_t *_pBuff, uint16_t _size)
	{
	tdStatus status;
	uint16_t transmissionSize = EEPROM_DRIVER_FM25V05_DATA_OP_INSTRUCTION_SIZE + EEPROM_DRIVER_FM25V05_DATA_ADDRESS_SIZE + _size;

	#if EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE > 0
	uint8_t txBuff[ EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE ];
	if( transmissionSize > EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE )
		return Fail;
	#else
	uint8_t *txBuff = (uint8_t *)malloc( transmissionSize );
	if( txBuff == NULL )
		return Fail;
	#endif

	memset(txBuff, 0x00, transmissionSize);

	txBuff[ 0 ] = (uint8_t)ReadData;
	txBuff[ 1 ] = (_address >> 8) & 0xFF;
	txBuff[ 2 ] = (_address >> 0) & 0xFF;

	do
		{
		status = EEPROM_DRIVER_dataTransmition(_pDevice, txBuff, _pBuff,  transmissionSize); // read data
		} while( status == Busy );	// Trying to read till ok

	#if EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE == 0
	free( txBuff );
	#endif

	return status;
	}

tdStatus EEPROM_DRIVER_readByte(tdSpiDevice *_pDevice, uint16_t _address, uint8_t *_byte)
	{
	tdStatus status;
	uint8_t rxData[ EEPROM_DRIVER_FM25V05_DATA_OP_INSTRUCTION_SIZE + EEPROM_DRIVER_FM25V05_DATA_ADDRESS_SIZE + EEPROM_DRIVER_FM25V05_DATA_BYTE_SIZE ];

	status = EEPROM_DRIVER_read(_pDevice, _address, rxData, EEPROM_DRIVER_FM25V05_DATA_BYTE_SIZE );

	if( status == Ok )
		{
		*_byte = rxData[ 3 ]; // Required information starts at *(rxBuff + 3)
		}
	else
		{
		*_byte = 0;
		}

	return status;
	}

// EEPROM_DRIVER_readBuffer slow reedBuffer implementation
// rxBuff copying to _pBuff. So you have additional O(n), n = _size
tdStatus EEPROM_DRIVER_readBuffer(tdSpiDevice *_pDevice, uint16_t _address, uint8_t *_pBuff, uint16_t _size)
	{
	uint8_t *pRxBuffer;
	uint16_t bytesToCopy;
	tdStatus status;
	uint16_t transmissionSize = EEPROM_DRIVER_FM25V05_DATA_OP_INSTRUCTION_SIZE + EEPROM_DRIVER_FM25V05_DATA_ADDRESS_SIZE + _size;

	#if EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE > 0
	uint8_t rxBuff[ EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE ];
	if( transmissionSize > EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE )
		return Fail;
	#else
	uint8_t *rxBuff = (uint8_t *)malloc( transmissionSize );
	if( rxBuff == NULL )
		return Fail;
	#endif

	status = EEPROM_DRIVER_read(_pDevice, _address, rxBuff, transmissionSize);

	if( status == Ok )
		{
		pRxBuffer = &rxBuff[ 3 ]; // Required information starts at *(rxBuff + 3)

		bytesToCopy = _size;
		while( bytesToCopy-- )
			{
			*_pBuff++ = *pRxBuffer++;
			}
		}

	#if EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE == 0
	free( rxBuff );
	#endif

	return status;
	}

// EEPROM_DRIVER_readBuffer_first3BytesDummy fast reedBuffer implementation
// Note:
// *(_pBuff + 0), *(_pBuff + 1), *(_pBuff + 2) - trash bytes. Required information starts at *(_pBuff + 3)
// _size = 3 + sizeof(dataYouWantToGet)
tdStatus EEPROM_DRIVER_readBuffer_first3BytesDummy(tdSpiDevice *_pDevice, uint16_t _address, uint8_t *_pBuff, uint16_t _size)
	{
	return EEPROM_DRIVER_read(_pDevice, _address, _pBuff, _size);
	}

tdStatus EEPROM_DRIVER_eraseRange(tdSpiDevice *_pDevice, uint16_t _addressFrom, uint16_t _addressTo)
	{
	if( _addressFrom > _addressTo )
		return Fail;

	tdStatus status;
	uint16_t size = _addressTo - _addressFrom;
	uint16_t transmissionSize = EEPROM_DRIVER_FM25V05_DATA_OP_INSTRUCTION_SIZE + EEPROM_DRIVER_FM25V05_DATA_ADDRESS_SIZE + size;

	#if EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE > 0
	uint8_t txBuff[ EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE ];
	if( transmissionSize > EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE )
		return Fail;
	#else
	uint8_t *txBuff = (uint8_t *)malloc( transmissionSize );
	if( txBuff == NULL )
		return Fail;
	#endif

	memset(txBuff, 0x00, transmissionSize);

	txBuff[ 0 ] = (uint8_t)WriteData;
	txBuff[ 1 ] = (_addressFrom >> 8) & 0xFF;
	txBuff[ 2 ] = (_addressFrom >> 0) & 0xFF;

	status = EEPROM_DRIVER_writeBuffer_(_pDevice, txBuff, transmissionSize);

	#if EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE == 0
	free( txBuff );
	#endif

	return status;
	}

tdStatus EEPROM_DRIVER_format(tdSpiDevice *_pDevice)
	{
	// For more effectiveness make this function like EEPROM_DRIVER_eraseRange
	// Allocate memory only once and then use it till you need
	// In current realization allocation and free used ((EEPROM_DRIVER_FM25V05_SIZE / SIZE_TO_ERASE) - 1) dummy times
	#if EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE > 0
		#define SIZE_TO_ERASE (EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE - (EEPROM_DRIVER_FM25V05_DATA_OP_INSTRUCTION_SIZE + EEPROM_DRIVER_FM25V05_DATA_ADDRESS_SIZE) )
	#else
		#define SIZE_TO_ERASE 1024 /* 1 kb */
	#endif
	int32_t addressFrom, addressTo;

	for( addressFrom = 0x0000, addressTo = SIZE_TO_ERASE; addressFrom <= EEPROM_DRIVER_FM25V05_SIZE - SIZE_TO_ERASE; addressFrom += SIZE_TO_ERASE, addressTo += SIZE_TO_ERASE )
		{
		if( (addressTo < EEPROM_DRIVER_FM25V05_SIZE - 1) == 0 )
			addressTo = EEPROM_DRIVER_FM25V05_SIZE - 1;

		if( EEPROM_DRIVER_eraseRange(_pDevice, addressFrom, addressTo) != Ok )
			return Fail;
		}

	return Ok;
	#undef SIZE_TO_ERASE
	}

