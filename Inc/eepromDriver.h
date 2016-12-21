#ifndef __HOLT_DRIVER_CORE_H_
#define __HOLT_DRIVER_CORE_H_



/*===============================================================================================*/
/*===============================================================================================*/
// Comments



/*===============================================================================================*/
/*===============================================================================================*/
// Includes
#include "tdStatus.h"
#include "spiDriver.h"
#include "bhd2settings.h"


/*===============================================================================================*/
/*===============================================================================================*/
// Defines
#define EEPROM_DRIVER_CONSTANT_TRANSFER_BUFFER_SIZE 0

#define EEPROM_DRIVER_3_DUMMY_BYTES 3
#define EEPROM_DRIVER_3_COMMAND_BYTES 3

#define EEPROM_DRIVER_FM25V05_SIZE 0x100000
#define EEPROM_DRIVER_FM25V05_DATA_OP_INSTRUCTION_SIZE 1
#define EEPROM_DRIVER_FM25V05_DATA_ADDRESS_SIZE 2
#define EEPROM_DRIVER_FM25V05_DATA_BYTE_SIZE 1

//#define EEPROM_SPI_1

/*===============================================================================================*/
/*===============================================================================================*/
// Data types, structures, ect.

typedef enum
	{
	WriteEnable		= 0x06,	//	WREN
	WriteDisable	= 0x04,	//	WRDI
	ReadStatus		= 0x05,	//	RDSR
	WriteStatus		= 0x01,	//	WRSR
	ReadData			= 0x03,	//	READ
	ReadDataFast	= 0x0B,	//	FSTRD
	WriteData			= 0x02,	//	WRITE
	SleepMode			= 0xB9,	//	SLEEP
	ReedDeviceId	= 0x9F	//	RDID
	} tdEepromOpCodes;



/*===============================================================================================*/
/*===============================================================================================*/

tdStatus EEPROM_DRIVER_dataTransmition(tdSpiDevice *_pDevice, uint8_t *_pTxBuff, uint8_t *_pRxBuff, uint16_t _size);

tdStatus EEPROM_DRIVER_write(tdSpiDevice *_pDevice, uint8_t *_pBuff, uint16_t _size);

tdStatus EEPROM_DRIVER_writeByte(tdSpiDevice *_pDevice, uint16_t _address, uint8_t _byte);

// EEPROM_DRIVER_writeBuffer slow implementation
// Overload
// _pBuffer copying to txBuff. So you have additional O(n), n = _size
tdStatus EEPROM_DRIVER_writeBuffer(tdSpiDevice *_pDevice, uint16_t _address, uint8_t *_pBuffer, uint16_t _size);

// EEPROM_DRIVER_writeBuffer fast implementation
// Overload
// If you want to use this implementation you must fill 0, 1, 2 byts of _pBuffer yourself
// *(_pBuffer + 0) = (uint8_t)WriteData;
// *(_pBuffer + 1) = Addres_MSB;
// *(_pBuffer + 2) = Addres_LSB;
// Note that _size = 3 + sizeof(yourData)
tdStatus EEPROM_DRIVER_writeBuffer_(tdSpiDevice *_pDevice, uint8_t *_pBuffer, uint16_t _size);

tdStatus EEPROM_DRIVER_read(tdSpiDevice *_pDevice, uint16_t _address, uint8_t *_pBuff, uint16_t _size);

tdStatus EEPROM_DRIVER_readByte(tdSpiDevice *_pDevice, uint16_t _address, uint8_t *_byte);

// EEPROM_DRIVER_readBuffer slow reedBuffer implementation
// rxBuff copying to _pBuff. So you have additional O(n), n = _size
tdStatus EEPROM_DRIVER_readBuffer(tdSpiDevice *_pDevice, uint16_t _address, uint8_t *_pBuff, uint16_t _size);

// EEPROM_DRIVER_readBuffer_first3BytesDummy fast reedBuffer implementation
// Note:
// *(_pBuff + 0), *(_pBuff + 1), *(_pBuff + 2) - trash bytes. Required information starts at *(_pBuff + 3)
// _size = 3 + sizeof(dataYouWantToGet)
tdStatus EEPROM_DRIVER_readBuffer_first3BytesDummy(tdSpiDevice *_pDevice, uint16_t _address, uint8_t *_pBuff, uint16_t _size);

tdStatus EEPROM_DRIVER_eraseRange(tdSpiDevice *_pDevice, uint16_t _addressFrom, uint16_t _addressTo);

tdStatus EEPROM_DRIVER_format(tdSpiDevice *_pDevice);



/*===============================================================================================*/
/*===============================================================================================*/



#endif
