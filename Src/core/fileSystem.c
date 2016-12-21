#include "eepromDriver.h"
#include "fileSystem.h"
#include "stdlib.h"
#include "string.h"
#include "crc.h"
#include "bhd2settings.h"



extern int16_t ADC2_code;
extern int16_t ADC1_code;

uint8_t badBlocksList[ FILE_SYSTEM_MEMORY_BAD_BLOCK_LIST_SIZE ];

tdSpiDevice RAM_1;
tdSpiDevice RAM_2;
tdSpiDevice *pFileSystemDataRam;

uint8_t FILE_SYSTEM_isRamPresent(tdSpiDevice *_pRam)
	{
	uint8_t twentyThreeWrite = 23;
	uint8_t twentyThreeRead = 0;
	uint8_t fortyTwoWrite = 42;
	uint8_t fortyTwoRead = 0;

//	EEPROM_DRIVER_writeByte(_pRam, 0x0000, twentyThreeWrite);
//	EEPROM_DRIVER_writeByte(_pRam, 0xFFFF, fortyTwoWrite);

//	EEPROM_DRIVER_readByte(_pRam, 0x0000, &twentyThreeRead);
//	EEPROM_DRIVER_readByte(_pRam, 0xFFFF, &fortyTwoRead);

	if( twentyThreeWrite == twentyThreeRead || fortyTwoWrite == fortyTwoRead )
		return 1;

	return 0;
	}

tdStatus FILE_SYSTEM_setDataRam(void)
	{
	tdSpiDevice *ramList[ ] = {&RAM_1, &RAM_2};
	uint8_t ramPresent = 0;


	for( uint8_t ram = 0; ram < sizeof(ramList) / sizeof(ramList[ 0 ]); ram++ )
		{
		if( FILE_SYSTEM_isRamPresent( ramList[ ram ] ) == 1 )
			{
			pFileSystemDataRam = ramList[ ram ];
			ramPresent = 1;
			break;
			}
		}

	if( ramPresent == 1 )
		return Ok;
	else
		pFileSystemDataRam = ramList[ 0 ]; // no RAM's(((

	return Fail;
	}

tdStatus FILE_SYSTEM_satDefaultConfigurationTable(tdConfigurationTable *_pConfigTable)
	{
	_pConfigTable->SBD = 0;
	_pConfigTable->NTD = 1;
	_pConfigTable->size = 29 + _pConfigTable->SBD + _pConfigTable->NTD;

	MWN_M_setValue32ToBuffer((unsigned int)FILE_SYSTEM_MEMORY_SPARE_DATA_SIZE, _pConfigTable->data, FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_SSD );
	MWN_M_setValue32ToBuffer((unsigned int)FILE_SYSTEM_MEMORY_PAGE_DATA_SIZE,  _pConfigTable->data, FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_SMD );
	MWN_M_setValue32ToBuffer((unsigned int)FILE_SYSTEM_MEMORY_PAGES_AMOUNT,    _pConfigTable->data, FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_SB );
	MWN_M_setValue32ToBuffer((unsigned int)FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT,   _pConfigTable->data, FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_SM );
	MWN_M_setValue32ToBuffer((unsigned int)1,                                  _pConfigTable->data, FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_SD );
	MWN_M_setValue32ToBuffer((unsigned int)0,                                  _pConfigTable->data, FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_SERIAL_N );
	MWN_M_setValue32ToBuffer((unsigned int)_pConfigTable->SBD,                 _pConfigTable->data, FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_SBD );
	MWN_M_setValue8ToBuffer(_pConfigTable->NTD,                                _pConfigTable->data, FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_NTD );
	MWN_M_setValue8ToBuffer(0x10/*ARINC_DATA*/,                         			 _pConfigTable->data, 29 + _pConfigTable->SBD );

	return Ok;
	}

tdStatus FILE_SYSTEM_saveConfigurationTable(tdConfigurationTable *_pConfigTable)
	{
	uint8_t sbdBuff[ 4 ];

	MWN_M_setValue32ToBuffer((unsigned int)_pConfigTable->SBD, sbdBuff, 0 );

//	EEPROM_DRIVER_writeByte(pFileSystemDataRam, FILE_SYSTEM_ADDRESS_OF_CONFIGURATION_TABLE_EXIST, _pConfigTable->exist);
//	EEPROM_DRIVER_writeByte(pFileSystemDataRam, FILE_SYSTEM_ADDRESS_OF_CONFIGURATION_TABLE_NTD, _pConfigTable->NTD );
//	EEPROM_DRIVER_writeBuffer(pFileSystemDataRam, FILE_SYSTEM_ADDRESS_OF_CONFIGURATION_TABLE_SBD, sbdBuff, 4 );
//	EEPROM_DRIVER_writeBuffer(pFileSystemDataRam, FILE_SYSTEM_ADDRESS_OF_CONFIGURATION_TABLE, _pConfigTable->data, _pConfigTable->size );

	return Ok;
	}

tdStatus FILE_SYSTEM_readConfigurationTable(tdConfigurationTable *_pConfigTable)
	{
	uint8_t exist;
	uint8_t sbdBuff[ 4 ];
	uint8_t NTD;

	//EEPROM_DRIVER_readByte(pFileSystemDataRam, FILE_SYSTEM_ADDRESS_OF_CONFIGURATION_TABLE_EXIST, &exist);

	_pConfigTable->exist = exist;

	if( _pConfigTable->exist == 1 )
		{
		//if( EEPROM_DRIVER_readBuffer(pFileSystemDataRam, FILE_SYSTEM_ADDRESS_OF_CONFIGURATION_TABLE_SBD, sbdBuff, 4 ) != Ok )
		//	return Fail;

	//	if( EEPROM_DRIVER_readByte(pFileSystemDataRam, FILE_SYSTEM_ADDRESS_OF_CONFIGURATION_TABLE_NTD, &NTD ) != Ok )
	//		return Fail;

		_pConfigTable->SBD = MWN_M_getValue32FromBuffer(sbdBuff, 0);
		_pConfigTable->NTD = NTD;
		_pConfigTable->size = 29 + _pConfigTable->SBD + _pConfigTable->NTD;

		//if( EEPROM_DRIVER_readBuffer(pFileSystemDataRam, FILE_SYSTEM_ADDRESS_OF_CONFIGURATION_TABLE, _pConfigTable->data, _pConfigTable->size ) != Ok )
		//	return Fail;
		}
	else
		{
		FILE_SYSTEM_satDefaultConfigurationTable(_pConfigTable);
		}

	return Ok;
	}

tdStatus FILE_SYSTEM_saveHoltConfiguration(tdArinc *_pArinc)
	{
	tdArincChannel *pChannel;
	uint8_t channel;
	uint16_t address;

	channel = 0;
	pChannel = &_pArinc->channel[ 0 ];
	address = FILE_SYSTEM_ADDRESS_OF_DATA_TYPE_TABLE;
	while( channel < HOLT_CORE_CHANNELS_AMOUNT )
		{
		//EEPROM_DRIVER_writeByte(pFileSystemDataRam, address + 0, pChannel->status.enable);
		//EEPROM_DRIVER_writeByte(pFileSystemDataRam, address + 1, pChannel->freq);

		address += FILE_SYSTEM_DATA_TYPE_SIZE;
		pChannel++;
		channel++;
		}

	return Ok;
	}

tdStatus FILE_SYSTEM_readHoltConfiguration(tdArinc *_pArinc)
	{
	tdArincChannel *pChannel;
	uint16_t address;
	uint8_t channel;
	uint8_t enable;
	uint8_t freq;

	_pArinc->enabledChannels = 0;

	channel = 0;
	pChannel = &_pArinc->channel[ 0 ];
	address = FILE_SYSTEM_ADDRESS_OF_DATA_TYPE_TABLE;
	while( channel < HOLT_CORE_CHANNELS_AMOUNT )
		{
		//EEPROM_DRIVER_readByte(pFileSystemDataRam, address + 0, &enable);
		//EEPROM_DRIVER_readByte(pFileSystemDataRam, address + 1, &freq);

		pChannel->status.enable = enable;
		pChannel->freq = (tdArincFreq)freq;

		if( pChannel->status.enable == 1 )
			_pArinc->enabledChannels++;

		address += FILE_SYSTEM_DATA_TYPE_SIZE;
		pChannel++;
		channel++;
		}
	return Ok;
	}

//#define DEBUG_CLEAR_BLOCKS
tdStatus FILE_SYSTEM_init(tdFat *_pFat)
	{
	Init_Driver();

	FILE_SYSTEM_createFatTable( _pFat );

	return Ok;
	}

tdStatus FILE_SYSTEM_getPtrToBadBlocksList(uint8_t **_pBadBlockListPtr)
	{
	*_pBadBlockListPtr = badBlocksList;
	return Ok;
	}

tdStatus FILE_SYSTEM_clearBadBlocksList(void)
	{
	tdStatus status = Ok;

	//if( EEPROM_DRIVER_eraseRange(pFileSystemDataRam, FILE_SYSTEM_ADDRESS_OF_BAD_BLOCKS_LIST, FILE_SYSTEM_ADDRESS_OF_BAD_BLOCKS_LIST + FILE_SYSTEM_MEMORY_BAD_BLOCK_LIST_SIZE) != Ok )
	//	status = Fail;

	memset(badBlocksList, 0x00, (sizeof(badBlocksList) / sizeof(badBlocksList[0])) );

	return status;
	}

tdStatus FILE_SYSTEM_clearBadBlocksList_(uint32_t _from, uint32_t _size)
	{
	tdStatus status = Ok;
	uint32_t to;
	uint8_t *pByte;
	uint32_t block;

	if( (_from < FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT) == 0 )
		return Fail;

	block = _from;
	pByte = &badBlocksList[ block ];
	to = _from + _size;

	if( (to < FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT) == 0 )
		to = FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT;

	while( block < to )
		{
		if( *pByte != 0 )
			{
			*pByte = 0;

			//HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
			//EEPROM_DRIVER_writeByte(pFileSystemDataRam, FILE_SYSTEM_ADDRESS_OF_BAD_BLOCKS_LIST + block, 0x00);
			//HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
			}

		pByte++;
		block++;
		}

	return status;
	}

tdStatus FILE_SYSTEM_readBadBlocksList(void)
	{
//	if( EEPROM_DRIVER_readBuffer(pFileSystemDataRam, FILE_SYSTEM_ADDRESS_OF_BAD_BLOCKS_LIST, badBlocksList, FILE_SYSTEM_MEMORY_BAD_BLOCK_LIST_SIZE ) != Ok )
//		return Fail;

	return Ok;
	}

tdStatus FILE_SYSTEM_saveBadBlockStatusToMemmory(tdFileSystemBlock _block, uint8_t _status)
	{
	int byte, bit;
	uint8_t *pByte;
	if( (_block < FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT) == 0 )
		return Fail;

	byte = _block / MWN_M_BITS_IN_BYTE;
	bit = _block % MWN_M_BITS_IN_BYTE;

	pByte = &badBlocksList[ byte ];

	if( _status == 1 )
		*pByte |= (1 << bit);
	else
		*pByte &= ~(1 << bit);

//	if( EEPROM_DRIVER_writeByte(pFileSystemDataRam, FILE_SYSTEM_ADDRESS_OF_BAD_BLOCKS_LIST + byte, *pByte) != Ok )
//		return Fail;

	return Ok;
	}

tdStatus FILE_SYSTEM_setBadBlock(tdFileSystemBlock _block)
	{
	if( (_block < FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT) == 0 )
		return Fail;

	return FILE_SYSTEM_saveBadBlockStatusToMemmory( _block, 1 );
	}

tdStatus FILE_SYSTEM_clearBadBlock(tdFileSystemBlock _block)
	{
	if( (_block < FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT) == 0 )
		return Fail;

	return  FILE_SYSTEM_saveBadBlockStatusToMemmory( _block, 0 );
	}

uint8_t FILE_SYSTEM_isBadBlock(tdFileSystemBlock _block)
	{
	int byte, bit;
	uint8_t *pByte;
	if( (_block < FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT) == 0 )
		return 1;

	byte = _block / MWN_M_BITS_IN_BYTE;
	bit = _block % MWN_M_BITS_IN_BYTE;

	pByte = &badBlocksList[ byte ];

	return (*pByte & (1 << bit)) ? 1 : 0;
	}

uint8_t FILE_SYSTEM_isBadBlockAtAddress(tdFileSystemPageAddress _pageAddress)
	{
	return FILE_SYSTEM_isBadBlock( _pageAddress / FILE_SYSTEM_MEMORY_PAGES_AMOUNT );
	}

tdStatus FILE_SYSTEM_getSparePageData(const nand_addr_t *_pAddress, tdSparePageData *_pSpareData, uint8_t *_pSparePage)
	{
	if( FILE_SYSTEM_readSparePage(*_pAddress, _pSparePage, FILE_SYSTEM_MEMORY_SPARE_DATA_SIZE) == TimeOut )
		return Fail;

	_pSpareData->dataType = FS_M_getDataType( _pSparePage );
	_pSpareData->fileNum = FS_M_getFileNumber( _pSparePage );
	_pSpareData->crc8 = FS_M_getCRC8( _pSparePage );
	_pSpareData->timeStart = FS_M_getTimeBegin( _pSparePage );
	_pSpareData->timeEnd = FS_M_getTimeEnd( _pSparePage );

	return Ok;
	}

tdStatus FILE_SYSTEM_getLastPageAddressInBlock(nand_addr_t *_pAddress, uint8_t *_pSparePage)
	{
	tdSparePageData spareData;
	_pAddress->page = 0;

	while( _pAddress->page < FILE_SYSTEM_MEMORY_PAGES_AMOUNT )
		{
		FILE_SYSTEM_getSparePageData(_pAddress, &spareData, _pSparePage);

		if( spareData.dataType == 0xFF )
			{
			if( _pAddress->page > 1 )
				_pAddress->page -= 1; // take a prev page num;

			return Ok;
			}

		_pAddress->page++;
		}

	if( _pAddress->page > 1 )
		_pAddress->page -= 1; // take a prev page num;

	return Ok;
	}

// @Function: tdStatus FILE_SYSTEM_createFatTable(tdFat *_pFat)
// Read NAND Flash and create FAT Table based on spare information( fileNumber )
// NOTE:	When we save file to NAND Flash file number is growing. Each new file number = number + 1.
// 				But when download files to PC we enumerate files from last written, so first == lastWritten with highest fileNumber.
//				So _pFat->pFirstFile is the newest file and _pFat->pLastFile is the oldest.
tdStatus FILE_SYSTEM_createFatTable(tdFat *_pFat)
	{
	tdFileNode *pFile;
	uint8_t *pSparePage;
	tdFileMinMax fileMin;
	tdFileMinMax fileMax;
	nand_addr_t natCheckAddress;
	nand_addr_t natTmpAddress;
	tdSparePageData spareData;
	uint8_t startsFound;
	tdFileAddressAndTime timeStart;
	tdFileAddressAndTime timeEnd;

	if( _pFat == NULL )
		return Fail;

	pSparePage = (uint8_t *)malloc( FILE_SYSTEM_MEMORY_SPARE_DATA_SIZE );
	if( pSparePage == NULL )
		return Fail;

	memset(&natCheckAddress, 0x00, sizeof(natCheckAddress));
	memset(&natTmpAddress, 0x00, sizeof(natTmpAddress));

	_pFat->filesAmount = 0;
	_pFat->pFirstFile = (tdFileNode *)malloc( sizeof(tdFileNode) );
	_pFat->pFirstFile->pPreviousFile = NULL;
	_pFat->pFirstFile->pNextFile = NULL;

	pFile = _pFat->pFirstFile;

	fileMin.value = 0xFFFFFFFF;
	fileMin.pFile = NULL;

	fileMax.value = 0x00000000;
	fileMax.pFile = NULL;

	startsFound = 0;
	timeStart.time = 0xFFFFFFFF;
	timeEnd.time = 0x00000000;

	do
		{
		FILE_SYSTEM_getSparePageData(&natCheckAddress, &spareData, pSparePage);

		if( startsFound == 1 && (spareData.fileNum != pFile->fileNum || natCheckAddress.block == FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT - 1 || spareData.dataType == 0xFF ) )
			{
			memset(&natTmpAddress, 0x00, sizeof(natTmpAddress));
			natTmpAddress.block = natCheckAddress.block - 1;
			if( (natTmpAddress.block < FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT) == 0 )
				natTmpAddress.block = 0;

			FILE_SYSTEM_getLastPageAddressInBlock(&natTmpAddress, pSparePage);

			pFile->addressEnd = natTmpAddress.block * FILE_SYSTEM_MEMORY_PAGES_AMOUNT + natTmpAddress.page;

			// Searching up for minimum and maximum file numbers for knowing where is first and last(oldest) file;
			if( fileMin.value > pFile->fileNum )
				{
				fileMin.value = pFile->fileNum;
				fileMin.pFile = pFile; // fileNum of pFile is less then current. So pFile is a new min.
				}
			if( fileMax.value < pFile->fileNum )
				{
				fileMax.value = pFile->fileNum;
				fileMax.pFile = pFile; // fileNum of pFile is greater then current. So pFile is a new max.
				}

			pFile->pNextFile = (tdFileNode *)malloc( sizeof(tdFileNode) ); // creating next file
			pFile->pNextFile->pPreviousFile = pFile; //next file has a previous file. and.... it's current file;)

			pFile = pFile->pNextFile; // working with next file
			pFile->pNextFile = NULL;

			_pFat->filesAmount++;

			startsFound = 0;
			}

		if( FILE_SYSTEM_isBadBlock( natCheckAddress.block ) == 1 )
			continue;

		if( spareData.dataType != 0xFF && spareData.crc8 != 0 )
			{
			if( startsFound == 0 )
				{
				pFile->fileNum = spareData.fileNum;
				pFile->addressStart = natCheckAddress.block * FILE_SYSTEM_MEMORY_PAGES_AMOUNT;

				startsFound = 1;
				}

			if( spareData.timeStart < timeStart.time )
				{
				timeStart.time = spareData.timeStart;
				timeStart.address = natCheckAddress.block * FILE_SYSTEM_MEMORY_PAGES_AMOUNT;
				}

			if( spareData.timeEnd > timeEnd.time )
				{
				timeEnd.time = spareData.timeEnd;
				timeEnd.address = natCheckAddress.block * FILE_SYSTEM_MEMORY_PAGES_AMOUNT;
				}
			}
			#warning: "CHTOTO NETO";
		} while( ++natCheckAddress.block < /*FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT */ 2048 );

	if( _pFat->filesAmount == 0 )
		{
		free(_pFat->pFirstFile);
		_pFat->pFirstFile = NULL;
		}
	else
		{
		pFile = pFile->pPreviousFile;
		free( pFile->pNextFile );

		_pFat->pLastFile = pFile;
		_pFat->pLastFile->pNextFile = _pFat->pFirstFile;
		_pFat->pFirstFile->pPreviousFile = _pFat->pLastFile;
		}

	if( _pFat->filesAmount == 1 )
		{
		if( timeStart.address > timeEnd.address )
			{
			/*
				   timeEnd --+
			+--------------|-------------------+
			|              | |                 |  <--- One file on memory
			+----------------|-----------------+
			                 +-- timeStart
			*/
			_pFat->pFirstFile->addressStart = timeStart.address;

			FILE_SYSTEM_generateNandAddress(&natTmpAddress, timeEnd.address);
			FILE_SYSTEM_getLastPageAddressInBlock(&natTmpAddress, pSparePage);

			pFile->addressEnd = natTmpAddress.block * FILE_SYSTEM_MEMORY_PAGES_AMOUNT + natTmpAddress.page;
			}
		}
	else if( _pFat->filesAmount > 1 )
		{
		if( _pFat->pFirstFile != NULL && _pFat->pLastFile != NULL )
			{
			if( _pFat->pFirstFile != _pFat->pLastFile && _pFat->pFirstFile->fileNum == _pFat->pLastFile->fileNum )
				{
				_pFat->pFirstFile->addressStart = _pFat->pLastFile->addressStart;

				_pFat->pLastFile = _pFat->pLastFile->pPreviousFile;
				free( _pFat->pLastFile->pNextFile );

				_pFat->pLastFile->pNextFile = _pFat->pFirstFile;
				_pFat->pFirstFile->pPreviousFile = _pFat->pLastFile;
				}
			}

		_pFat->pFirstFile = fileMax.pFile;
		_pFat->pLastFile = fileMin.pFile;
		}

	free( pSparePage );

	return Ok;
	}

tdStatus FILE_SYSTEM_clearFatTable(tdFat *_pFat)
	{
	tdFileNode *pFile;

	if( _pFat == NULL )
		return Fail;

	if( _pFat->pFirstFile == NULL )
		return Ok;

	while( _pFat->pFirstFile->pNextFile != NULL )
		{
		pFile = _pFat->pFirstFile;
		_pFat->pFirstFile = _pFat->pFirstFile->pNextFile;
		free( pFile );
		}

	free( _pFat->pFirstFile );

	return Ok;
	}

tdStatus FILE_SYSTEM_newFile(tdFat *_pFat)
	{
	uint16_t attempt;

	if( _pFat == NULL )
		return Fail;

	if( _pFat->filesAmount == 0 )
		{
		_pFat->pFirstFile = malloc(sizeof(tdFileNode));
		_pFat->pFirstFile->pNextFile = NULL;
		_pFat->pFirstFile->pPreviousFile = NULL;
		_pFat->pFirstFile->fileNum = 1;
		_pFat->pFirstFile->addressStart = 0; // Life Hack;) In a few instructions, we call FILE_SYSTEM_getAddressForNewFile	and it generate new address - // 0 if 0 block not bad
		}
	else
		{
		_pFat->pFirstFile->pNextFile = malloc(sizeof(tdFileNode));
		_pFat->pFirstFile->pNextFile->pPreviousFile = _pFat->pFirstFile;
		_pFat->pFirstFile = _pFat->pFirstFile->pNextFile;

		_pFat->pFirstFile->pNextFile = NULL;
		_pFat->pFirstFile->fileNum = _pFat->pFirstFile->pPreviousFile->fileNum + 1;
		_pFat->pFirstFile->addressStart = _pFat->pFirstFile->pPreviousFile->addressEnd;

		FILE_SYSTEM_getNextBlockAddress(&_pFat->pFirstFile->addressStart);
		}

	attempt = 0;
	while( FILE_SYSTEM_isBadBlockAtAddress(_pFat->pFirstFile->addressStart) == 1 && attempt++ < FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT )
		{
		FILE_SYSTEM_getNextBlockAddress(&_pFat->pFirstFile->addressStart);
		}

	_pFat->pFirstFile->addressEnd = _pFat->pFirstFile->addressStart;
	_pFat->pFirstFile->filePageAddress = 0;

	_pFat->filesAmount++;

	return Ok;
	}

tdStatus FILE_SYSTEM_getNextBlockAddress(tdFileSystemPageAddress *_pPageAddress)
	{
	*_pPageAddress = ((*_pPageAddress / FILE_SYSTEM_MEMORY_PAGES_AMOUNT) + 1) * FILE_SYSTEM_MEMORY_PAGES_AMOUNT;

	if( ( *_pPageAddress < FILE_SYSTEM_MEMORY_MAX_ADDRESS ) == 0 )
		*_pPageAddress = 0; // make a ring

	return Ok;
	}

tdStatus FILE_SYSTEM_getPreviousBlockAddress(tdFileSystemPageAddress *_pPageAddress)
	{
	*_pPageAddress = ((*_pPageAddress / FILE_SYSTEM_MEMORY_PAGES_AMOUNT) - 1) * FILE_SYSTEM_MEMORY_PAGES_AMOUNT;

	if( ( *_pPageAddress < FILE_SYSTEM_MEMORY_MAX_ADDRESS ) == 0 )
		*_pPageAddress = ((FILE_SYSTEM_MEMORY_MAX_ADDRESS / FILE_SYSTEM_MEMORY_PAGES_AMOUNT) - 1) * FILE_SYSTEM_MEMORY_PAGES_AMOUNT;  // make a ring

	return Ok;
	}

tdStatus FILE_SYSTEM_generateNandAddress(nand_addr_t *_pAddress, tdFileSystemPageAddress _pageAddress)
	{
	if( ( _pageAddress < FILE_SYSTEM_MEMORY_MAX_ADDRESS ) == 0 )
		return Fail;

	_pAddress->lun = 0; // const
	_pAddress->block = _pageAddress / FILE_SYSTEM_MEMORY_PAGES_AMOUNT;
	_pAddress->page = _pageAddress % FILE_SYSTEM_MEMORY_PAGES_AMOUNT;
	_pAddress->column = 0; // data bytes per page

	return Ok;
	}

tdStatus FILE_SYSTEM_eraseBlock(tdFileSystemPageAddress _block)
	{
	MT_uint8 nandStatus;
	nand_addr_t address;

	if( (_block < FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT) == 0 )
		return Fail;

	address.block = _block;
	address.column = 0;
	address.lun = 0;
	address.page = 0;

	nandStatus = NAND_Block_Erase( address );

	if( nandStatus == NAND_BAD_BLOCK )
		{
		FILE_SYSTEM_setBadBlock( address.block );
		return Fail;
		}

	return Ok;
	}

tdStatus FILE_SYSTEM_writePage(nand_addr_t _address, tdFileSystemData *_pData, int _size)
	{
	if( NAND_Page_Program(_address, _pData, _size) != NAND_SUCCESS )
		return Fail;

	return Ok;
	}

tdStatus FILE_SYSTEM_writePage_(tdFileSystemPageAddress _pageAddress, tdFileSystemData *_pData, int _size)
	{
	nand_addr_t address;
	FILE_SYSTEM_generateNandAddress(&address, _pageAddress);

	if( FILE_SYSTEM_writePage(address, _pData, _size) != Ok )
		return Fail;

	return Ok;
	}

tdStatus FILE_SYSTEM_readPage(nand_addr_t _address, tdFileSystemData *_pData, int _size)
	{
	if( NAND_Page_Read(_address, _pData, _size) != NAND_SUCCESS )
		return Fail;

	return Ok;
	}

tdStatus FILE_SYSTEM_readPage_(tdFileSystemPageAddress _pageAddress, tdFileSystemData *_pData, int _size)
	{
	nand_addr_t address;
	FILE_SYSTEM_generateNandAddress(&address, _pageAddress);

	if( FILE_SYSTEM_readPage(address, _pData, _size) != Ok )
		return Fail;

	return Ok;
	}

tdStatus FILE_SYSTEM_writeSparePage(nand_addr_t _address, tdFileSystemData *_pData, int _size)
	{
	if( NAND_Spare_Program(_address, _pData, _size) != NAND_SUCCESS )
		return Fail;

	return Ok;
	}

tdStatus FILE_SYSTEM_writeSparePage_(tdFileSystemPageAddress _pageAddress, tdFileSystemData *_pData, int _size)
	{
	nand_addr_t address;
	FILE_SYSTEM_generateNandAddress(&address, _pageAddress);

	if( FILE_SYSTEM_writeSparePage(address, _pData, _size) != Ok )
		return Fail;

	return Ok;
	}

tdStatus FILE_SYSTEM_readSparePage(nand_addr_t _address, tdFileSystemData *_pData, int _size)
	{
	if( NAND_Spare_Read(_address, _pData, _size) != NAND_SUCCESS )
		return Fail;

	return Ok;
	}

tdStatus FILE_SYSTEM_readSparePage_(tdFileSystemPageAddress _pageAddress, tdFileSystemData *_pData, int _size)
	{
	nand_addr_t address;
	FILE_SYSTEM_generateNandAddress(&address, _pageAddress);

	if( FILE_SYSTEM_readSparePage(address, _pData, _size) != Ok )
		return Fail;

	return Ok;
	}

tdStatus FILE_SYSTEM_writeDataToFile(tdFileNode *pFile, tdFileSystemData *_pData, int _dataSize, tdFileSystemData *_pSpare, int _spareSize)
	{
	nand_addr_t address;
	unsigned char crc8;
	tdStatus status = Ok;

	// Every time when we call this function we write to next address
	// It means that every time we write to the addressEnd.
	// When we finished addressEnd will shows us address of the end of file.
	FILE_SYSTEM_generateNandAddress(&address, pFile->addressEnd);

	if( FILE_SYSTEM_writePage(address, _pData, _dataSize) != Ok )
		return Fail;

	FS_M_setFileNumber(_pSpare, pFile->fileNum);
	FS_M_setStartPage(_pSpare, pFile->addressStart);
	FS_M_setCurrentPage(_pSpare, pFile->filePageAddress);

	crc8 = CRC8_( _pSpare, FILE_SYSTEM_DATA_SPARE_CRC8_POSITION - 1 ); // crc8 of _pSpare. From  _pSpare[ 0 ] to _pSpare[ CRC8_POSITION - 1 ]

	FS_M_setCRC8(_pSpare, crc8);

	if( FILE_SYSTEM_writeSparePage(address, _pSpare, _spareSize) != Ok )
		status = Fail;

	#warning: "check data successful write if not write to next address"

	pFile->filePageAddress++;

	pFile->addressEnd++;
	if( (pFile->addressEnd < FILE_SYSTEM_MEMORY_MAX_ADDRESS) == 0 )
		pFile->addressEnd = 0; // make a ring

	while( FILE_SYSTEM_isBadBlockAtAddress( pFile->addressEnd ) == 1 )
		{
		FILE_SYSTEM_getNextBlockAddress(&pFile->addressEnd);
		}

	return status;
	}
