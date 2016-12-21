#ifndef __FILE_SYSTEM_H_
#define __FILE_SYSTEM_H_



/*===============================================================================================*/
/*===============================================================================================*/
// Comments



/*===============================================================================================*/
/*===============================================================================================*/
// Includes
#include "manipulationsWithNumbers.h"
#include "tdStatus.h"
#include "nand_MT29F_lld.h"
#include "stdint.h"
#include "holtCore.h"


/*===============================================================================================*/
/*===============================================================================================*/
// Defines
#define FILE_SYSTEM_MALLOC_ENABLE 1

//----------------------------------------------------------------
#define FILE_SYSTEM_MEMORY_PAGE_DATA_SIZE 4096
#define FILE_SYSTEM_MEMORY_SPARE_DATA_SIZE 224
#define FILE_SYSTEM_MEMORY_PAGE_SIZE (FILE_SYSTEM_MEMORY_PAGE_DATA_SIZE + FILE_SYSTEM_MEMORY_SPARE_DATA_SIZE)

#define FILE_SYSTEM_MEMORY_PAGES_AMOUNT 128
#define FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT 4096
#define FILE_SYSTEM_MEMORY_MAX_ADDRESS (FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT * FILE_SYSTEM_MEMORY_PAGES_AMOUNT)

//----------------------------------------------------------------
#define FILE_SYSTEM_BAD_BLOCK_TEST_BYTES_SIZE 1
#define FILE_SYSTEM_BAD_BLOCK_TEST_SUCCESS_VALUE 0xFF

#define FILE_SYSTEM_ADDRESS_OF_BAD_BLOCKS_LIST 0x0100
#define FILE_SYSTEM_MEMORY_BAD_BLOCK_LIST_SIZE (FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT / 8)

#define FILE_SYSTEM_ADDRESS_OF_DATA_TYPE_TABLE 0x0800
#define FILE_SYSTEM_DATA_TYPE_SIZE 2
#define FILE_SYSTEM_ADDRESS_OF_DATA_TYPE_SIZE ( HOLT_CORE_CHANNELS_AMOUNT * FILE_SYSTEM_DATA_TYPE_SIZE )

#define FILE_SYSTEM_ADDRESS_OF_CONFIGURATION_TABLE_EXIST 0xFF0
#define FILE_SYSTEM_ADDRESS_OF_CONFIGURATION_TABLE_SIZE 0xFF1
#define FILE_SYSTEM_ADDRESS_OF_CONFIGURATION_TABLE_SBD 0xFF5
#define FILE_SYSTEM_ADDRESS_OF_CONFIGURATION_TABLE_NTD 0xFF9
#define FILE_SYSTEM_ADDRESS_OF_CONFIGURATION_TABLE 0x1000
#define FILE_SYSTEM_CONFIGURATION_TABLE_MAX_SIZE 0x2000
#define FILE_SYSTEM_CONFIGURATION_TABLE_DEFAULT_SIZE 31

#define FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_SSD 0
#define FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_SMD 4
#define FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_SB  8
#define FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_SM 12
#define FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_SD 16
#define FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_SERIAL_N 20
#define FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_SBD 24
#define FILE_SYSTEM_CONFIGURATION_TABLE_POSITION_NTD 28

#define FILE_SYSTEM_ADDRESS_OF_COUNTERS_TABLE 0x3000

//----------------------------------------------------------------
#define FILE_SYSTEM_DATA_SPARE_DATA_TYPE_POSITION 0
#define FILE_SYSTEM_DATA_SPARE_FILE_NUMBER_POSITION 1
#define FILE_SYSTEM_DATA_SPARE_START_PAGE_POSITION 5
#define FILE_SYSTEM_DATA_SPARE_CURRENT_PAGE_POSITION 9
#define FILE_SYSTEM_DATA_SPARE_TIME_BEGIN_POSITION 13
#define FILE_SYSTEM_DATA_SPARE_TIME_END_POSITION 17
#define FILE_SYSTEM_DATA_SPARE_CRC32_POSITION 21
#define FILE_SYSTEM_DATA_SPARE_CRC8_POSITION 25

//----------------------------------------------------------------
#define FS_M_getDataType(_buffer) 						MWN_M_getValue8FromBuffer(_buffer, FILE_SYSTEM_DATA_SPARE_DATA_TYPE_POSITION)
#define FS_M_setDataType(_buffer, _value)			MWN_M_setValue8ToBuffer(_value, _buffer, FILE_SYSTEM_DATA_SPARE_DATA_TYPE_POSITION)

#define FS_M_getFileNumber(_buffer)						MWN_M_getValue32FromBuffer(_buffer, FILE_SYSTEM_DATA_SPARE_FILE_NUMBER_POSITION)
#define FS_M_setFileNumber(_buffer, _value)		MWN_M_setValue32ToBuffer(_value, _buffer, FILE_SYSTEM_DATA_SPARE_FILE_NUMBER_POSITION)

#define FS_M_getStartPage(_buffer)						MWN_M_getValue32FromBuffer(_buffer, FILE_SYSTEM_DATA_SPARE_START_PAGE_POSITION)
#define FS_M_setStartPage(_buffer, _value)		MWN_M_setValue32ToBuffer(_value, _buffer, FILE_SYSTEM_DATA_SPARE_START_PAGE_POSITION)

#define FS_M_getCurrentPage(_buffer)					MWN_M_getValue32FromBuffer(_buffer, FILE_SYSTEM_DATA_SPARE_CURRENT_PAGE_POSITION)
#define FS_M_setCurrentPage(_buffer, _value)	MWN_M_setValue32ToBuffer(_value, _buffer, FILE_SYSTEM_DATA_SPARE_CURRENT_PAGE_POSITION)

#define FS_M_getTimeBegin(_buffer)						MWN_M_getValue32FromBuffer(_buffer, FILE_SYSTEM_DATA_SPARE_TIME_BEGIN_POSITION)
#define FS_M_setTimeBegin(_buffer, _value)		MWN_M_setValue32ToBuffer(_value, _buffer, FILE_SYSTEM_DATA_SPARE_TIME_BEGIN_POSITION)

#define FS_M_getTimeBegin(_buffer)						MWN_M_getValue32FromBuffer(_buffer, FILE_SYSTEM_DATA_SPARE_TIME_BEGIN_POSITION)
#define FS_M_setTimeBegin(_buffer, _value)		MWN_M_setValue32ToBuffer(_value, _buffer, FILE_SYSTEM_DATA_SPARE_TIME_BEGIN_POSITION)

#define FS_M_getTimeEnd(_buffer)							MWN_M_getValue32FromBuffer(_buffer, FILE_SYSTEM_DATA_SPARE_TIME_END_POSITION)
#define FS_M_setTimeEnd(_buffer, _value)			MWN_M_setValue32ToBuffer(_value, _buffer, FILE_SYSTEM_DATA_SPARE_TIME_END_POSITION)

#define FS_M_getCRC32(_buffer)								MWN_M_getValue32FromBuffer(_buffer, FILE_SYSTEM_DATA_SPARE_CRC32_POSITION)
#define FS_M_setCRC32(_buffer, _value)				MWN_M_setValue32ToBuffer(_value, _buffer, FILE_SYSTEM_DATA_SPARE_CRC32_POSITION)

#define FS_M_getCRC8(_buffer)									MWN_M_getValue8FromBuffer(_buffer, FILE_SYSTEM_DATA_SPARE_CRC8_POSITION)
#define FS_M_setCRC8(_buffer, _value)					MWN_M_setValue8ToBuffer(_value, _buffer, FILE_SYSTEM_DATA_SPARE_CRC8_POSITION)


/*===============================================================================================*/
/*===============================================================================================*/
// Data types, structures, ect.

typedef unsigned int tdFileSystemPageAddress;
typedef unsigned int tdFileSystemFileNumber;
typedef unsigned char tdFileSystemData;
typedef unsigned int tdFileSystemBlock;

typedef struct _tdFileNode
	{
	tdFileSystemFileNumber fileNum;
	tdFileSystemPageAddress addressStart;
	tdFileSystemPageAddress addressEnd;
	tdFileSystemPageAddress filePageAddress;
	struct _tdFileNode *pNextFile;
	struct _tdFileNode *pPreviousFile;
	} tdFileNode;

typedef struct
	{
	tdFileSystemFileNumber filesAmount;
	tdFileNode *pFirstFile;
	tdFileNode *pLastFile;
	} tdFat;

typedef struct
	{
	uint8_t dataType;
	tdFileSystemFileNumber fileNum;
	tdFileSystemPageAddress addressStart;
	tdFileSystemPageAddress addressEnd;
	uint8_t crc8;
	uint32_t timeStart;
	uint32_t timeEnd;
	} tdSparePageData;

typedef struct
	{
	tdFileSystemFileNumber value;
	tdFileNode *pFile;
	} tdFileMinMax;

typedef struct
	{
	tdFileSystemPageAddress address;
	uint32_t time;
	} tdFileAddressAndTime;

typedef struct
	{
	uint32_t size;
	uint32_t SBD;
	uint8_t exist;
	uint8_t NTD;
	uint8_t data[ 1024 ];
	} tdConfigurationTable;

/*===============================================================================================*/
/*===============================================================================================*/
tdStatus FILE_SYSTEM_saveConfigurationTable(tdConfigurationTable *_pConfigTable);

tdStatus FILE_SYSTEM_readConfigurationTable(tdConfigurationTable *_pConfigTable);

tdStatus FILE_SYSTEM_saveHoltConfiguration(tdArinc *_pArinc);

tdStatus FILE_SYSTEM_readHoltConfiguration(tdArinc *_pArinc);

tdStatus FILE_SYSTEM_init(tdFat *_pFat);

tdStatus FILE_SYSTEM_getPtrToBadBlocksList(uint8_t **_pBadBlockListPtr);

tdStatus FILE_SYSTEM_clearBadBlocksList(void);

tdStatus FILE_SYSTEM_clearBadBlocksList_(uint32_t _from, uint32_t _size);

tdStatus FILE_SYSTEM_readBadBlocksList(void);

tdStatus FILE_SYSTEM_saveBadBlockStatusToMemmory(tdFileSystemBlock _block, uint8_t _status);

tdStatus FILE_SYSTEM_setBadBlock(tdFileSystemBlock _block);

tdStatus FILE_SYSTEM_clearBadBlock(tdFileSystemBlock _block);

uint8_t FILE_SYSTEM_isBadBlock(tdFileSystemBlock _block);

uint8_t FILE_SYSTEM_isBadBlockAtAddress(tdFileSystemPageAddress _pageAddress);

tdStatus FILE_SYSTEM_getSparePageData(const nand_addr_t *_pAddress, tdSparePageData *_pSpareData, uint8_t *_pSparePage);

tdStatus FILE_SYSTEM_getLastPageAddressInBlock(nand_addr_t *_pAddress, uint8_t *_pSparePage);

// @Function:tdStatus FILE_SYSTEM_createFatTable(tdFat *_pFat)
// Read NAND Flash and create FAT Table based on spare information( fileNumber )
// NOTE:	When we save file to NAND Flash file number is growing. Each new file number = number + 1.
// 				But when download files to PC we enumerate files from last written, so first == lastWritten with highest fileNumber.
//				So _pFat->pFirstFile is the newest file and _pFat->pLastFile is the oldest.
tdStatus FILE_SYSTEM_createFatTable(tdFat *_pFat);

tdStatus FILE_SYSTEM_clearFatTable(tdFat *_pFat);

tdStatus FILE_SYSTEM_getAddressForNewFile(tdFileSystemPageAddress *_pPageAddress);

tdStatus FILE_SYSTEM_newFile(tdFat *_pFat);

tdStatus FILE_SYSTEM_getNextBlockAddress(tdFileSystemPageAddress *_pPageAddress);

tdStatus FILE_SYSTEM_getPreviousBlockAddress(tdFileSystemPageAddress *_pPageAddress);

tdStatus FILE_SYSTEM_generateNandAddress(nand_addr_t *_pAddress, tdFileSystemPageAddress _pageAddress);

tdStatus FILE_SYSTEM_eraseBlock(tdFileSystemPageAddress _pageAddress);

tdStatus FILE_SYSTEM_writePage(nand_addr_t _address, tdFileSystemData *_pData, int _size);

tdStatus FILE_SYSTEM_writePage_(tdFileSystemPageAddress _pageAddress, tdFileSystemData *_pData, int _size);

tdStatus FILE_SYSTEM_readPage(nand_addr_t _address, tdFileSystemData *_pData, int _size);

tdStatus FILE_SYSTEM_readPage_(tdFileSystemPageAddress _pageAddress, tdFileSystemData *_pData, int _size);

tdStatus FILE_SYSTEM_writeSparePage(nand_addr_t _address, tdFileSystemData *_pData, int _size);

tdStatus FILE_SYSTEM_writeSparePage_(tdFileSystemPageAddress _pageAddress, tdFileSystemData *_pData, int _size);

tdStatus FILE_SYSTEM_readSparePage(nand_addr_t _address, tdFileSystemData *_pData, int _size);

tdStatus FILE_SYSTEM_readSparePage_(tdFileSystemPageAddress _pageAddress, tdFileSystemData *_pData, int _size);

tdStatus FILE_SYSTEM_writeDataToFile(tdFileNode *pFile, tdFileSystemData *_pData, int _dataSize, tdFileSystemData *_pSpare, int _spareSize);


/*===============================================================================================*/
/*===============================================================================================*/



#endif
