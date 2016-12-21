#ifndef __CORE_H_
#define __CORE_H_



/*===============================================================================================*/
/*===============================================================================================*/
// Comments



/*===============================================================================================*/
/*===============================================================================================*/
// Includes
#include "tdStatus.h"
#include "stdint.h"
#include "fileSystem.h"


/*===============================================================================================*/
/*===============================================================================================*/
// Defines
#define CORE_PAGES_QUEUE_SIZE 4
#define CORE_PAGES_QUEUE_DATA_MAX_SIZE ((unsigned int)(FILE_SYSTEM_MEMORY_PAGE_DATA_SIZE / HOLT_CORE_READ_BYTES_AMOUNT) * HOLT_CORE_READ_BYTES_AMOUNT)

#define CORE_ACTIVE_CHANNEL_DEFAULT_TRIGGER ( 1 << 13 )

/*===============================================================================================*/
/*===============================================================================================*/
// Data types, structures, ect.
typedef struct
	{
	uint8_t data[ FILE_SYSTEM_MEMORY_PAGE_DATA_SIZE ];
	uint8_t spare[ FILE_SYSTEM_MEMORY_SPARE_DATA_SIZE ];
	uint32_t dataSize;
	uint8_t dataReady;
	} tdMemoryPage;

typedef struct
	{
	tdMemoryPage page[ CORE_PAGES_QUEUE_SIZE ];
	uint32_t eye;
	uint32_t pen;
	} tdPagesQueue;

typedef struct
	{
	uint8_t erase;
	uint32_t erasingBlock;
	} tdMainMemoryErase;

typedef struct
	{
	uint8_t clear;
	uint32_t counter;
	} tdClearBadBlocks;

typedef enum { usbNotConnect = 0x00, usbConnect } tdUsbConectionStatus;
typedef enum { ARINC_DATA = 0x10 } tdDataTypes;




/*===============================================================================================*/
/*===============================================================================================*/
tdArincConfigurationStatus CORE_arincHandleSlotTD( uint8_t *_pData, uint8_t _slotsAmount);

void CORE_init(void);

void CORE_threadMain(void);

void CORE_initEraseMainMemory(void);
uint32_t CORE_isErasing(void);
uint32_t CORE_getErasingBlock(void);
void CORE_initClearBadBlocks(void);

/*===============================================================================================*/
/*===============================================================================================*/
extern tdArinc arinc;
extern tdConfigurationTable configurationTable;


#endif
