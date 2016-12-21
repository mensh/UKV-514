#include "core.h"
#include "eepromDriver.h"
#include "fileSystem.h"
#include "usbCore.h"
#include "holtCore.h"
#include "crc.h"
#include "counter.h"
#include "stm32f2xx_hal_gpio.h"
#include "usb_device.h"
#include "adcCore.h"
#include "bhd2settings.h"
#include "timCore.h"
#include "FIFO.h"
#include "ringbuffer.h"
tdConfigurationTable configurationTable;
tdFat FAT;
tdPagesQueue pagesQueue;
tdArinc arinc;
//extern fifo_t* fifo_buf;				
static tdMainMemoryErase mainMemoryErase;
static tdClearBadBlocks	clearBadBlocks;

void CORE_threadGetData(void);
//RINGBUFFER_EXTERN(rb);

tdUsbConectionStatus isUsbConnected(void)
	{
	uint16_t vBus = 0;
	uint16_t vBusChecks = 0xFF;

	while( vBusChecks-- )
		{
		if( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) )
			{
			vBus++;
			}
		}

	return ( vBus > 0xFF / 2 ) ? usbConnect : usbNotConnect;
	}


void (*CORE_mode)(void);
void CORE_deviceMode(void);
void CORE_usbMode(void);

	
#define USB_CORE_0x1B_SDB 0
#define USB_CORE_0x1B_NTD 1
#define USB_CORE_0x1B_PACKET_SIZE_0 ( USB_CORE_SERVICE_DATA_SIZE )
#define USB_CORE_0x1B_PACKET_SERVICE_SHIFT 0

#define USB_CORE_0x1B_PACKET_SIZE_1 ( 30 + USB_CORE_0x1B_SDB + USB_CORE_0x1B_NTD + 2 + 1)
#define USB_CORE_0x1B_PACKET_SERVICE_SHIFT_1 0

void fillConfTable(void)
	{
	// Configuration table OUT		
	int lenS, lenE, crc8;
	memset(configurationTable.data, 0x00, USB_CORE_0x1B_PACKET_SIZE_1);
	*(configurationTable.data + USB_CORE_0x1B_PACKET_SERVICE_SHIFT_1 +  0 ) = 0x28;
	MWN_M_setValue32ToBuffer((unsigned int)FILE_SYSTEM_MEMORY_SPARE_DATA_SIZE, configurationTable.data, USB_CORE_0x1B_PACKET_SERVICE_SHIFT_1 + 1);
	MWN_M_setValue32ToBuffer((unsigned int)FILE_SYSTEM_MEMORY_PAGE_DATA_SIZE, configurationTable.data, USB_CORE_0x1B_PACKET_SERVICE_SHIFT_1 + 5);		
	MWN_M_setValue32ToBuffer((unsigned int)FILE_SYSTEM_MEMORY_PAGES_AMOUNT, configurationTable.data, USB_CORE_0x1B_PACKET_SERVICE_SHIFT_1 + 9);	
	MWN_M_setValue32ToBuffer((unsigned int)FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT, configurationTable.data, USB_CORE_0x1B_PACKET_SERVICE_SHIFT_1 + 13);
	MWN_M_setValue32ToBuffer((unsigned int)1, configurationTable.data, USB_CORE_0x1B_PACKET_SERVICE_SHIFT_1 + 17);	
	MWN_M_setValue32ToBuffer((unsigned int)137132342, configurationTable.data, USB_CORE_0x1B_PACKET_SERVICE_SHIFT_1 + 21);
	//MWN_M_setValue32ToBuffer((unsigned int)USB_CORE_0x1B_SDB, configurationTable.data, USB_CORE_0x1B_PACKET_SERVICE_SHIFT_1 + 25);
	
	*(configurationTable.data + USB_CORE_0x1B_PACKET_SERVICE_SHIFT_1 + 25 ) = (unsigned int)USB_CORE_0x1B_SDB;
		
	MWN_M_setValue8ToBuffer(USB_CORE_0x1B_NTD, configurationTable.data, USB_CORE_0x1B_PACKET_SERVICE_SHIFT_1 + 29);			
	MWN_M_setValue8ToBuffer(0x10, configurationTable.data, USB_CORE_0x1B_PACKET_SERVICE_SHIFT_1 + 30 + USB_CORE_0x1B_SDB);
	lenS = 1;
	lenE = 30 + USB_CORE_0x1B_SDB + USB_CORE_0x1B_NTD - 1;
	crc8 = CRC8_( (configurationTable.data + lenS), lenE);	
	MWN_M_setValue8ToBuffer(crc8, configurationTable.data, USB_CORE_0x1B_PACKET_SERVICE_SHIFT_1 + 30 + USB_CORE_0x1B_SDB + USB_CORE_0x1B_NTD);		
		
	*(configurationTable.data + USB_CORE_0x1B_PACKET_SERVICE_SHIFT_1 + 30 + USB_CORE_0x1B_SDB + USB_CORE_0x1B_NTD + 1 ) = 0x00;	
	*(configurationTable.data + USB_CORE_0x1B_PACKET_SERVICE_SHIFT_1 + 30 + USB_CORE_0x1B_SDB + USB_CORE_0x1B_NTD + 2 ) = 0x00;
	
	configurationTable.SBD = USB_CORE_0x1B_SDB;
	configurationTable.NTD = USB_CORE_0x1B_NTD;
	configurationTable.size = USB_CORE_0x1B_PACKET_SIZE_1;			
	}	
	
extern USBD_HandleTypeDef hUsbDeviceHS;	
	uint8_t usb_connect_can;
void CORE_init(void)
	{
	tdUsbConectionStatus usbConectionStatus;
	fillConfTable();

	FILE_SYSTEM_init(&FAT);
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);	
	//FILE_SYSTEM_readConfigurationTable(&configurationTable);
	MX_USB_DEVICE_Init();
	
	HAL_Delay(1000);		
	if (hUsbDeviceHS.dev_state==USBD_STATE_CONFIGURED /*HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9)==GPIO_PIN_RESET*/) 
	{
		usbConectionStatus = usbConnect;//isUsbConnected();
	}
	else 
	{
		usbConectionStatus = usbNotConnect;
		//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);	
		MX_TIM7_Init();
		MX_TIM3_Init();	
		
	}
	if( usbConectionStatus == usbConnect )
		{
		CORE_mode = CORE_usbMode;
		USB_CORE_init();
		usb_connect_can=1;	
		}
	else
		{
		CORE_mode = CORE_deviceMode;

		FILE_SYSTEM_newFile(&FAT);

		tdMemoryPage *pPage = &pagesQueue.page[ 0 ];
		pPage->dataSize = 0;
		pPage->dataReady = 0;
		pagesQueue.eye = 0;
		pagesQueue.pen = 0;

		#warning:
//		TIM_CORE_initTim7();
//		TIM_CORE_setCallback(&htim7, CORE_threadGetData);
		
		//HOLT_CORE_init( &HOLT3598, &arinc );
		}

	}

void CORE_initEraseMainMemory(void)
	{
	mainMemoryErase.erase = 1;
	mainMemoryErase.erasingBlock = 0;
	}

uint32_t CORE_getErasingBlock(void)
	{
	return mainMemoryErase.erasingBlock;
	}

uint32_t CORE_isErasing(void)
	{
	return mainMemoryErase.erase;
	}

void CORE_threadEraseMainMemory(void)
	{
	FILE_SYSTEM_eraseBlock( mainMemoryErase.erasingBlock );

	mainMemoryErase.erasingBlock++;
	if( (mainMemoryErase.erasingBlock < FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT ) == 0 )
		{
		mainMemoryErase.erase = 0;
		FILE_SYSTEM_clearFatTable(&FAT);
		FILE_SYSTEM_createFatTable(&FAT);
		}
	}

void CORE_initClearBadBlocks(void)
	{
	clearBadBlocks.clear = 1;
	clearBadBlocks.counter = 0;
	}

void CORE_threadClearBadBlocks(void)
	{
	#define CORE_CLEAR_BLOCKS 16
	FILE_SYSTEM_clearBadBlocksList_( clearBadBlocks.counter, CORE_CLEAR_BLOCKS );

	clearBadBlocks.counter += CORE_CLEAR_BLOCKS;
	if( (clearBadBlocks.counter < FILE_SYSTEM_MEMORY_BAD_BLOCK_LIST_SIZE) == 0 )
		{
		clearBadBlocks.clear = 0;
		clearBadBlocks.counter = 0;
		}
	#undef CORE_CLEAR_BLOCKS
	}

/*
(tdArinc *_pArinc, tdArincChannel *_pChannel, uint8_t _channelNumber) - A little bit over, but i don't know exactly what will be needed.
*/
tdChannelState CORE_holtIsChannelActive(tdArinc *_pArinc, tdArincChannel *_pChannel, uint8_t _channelNumber)
	{
	tdChannelState active = sameState;

	return active;
	}

void CORE_threadHoltActiveChannels(void)
	{
	
	}
extern RingBuffer *rb;
void CORE_threadGetData(void)
	{
	uint32_t time;
	static uint32_t prevDataSize = 0;
	static uint32_t crc32 = 0;
	uint32_t  counter=0;
	tdMemoryPage *pPage = &pagesQueue.page[ pagesQueue.pen % CORE_PAGES_QUEUE_SIZE ];

	time = HAL_GetTick();
  counter = rbReadCapacity(rb);
	if( counter > 4095)// pPage->dataSize < CORE_PAGES_QUEUE_DATA_MAX_SIZE )
		{
			//counter=fifoBytesFilled(fifo_buf);
		
			//fifoPopBytes(fifo_buf,pPage->data,4095);
//			pPage->dataSize=ringbuffer_read(&rb,pPage->data,4095);
			rbPopN(rb, pPage->data, 4095);
			pPage->dataSize = 4096;
		}
		//}

	if( prevDataSize == 0 && pPage->dataSize != 0 )
		{
		MWN_M_setValue8ToBuffer( ARINC429, pPage->spare, FILE_SYSTEM_DATA_SPARE_DATA_TYPE_POSITION);
		MWN_M_setValue32ToBuffer( time, pPage->spare, FILE_SYSTEM_DATA_SPARE_TIME_BEGIN_POSITION);
		crc32 = CRC32_(pPage->data,  pPage->dataSize);
		}

	if( prevDataSize != pPage->dataSize )
		{
		crc32 = CRC32(pPage->data + prevDataSize,  pPage->dataSize, crc32);
		}

	prevDataSize = pPage->dataSize;

	if( ((pPage->dataSize < CORE_PAGES_QUEUE_DATA_MAX_SIZE) == 0)  )
		{
		MWN_M_setValue32ToBuffer( time, pPage->spare, FILE_SYSTEM_DATA_SPARE_TIME_END_POSITION);
		MWN_M_setValue32ToBuffer( crc32, pPage->spare, FILE_SYSTEM_DATA_SPARE_CRC32_POSITION);

		pPage->dataReady = 1;
		pagesQueue.pen++;
		counter=0;
		prevDataSize = 0;
		}
		
	}

void CORE_threadClearNextBlock(void)
	{
	static uint32_t prevBlock = FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT + 1;
	nand_addr_t address;

	FILE_SYSTEM_generateNandAddress(&address, FAT.pFirstFile->addressEnd);

	if( prevBlock != address.block )
		{
		FILE_SYSTEM_eraseBlock( address.block );

		while( FILE_SYSTEM_isBadBlock( address.block ) == 1 )
			{
			FILE_SYSTEM_getNextBlockAddress(&FAT.pFirstFile->addressEnd);

			FILE_SYSTEM_generateNandAddress(&address, FAT.pFirstFile->addressEnd);

			FILE_SYSTEM_eraseBlock( address.block );
			}

		prevBlock = address.block;
		}
	}

void CORE_threadSaveToMemory(void)
	{
	tdMemoryPage *pPage = &pagesQueue.page[ pagesQueue.eye % CORE_PAGES_QUEUE_SIZE ];

	if( pPage->dataReady )
		{
		FILE_SYSTEM_writeDataToFile(FAT.pFirstFile, pPage->data, FILE_SYSTEM_MEMORY_PAGE_DATA_SIZE, pPage->spare, FILE_SYSTEM_MEMORY_SPARE_DATA_SIZE);
		
		#warning: "memset(pPage->data, 0x00, FILE_SYSTEM_MEMORY_SPARE_DATA_SIZE) ? "
		
		pPage->dataReady = 0;
		pPage->dataSize = 0;

		pagesQueue.eye++;
		}
	}

void CORE_deviceMode(void)
	{
	CORE_threadGetData();

	CORE_threadClearNextBlock();
	CORE_threadSaveToMemory();
	}

void CORE_usbMode(void)
	{
	if( mainMemoryErase.erase == 1 && clearBadBlocks.clear == 0 )
		{
		HAL_NVIC_DisableIRQ(OTG_HS_IRQn);

		CORE_threadEraseMainMemory();

		HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
		}
	if( clearBadBlocks.clear == 1 )
		{
		CORE_threadClearBadBlocks();
		}
	}

void CORE_threadMain(void)
	{
	CORE_mode();
	}


