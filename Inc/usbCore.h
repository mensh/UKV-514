#ifndef __USB_CORE_H__
#define __USB_CORE_H__



/*===============================================================================================*/
/*===============================================================================================*/
// Comments



/*===============================================================================================*/
/*===============================================================================================*/
// Includes
#include "stdint.h"
#include "fileSystem.h"


/*===============================================================================================*/
/*===============================================================================================*/
// Defines


#define USB_CORE_ANSWERS_AMOUNT 31

#define USB_CORE_SIMPLE_ANSWER_PACKET_SIZE 1024

#define USB_CORE_SERVICE_DATA_SIZE 16

#define USB_CORE_0x01_PACKET_SIZE USB_CORE_SERVICE_DATA_SIZE


#define USB_CORE_0x04_PACKET_SIZE 528
#define USB_CORE_0x04_PACKET_SERVICE_SHIFT (USB_CORE_0x04_PACKET_SIZE - USB_CORE_SERVICE_DATA_SIZE)


#define USB_CORE_0x05_PACKET_SIZE ((FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT / 8) + USB_CORE_SERVICE_DATA_SIZE)
#define USB_CORE_0x05_PACKET_SERVICE_SHIFT (USB_CORE_0x05_PACKET_SIZE - USB_CORE_SERVICE_DATA_SIZE)

#define USB_CORE_0x07_PACKET_SIZE USB_CORE_SERVICE_DATA_SIZE
#define USB_CORE_0x07_PACKET_SERVICE_SHIFT 0

#define USB_CORE_0x08_PACKET_SIZE USB_CORE_SERVICE_DATA_SIZE
#define USB_CORE_0x08_PACKET_SERVICE_SHIFT 0

#define USB_CORE_0x1A_PACKET_SIZE USB_CORE_SERVICE_DATA_SIZE
#define USB_CORE_0x1A_PACKET_SERVICE_SHIFT 0

#define USB_CORE_0x1B_SBD 0
#define USB_CORE_0x1B_NTD 1
#define USB_CORE_CONFIGURATION_TABLE_PACKET_SIZE ( USB_CORE_SERVICE_DATA_SIZE )
#define USB_CORE_CONFIGURATION_TABLE_PACKET_SHIFT 0

#define USB_CORE_0x1B_PACKET_SIZE ( 30 + USB_CORE_0x1B_SBD + USB_CORE_0x1B_NTD + 2 + 1 )
#define USB_CORE_0x1B_PACKET_SERVICE_SHIFT 0

#define USB_CORE_0x1C_PACKET_FIRST_SIZE ( 8 )
#define USB_CORE_0x1C_PACKET_SIZE ( 16 )
#define USB_CORE_0x1C_PACKET_SERVICE_SHIFT 0

#define USB_CORE_0x1D_PACKET_SIZE ( 16 )
#define USB_CORE_0x1D_PACKET_SERVICE_SHIFT 0

#define USB_CORE_0x1E_PACKET_SIZE 197
#define USB_CORE_0x1E_PACKET_SERVICE_SHIFT 192
#define USB_CORE_0x1E_SLOTS_IN_TRANSMISSION 16

#define USB_CORE_PAGE_SERVICE_DATA_SIZE 16
#define USB_CORE_PAGE_SMD_DATA_SIZE 4096

#define USB_CORE_FRAME_SMD_SIZE (USB_CORE_PAGE_SMD_DATA_SIZE + USB_CORE_PAGE_SERVICE_DATA_SIZE)

#define USB_CORE_PAGE_SSD_DATA_SIZE 224
#define USB_CORE_FRAME_SSD_SIZE (USB_CORE_PAGE_SSD_DATA_SIZE + USB_CORE_PAGE_SERVICE_DATA_SIZE)

#define USB_CORE_0x20_SSD_IN_TRANSMISSION ( USB_CORE_PAGE_SMD_DATA_SIZE / USB_CORE_PAGE_SSD_DATA_SIZE )
#define USB_CORE_0x20_SSD_SIZE ( USB_CORE_0x20_SSD_IN_TRANSMISSION * USB_CORE_PAGE_SSD_DATA_SIZE )
#define USB_CORE_0x20_PACKET_SIZE ( USB_CORE_0x20_SSD_SIZE + USB_CORE_SERVICE_DATA_SIZE )
#define USB_CORE_0x20_SERVICE_SHIFT USB_CORE_0x20_SSD_SIZE

#define FRAMES_AMOUNT (4)
#define PACKET_STRUCTURE_MAX_SIZE USB_CORE_FRAME_SMD_SIZE

#define BHD2_ID_DEV 0x020A
#define BHD2_SOFTWARE_VERSION 0x0001

#define USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_FALSE 0
#define USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_TRUE 1
#define USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_FALSE 0
#define USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_TRUE 1
#define USB_CORE_HANDLER_SETTINGS_PACKET_SEND_FALSE 0
#define USB_CORE_HANDLER_SETTINGS_PACKET_SEND_TRUE 1
#define USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_FALSE 0
#define USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_TRUE 1



/*===============================================================================================*/
/*===============================================================================================*/
// Data types, structures, ect.
typedef struct
	{
	uint8_t data[ USB_CORE_FRAME_SMD_SIZE ];
	uint32_t size;
	} tdData;

typedef struct
	{
	tdData packet[ FRAMES_AMOUNT ];
	uint32_t pen;
	uint32_t eye;
	uint32_t packetsToSent;
	uint32_t currentPacket;
	uint32_t currentPage;
	// temp
	uint32_t PTS;
	uint32_t PS;
	uint32_t PE;
	//
	} tdPacket;

typedef struct
	{
	void (*pDataFactory)(tdData *_pPacket);
	tdPacket packet;
	uint8_t usbSetHighPriority;
	} tdPacketManager;


/*===============================================================================================*/
/*===============================================================================================*/
void USB_CORE_initPacketManeger(void);
void USB_CORE_clearPacket(tdData *_pPacket, uint32_t _size);
void USB_CORE_clearPacketIndex(uint32_t _index, uint32_t _size);
void USB_CORE_clearAllPackets(void);
void USB_CORE_getPacketToPrepare(tdData **_pPacket);
void USB_CORE_getPacketToSent(tdData **_pPacket);
void USB_CORE_prepareNextPacket(void);
void USB_CORE_sendPacket(void);
void USB_CORE_requestHandler(uint8_t *_buf, uint32_t *_len);
void USB_CORE_init(void);



/*===============================================================================================*/
/*===============================================================================================*/



#endif
