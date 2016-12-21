#include "usbCore.h"
#include "usbd_cdc_if.h"
#include "map.h"
#include "crc.h"
#include "manipulationsWithNumbers.h"
#include "stm32DebugPrint.h"
#include "fileSystem.h"
#include "core.h"


tdPacketManager packetManeger;
tdMap usbCoreAnswers;

extern tdFat FAT;

void USB_CORE_0x01(uint8_t* _buf, uint32_t *_len)
	{
	// Device ID
	tdData *pPacket;

	USB_CORE_getPacketToPrepare(&pPacket);

	*(pPacket->data +  0 ) = 0x01;

	MWN_M_setValue16ToBuffer(BHD2_ID_DEV, pPacket->data, 1);
	MWN_M_setValue16ToBuffer(BHD2_SOFTWARE_VERSION, pPacket->data, 3);

	*(pPacket->data +  5 ) = configurationTable.exist;
	*(pPacket->data +  6 ) = 0x01;

	*(pPacket->data + 14 ) = 0x00;
	*(pPacket->data + 15 ) = *(pPacket->data + 15 ) ? 1 : 0;

	pPacket->size = USB_CORE_0x01_PACKET_SIZE;
	}

void USB_CORE_0x04(uint8_t* _buf, uint32_t *_len)
	{
	// Memory chip info
	tdData *pPacket;

	USB_CORE_getPacketToPrepare(&pPacket);

	*(pPacket->data + USB_CORE_0x04_PACKET_SERVICE_SHIFT +  0 ) = 0x04;
	*(pPacket->data + USB_CORE_0x04_PACKET_SERVICE_SHIFT +  1 ) = 0;
	*(pPacket->data + USB_CORE_0x04_PACKET_SERVICE_SHIFT + 14 ) = 0;
	*(pPacket->data + USB_CORE_0x04_PACKET_SERVICE_SHIFT + 15 ) = *(pPacket->data + USB_CORE_0x04_PACKET_SERVICE_SHIFT + 14 ) ? 1 : 0;

	pPacket->size = USB_CORE_0x04_PACKET_SIZE;
	}

void USB_CORE_0x05(uint8_t* _buf, uint32_t *_len)
	{
	// Bad blocks status
	tdData *pPacket;
	uint8_t *pBadBlockListPtr;
	uint8_t *pByte;
	uint16_t byte;

	USB_CORE_getPacketToPrepare(&pPacket);

	FILE_SYSTEM_getPtrToBadBlocksList(&pBadBlockListPtr);

	byte = USB_CORE_0x05_PACKET_SERVICE_SHIFT;
	pByte = pPacket->data;

	while( byte-- )
		*pByte++ = *pBadBlockListPtr++;

	*(pPacket->data + USB_CORE_0x05_PACKET_SERVICE_SHIFT +  0 ) = 0x05;

	*(pPacket->data + USB_CORE_0x05_PACKET_SERVICE_SHIFT + 15 ) = *(pPacket->data + USB_CORE_0x05_PACKET_SERVICE_SHIFT + 14 ) ? 1 : 0;

	pPacket->size = USB_CORE_0x05_PACKET_SIZE;
	}

void USB_CORE_ErasePage(tdData *_pPacket)
	{
	unsigned char status;

	packetManeger.packet.currentPacket = CORE_getErasingBlock();

	MWN_M_setValue8ToBuffer(0x07, _pPacket->data, 0);
	MWN_M_setValue32ToBuffer(packetManeger.packet.currentPacket, _pPacket->data, 1);

	if( CORE_isErasing() == 1 )
		{
		status = 0x02; // in progress
		}
	else
		{
		packetManeger.packet.packetsToSent = 1;
		status = 0x00; // Complete;
		}

	/*
	if( configurationTableAbsent == 1 )
		{
		*(pPacket->data + USB_CORE_0x07_PACKET_SERVICE_SHIFT + 14 ) = 1;
		status = 0x01;
		}
	*/

	MWN_M_setValue8ToBuffer(status, _pPacket->data, 15);
	_pPacket->size = USB_CORE_0x07_PACKET_SIZE;
	};

void USB_CORE_0x07(uint8_t* _buf, uint32_t *_len)
	{
	// erase main memory
	uint32_t startPage, endPage;

	//if( configurationTableAbsent == 0 )
	CORE_initEraseMainMemory();

	startPage = 0;
	endPage = FILE_SYSTEM_MEMORY_BLOCKS_AMOUNT;

	packetManeger.pDataFactory = USB_CORE_ErasePage;

	//if( configurationTableAbsent == 1 )
		//packetManeger.packet.packetsToSent = 1;
	//else
	packetManeger.packet.packetsToSent = endPage - startPage;

	packetManeger.packet.currentPacket = startPage;
	}

void USB_CORE_0x08(uint8_t* _buf, uint32_t *_len)
	{
	// Erase bad bloks list
	tdData *pPacket;

	USB_CORE_getPacketToPrepare(&pPacket);

	CORE_initClearBadBlocks();

	*(pPacket->data + USB_CORE_0x08_PACKET_SERVICE_SHIFT +  0 ) = 0x08;
	*(pPacket->data + USB_CORE_0x08_PACKET_SERVICE_SHIFT + 14 ) = 0;
	*(pPacket->data + USB_CORE_0x08_PACKET_SERVICE_SHIFT + 15 ) = *(pPacket->data + USB_CORE_0x08_PACKET_SERVICE_SHIFT + 14 ) ? 1 : 0;

	pPacket->size = USB_CORE_0x08_PACKET_SIZE;
	}

void USB_CORE_FillSmdPacket(tdData *_pPacket)
	{
	FILE_SYSTEM_readPage_( packetManeger.packet.currentPacket, _pPacket->data, USB_CORE_PAGE_SMD_DATA_SIZE);

	MWN_M_setValue8ToBuffer(0x09, _pPacket->data, USB_CORE_PAGE_SMD_DATA_SIZE + 0);
	MWN_M_setValue32ToBuffer(packetManeger.packet.currentPacket, _pPacket->data, USB_CORE_PAGE_SMD_DATA_SIZE + 1);

	#warning: "implement wrong address"
	*(_pPacket->data + USB_CORE_PAGE_SMD_DATA_SIZE + 14 ) = 0; // wrong address
	*(_pPacket->data + USB_CORE_PAGE_SMD_DATA_SIZE + 15 ) = *(_pPacket->data + USB_CORE_PAGE_SMD_DATA_SIZE + 14 ) ? 1 : 0;

	_pPacket->size = USB_CORE_FRAME_SMD_SIZE;
	};

void USB_CORE_0x09(uint8_t* _buf, uint32_t *_len)
	{
	// Read stream data
	unsigned int startPage, endPage;

	startPage = MWN_M_getValue32FromBuffer(_buf, 1);
	endPage = MWN_M_getValue32FromBuffer(_buf, 5);

	packetManeger.pDataFactory = USB_CORE_FillSmdPacket;
	packetManeger.packet.packetsToSent = 1 + endPage - startPage; //not (start...end) but [start...end]  so + 1

	packetManeger.packet.currentPacket = startPage;

	packetManeger.packet.PS = startPage;
	packetManeger.packet.PE = endPage;
	packetManeger.packet.PTS = packetManeger.packet.packetsToSent;
	}

void USB_CORE_0x1A(uint8_t* _buf, uint32_t *_len)
	{
	// Configuration table IN
	tdData *pPacket;
	uint32_t len, SBD, NTD;
	uint8_t crc8;
	uint8_t crcNotMatches;
	uint32_t byte;
	uint8_t *pByteFrom;
	uint8_t *pByteTo;

	SBD = MWN_M_getValue32FromBuffer(_buf, 25);
	NTD = MWN_M_getValue8FromBuffer(_buf, 29);

	len = 30 + SBD + NTD;

	crc8 = CRC8_( _buf + 1, len - 1);
	crcNotMatches = crc8 != *(_buf + len);// because length = 30 + SBD + NTD;

	if( crcNotMatches == 0 )
		{
		configurationTable.exist = 1;
		configurationTable.size = len - 1;
		configurationTable.SBD = SBD;
		configurationTable.NTD = NTD;

		byte = 0;
		pByteTo = configurationTable.data;
		pByteFrom = _buf + 1;
		while( byte < configurationTable.size )
			{
			*pByteTo++ = *pByteFrom++;
			byte++;
			}

		FILE_SYSTEM_saveConfigurationTable(&configurationTable);
		}

	USB_CORE_getPacketToPrepare(&pPacket);

	*(pPacket->data + USB_CORE_0x1A_PACKET_SERVICE_SHIFT +  0 ) = 0x27;
	*(pPacket->data + USB_CORE_0x1A_PACKET_SERVICE_SHIFT +  1 ) = crc8;
	*(pPacket->data + USB_CORE_0x1A_PACKET_SERVICE_SHIFT + 14 ) = crcNotMatches;
	*(pPacket->data + USB_CORE_0x1A_PACKET_SERVICE_SHIFT + 15 ) = *(pPacket->data + USB_CORE_0x1A_PACKET_SERVICE_SHIFT + 14 ) ? 1 : 0;

	pPacket->size = USB_CORE_0x1A_PACKET_SIZE;
	}

void USB_CORE_FillConfigurationTable(tdData *_pPacket)
	{
	uint32_t lenS, lenE, crc8;
	uint32_t byte;
	uint8_t *pByteFrom;
	uint8_t *pByteTo;
uint32_t z;
	if( packetManeger.packet.currentPacket == 0 )
		{
		*(_pPacket->data + 0 ) = 0x28;
		*(_pPacket->data + 1 ) = 0x5A;

		MWN_M_setValue32ToBuffer(configurationTable.SBD, _pPacket->data, 2);
		MWN_M_setValue8ToBuffer(configurationTable.NTD,  _pPacket->data, 6);

		*(_pPacket->data + 14 ) = 0; // configurationTable.exist;
		*(_pPacket->data + 15 ) = *(_pPacket->data + 14 ) ? 1 : 0;

		_pPacket->size	= USB_CORE_CONFIGURATION_TABLE_PACKET_SIZE;
		}
	else
		{
		*(_pPacket->data + 0 ) = 0x28;

		byte = 0;
		pByteTo = _pPacket->data + 1;
		pByteFrom = configurationTable.data+1;
		while( byte < configurationTable.size )
			{
			*pByteTo++ = *pByteFrom++;
			byte++;
			}

		lenS = 1;
		lenE = configurationTable.size;
		crc8 = CRC8_( (_pPacket->data + lenS), lenE);

		MWN_M_setValue8ToBuffer(crc8, _pPacket->data, lenS + configurationTable.size );

		*(_pPacket->data + lenS + configurationTable.size + 1 ) = 0x00; // configuration table absent
		*(_pPacket->data + lenS + configurationTable.size + 2 ) = *(_pPacket->data + lenS + configurationTable.size + 1 ) ? 1 : 0;

		_pPacket->size	=34;// lenS + configurationTable.size +3;
			z++;
		}
	}

void USB_CORE_0x1B(uint8_t* _buf, uint32_t *_len)
	{
	// Configuration table OUT
	uint32_t startPage, endPage;
	startPage = 0;
	endPage = 2;

	packetManeger.pDataFactory = USB_CORE_FillConfigurationTable;

	packetManeger.packet.packetsToSent = endPage - startPage;

	packetManeger.packet.currentPacket = startPage;
	}

void USB_CORE_0x1C(uint8_t* _buf, uint32_t *_len)
	{
	// Data types in
	tdData *pPacket;
	tdArincConfigurationStatus status;
	tdDataTypes dataType;
	uint8_t slotsAmount;

	USB_CORE_getPacketToPrepare(&pPacket);

	dataType = (tdDataTypes) *(_buf + 1);
	slotsAmount = *(_buf + 2);

	status = configDataTypeError;
	if( dataType == ARINC_DATA )
		{
		status = HOLT_CORE_handleSlotTD(&arinc, _buf + 3, slotsAmount);
		}

	*(pPacket->data + USB_CORE_0x1C_PACKET_SERVICE_SHIFT +  0 ) = 0x29;

	*(pPacket->data + USB_CORE_0x1C_PACKET_SERVICE_SHIFT + 14 ) = status;
	*(pPacket->data + USB_CORE_0x1C_PACKET_SERVICE_SHIFT + 15 ) = *(pPacket->data + USB_CORE_0x1C_PACKET_SERVICE_SHIFT + 14 ) ? 1 : 0;

	pPacket->size = USB_CORE_0x1C_PACKET_SIZE;
	}

void USB_CORE_FillDataTypes(tdData *_pPacket)
	{
	uint8_t channel;
	uint16_t shift;
	uint32_t STD;

	if( packetManeger.packet.currentPacket == 0 )
		{
		*(_pPacket->data + 0 ) = 0x2A;
		*(_pPacket->data + 1 ) = 0x5A;

		STD = 1 + (2 * arinc.enabledChannels);
		MWN_M_setValue32ToBuffer(STD, _pPacket->data, 2);

		*(_pPacket->data + 6 ) = 0; // 0x01 - unknown data type ????
		*(_pPacket->data + 7 ) = *(_pPacket->data + 6 ) ? 1 : 0;

		_pPacket->size = USB_CORE_0x1C_PACKET_FIRST_SIZE;
		}
	else
		{
		*(_pPacket->data + 0 ) = 0x2A;
		*(_pPacket->data + 1 ) = arinc.enabledChannels;

		shift = 2;
		for( channel = 0; channel < HOLT_CORE_CHANNELS_AMOUNT; channel++ )
			{
			if( arinc.channel[ channel ].status.enable == 0 )
				continue;

			*(_pPacket->data + shift + 0 ) = channel;
			*(_pPacket->data + shift + 1 ) = arinc.channel[ channel ].freq;

			shift += HOLT_CORE_USB_SLOT_TD_SIZE;
			}

		*(_pPacket->data + shift + 0 ) = 0; // 0x01 - unknown data type ????
		*(_pPacket->data + shift + 1 ) = *(_pPacket->data + shift + 0 ) ? 1 : 0;

		_pPacket->size = shift + 2;
		}
	}

void USB_CORE_0x1D(uint8_t* _buf, uint32_t *_len)
	{
	// Data types out
	uint32_t startPage, endPage;

	startPage = 0;
	endPage = 2;

	packetManeger.pDataFactory = USB_CORE_FillDataTypes;

	packetManeger.packet.packetsToSent = endPage - startPage;
	packetManeger.packet.currentPacket = startPage;
	}

tdFileNode *pCurrentFile;
void USB_CORE_FillFatPacket(tdData *_pPacket)
	{
	// FAT
	uint16_t slot;
	uint16_t byteShift;
	uint8_t transmitionEnd = 0;

	if( FAT.filesAmount && pCurrentFile != NULL )
		{
		byteShift = 0;
		for( slot = 0; slot < USB_CORE_0x1E_SLOTS_IN_TRANSMISSION; slot++ )
			{
			MWN_M_setValue32ToBuffer(pCurrentFile->fileNum,      _pPacket->data, byteShift + 0);
			MWN_M_setValue32ToBuffer(pCurrentFile->addressStart, _pPacket->data, byteShift + 4);
			MWN_M_setValue32ToBuffer(pCurrentFile->addressEnd,   _pPacket->data, byteShift + 8);

			if( pCurrentFile == FAT.pLastFile ) // In upper code we sent last file. Considering thet we hawe "ring FAT" and that we don't want to trap in infinite loop...
				{
				transmitionEnd = 1;
				break;
				}

			pCurrentFile = pCurrentFile->pPreviousFile;

			if( pCurrentFile == NULL )
				break;

			byteShift += 12;
			}

		slot += 1; // because slot will show amount(size)
		}
	else
		{
		transmitionEnd = 1;
		slot = 0;
		}

	MWN_M_setValue8ToBuffer(0x2B,   _pPacket->data, USB_CORE_0x1E_PACKET_SERVICE_SHIFT + 0);

	if( (pCurrentFile == FAT.pLastFile && transmitionEnd == 1) || pCurrentFile == NULL )
		{
		//NT
		MWN_M_setValue8ToBuffer(0x00, _pPacket->data, USB_CORE_0x1E_PACKET_SERVICE_SHIFT + 1);
		//CF
		MWN_M_setValue8ToBuffer(slot, _pPacket->data, USB_CORE_0x1E_PACKET_SERVICE_SHIFT + 2);
		}
	else
		{
		MWN_M_setValue8ToBuffer(0x01, _pPacket->data, USB_CORE_0x1E_PACKET_SERVICE_SHIFT + 1);
		MWN_M_setValue8ToBuffer(0x00, _pPacket->data, USB_CORE_0x1E_PACKET_SERVICE_SHIFT + 2);
		}

	_pPacket->size = USB_CORE_0x1E_PACKET_SIZE;
	};

void USB_CORE_0x1E(uint8_t* _buf, uint32_t *_len)
	{
	// FAT
	int packetsToSent;

	if( FAT.filesAmount )
		{
		pCurrentFile = FAT.pFirstFile;
		packetsToSent = (FAT.filesAmount / USB_CORE_0x1E_SLOTS_IN_TRANSMISSION) + ((FAT.filesAmount % USB_CORE_0x1E_SLOTS_IN_TRANSMISSION) ? 1 : 0);
		}
	else
		{
		pCurrentFile = NULL;
		packetsToSent = 1;
		}

	packetManeger.pDataFactory = USB_CORE_FillFatPacket;
	packetManeger.packet.packetsToSent = packetsToSent;

	packetManeger.packet.currentPacket = 0;
	}

void USB_CORE_FillSsdPacket(tdData *_pPacket)
	{
	FILE_SYSTEM_readSparePage_( packetManeger.packet.currentPacket, _pPacket->data, USB_CORE_PAGE_SSD_DATA_SIZE);

	MWN_M_setValue8ToBuffer(0x2C, _pPacket->data, USB_CORE_PAGE_SSD_DATA_SIZE + 0);
	MWN_M_setValue32ToBuffer(packetManeger.packet.currentPacket, _pPacket->data, USB_CORE_PAGE_SSD_DATA_SIZE + 1);

	_pPacket->size = USB_CORE_FRAME_SSD_SIZE;
	};

void USB_CORE_0x1F(uint8_t* _buf, uint32_t *_len)
	{
	// Read Spare Data
	uint32_t startPage, endPage;

	startPage = MWN_M_getValue32FromBuffer( _buf, 1 );
	endPage = MWN_M_getValue32FromBuffer( _buf, 5 );

	packetManeger.pDataFactory = USB_CORE_FillSsdPacket;
	packetManeger.packet.packetsToSent = 1 + endPage - startPage; //not (start...end) but [start...end]  so + 1

	packetManeger.packet.currentPacket = startPage;

	packetManeger.packet.PS = startPage;
	packetManeger.packet.PE = endPage;
	packetManeger.packet.PTS = packetManeger.packet.packetsToSent;
	}

uint32_t usbCore0x20endPage = 0;
void USB_CORE_FillSsd(tdData *_pPacket)
	{
	uint32_t page;
	uint32_t pagesToRead;
	uint32_t pagesNeededToRead;
	uint8_t *pData;

	pagesNeededToRead = 1 + usbCore0x20endPage - packetManeger.packet.currentPage;
	if( pagesNeededToRead < USB_CORE_0x20_SSD_IN_TRANSMISSION )
		pagesToRead = pagesNeededToRead;
	else
		pagesToRead = USB_CORE_0x20_SSD_IN_TRANSMISSION;

	pData = _pPacket->data;
	for( page = 0; page < pagesToRead; page++ )
		{
		FILE_SYSTEM_readSparePage_( packetManeger.packet.currentPage, pData, USB_CORE_PAGE_SSD_DATA_SIZE);

		pData += USB_CORE_PAGE_SSD_DATA_SIZE;

		packetManeger.packet.currentPage++;
		}

	MWN_M_setValue8ToBuffer(0x2D, _pPacket->data, USB_CORE_0x20_SERVICE_SHIFT + 0);
	MWN_M_setValue32ToBuffer( (packetManeger.packet.currentPage - 1), _pPacket->data, USB_CORE_0x20_SERVICE_SHIFT + 1);

	*(_pPacket->data + USB_CORE_0x20_SERVICE_SHIFT + 14 ) = 0;
	*(_pPacket->data + USB_CORE_0x20_SERVICE_SHIFT + 15 ) = *(_pPacket->data + USB_CORE_0x20_SERVICE_SHIFT + 14 ) ? 1 : 0;

	_pPacket->size = USB_CORE_0x20_PACKET_SIZE;
	};

void USB_CORE_0x20(uint8_t* _buf, uint32_t *_len)
	{
	// Read Spare Data
	uint32_t startPage, endPage;
	uint32_t pagesToSent;

	startPage = MWN_M_getValue32FromBuffer( _buf, 1 );
	endPage = MWN_M_getValue32FromBuffer( _buf, 5 );

	usbCore0x20endPage = endPage;

	packetManeger.pDataFactory = USB_CORE_FillSsd;

	pagesToSent = 1 + endPage - startPage;
	packetManeger.packet.packetsToSent = pagesToSent / USB_CORE_0x20_SSD_IN_TRANSMISSION;

	if( pagesToSent % USB_CORE_0x20_SSD_IN_TRANSMISSION != 0 )
		packetManeger.packet.packetsToSent += 1;

	packetManeger.packet.currentPage = startPage;
	packetManeger.packet.currentPacket = startPage;
	}

void USB_CORE_0xFE(uint8_t* _buf, uint32_t *_len)
	{
	tdData *pPacket = &packetManeger.packet.packet[ (packetManeger.packet.pen - 1) % FRAMES_AMOUNT ];

	if( pPacket->size > 3 ) // (pPacket->size - 3) in worst case size -> 0
		{
		*( pPacket->data + (pPacket->size - 3) ) = 1;	// Break Command
		}

	packetManeger.packet.packetsToSent = packetManeger.packet.pen - packetManeger.packet.eye;
	}

/*=================================================
					================================================
										================================================
															================================================
										================================================
					================================================
	===============================================*/

void USB_CORE_initPacketManeger(void)
	{
	packetManeger.pDataFactory = NULL;
	packetManeger.packet.pen = 0;
	packetManeger.packet.eye = 0;
	packetManeger.packet.packetsToSent = 1; // if not so function will calculate required value
	packetManeger.packet.currentPacket = 0;
	USB_CORE_clearPacketIndex( 0 , 0 );
	}

void USB_CORE_clearPacket(tdData *_pPacket, uint32_t _size)
	{
	if( _size == 0 || _size > PACKET_STRUCTURE_MAX_SIZE )
		_size = PACKET_STRUCTURE_MAX_SIZE;

	memset( _pPacket->data, 0x00, _size );
	}

void USB_CORE_clearPacketIndex(uint32_t _index, uint32_t _size)
	{
	tdData *pPacket = &packetManeger.packet.packet[ _index ];

	USB_CORE_clearPacket(pPacket, _size);
	}

void USB_CORE_clearAllPackets(void)
	{
	memset( &packetManeger.packet.packet, 0x00, PACKET_STRUCTURE_MAX_SIZE * FRAMES_AMOUNT );
	}

void USB_CORE_getPacketToPrepare(tdData **_pPacket)
	{
	*_pPacket = &packetManeger.packet.packet[ packetManeger.packet.pen % FRAMES_AMOUNT ];
	}

void USB_CORE_getPacketToSent(tdData **_pPacket)
	{
	*_pPacket = &packetManeger.packet.packet[ packetManeger.packet.eye % FRAMES_AMOUNT ];
	}

void USB_CORE_prepareNextPacket(void)
	{
	tdData *pPacket;

	if( packetManeger.pDataFactory != NULL )
		{
		USB_CORE_getPacketToPrepare( &pPacket );

		USB_CORE_clearPacket( pPacket, pPacket->size );

		packetManeger.pDataFactory( pPacket );

		packetManeger.packet.currentPacket++;
		packetManeger.packet.pen++;
		}
	}

void USB_CORE_requestHandler(uint8_t *_buf, uint32_t *_len)
	{
	tdMapHandler handler;

	MAP_getElement( _buf[ 0 ], &handler, &usbCoreAnswers );

	if( handler.function == NULL )
		return;

	if( handler.settings.usbSetLowPriority == USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_TRUE )
		{
		packetManeger.usbSetHighPriority = 1;
		HAL_NVIC_SetPriority(OTG_HS_IRQn, 3, 3);
		}

	if( handler.settings.clearPacket == USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_TRUE )
		{
		memset(&packetManeger, 0x00, sizeof(packetManeger));
		USB_CORE_initPacketManeger();
		}

	handler.function(_buf, _len);

	if( handler.settings.preparePacket == USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_TRUE )
		{
		USB_CORE_prepareNextPacket();
		}

	if( handler.settings.sendPacket == USB_CORE_HANDLER_SETTINGS_PACKET_SEND_TRUE )
		{
		USB_CORE_sendPacket();
		}

	}

void USB_CORE_sendPacket(void)
	{
	tdData *packet;

	if( packetManeger.packet.packetsToSent == 0 )
		return;

	USB_CORE_getPacketToSent(&packet);

	if( packet->data == NULL || packet->size == 0 )
		return;

	CDC_Transmit_HS(packet->data, packet->size);

	packetManeger.packet.eye++;
	packetManeger.packet.packetsToSent--;

	if( packetManeger.packet.packetsToSent != 0 )
		{
		USB_CORE_prepareNextPacket();
		}
	else
		{
		if( packetManeger.usbSetHighPriority == 1 )
			{
			HAL_NVIC_SetPriority(OTG_HS_IRQn, 0, 0);
			}
		}

	}

void USB_CORE_init(void)
	{
	tdMapFunctionSettings settings;

	MAP_init(&usbCoreAnswers);

	settings.clearPacket = USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_TRUE;
	settings.preparePacket = USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_FALSE;
	settings.sendPacket = USB_CORE_HANDLER_SETTINGS_PACKET_SEND_TRUE;
	settings.usbSetLowPriority = USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_FALSE;
	MAP_add(&usbCoreAnswers, 0x01, USB_CORE_0x01, &settings);

	settings.clearPacket = USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_TRUE;
	settings.preparePacket = USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_FALSE;
	settings.sendPacket = USB_CORE_HANDLER_SETTINGS_PACKET_SEND_TRUE;
	settings.usbSetLowPriority = USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_FALSE;
	MAP_add(&usbCoreAnswers, 0x04, USB_CORE_0x04, &settings);

	settings.clearPacket = USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_TRUE;
	settings.preparePacket = USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_FALSE;
	settings.sendPacket = USB_CORE_HANDLER_SETTINGS_PACKET_SEND_TRUE;
	settings.usbSetLowPriority = USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_FALSE;
	MAP_add(&usbCoreAnswers, 0x05, USB_CORE_0x05, &settings);

	settings.clearPacket = USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_TRUE;
	settings.preparePacket = USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_TRUE;
	settings.sendPacket = USB_CORE_HANDLER_SETTINGS_PACKET_SEND_TRUE;
	settings.usbSetLowPriority = USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_FALSE;
	MAP_add(&usbCoreAnswers, 0x07, USB_CORE_0x07, &settings);

	settings.clearPacket = USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_TRUE;
	settings.preparePacket = USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_FALSE;
	settings.sendPacket = USB_CORE_HANDLER_SETTINGS_PACKET_SEND_TRUE;
	settings.usbSetLowPriority = USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_FALSE;
	MAP_add(&usbCoreAnswers, 0x08, USB_CORE_0x08, &settings);

	settings.clearPacket = USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_TRUE;
	settings.preparePacket = USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_TRUE;
	settings.sendPacket = USB_CORE_HANDLER_SETTINGS_PACKET_SEND_TRUE;
	settings.usbSetLowPriority = USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_FALSE;
	MAP_add(&usbCoreAnswers, 0x09, USB_CORE_0x09, &settings);

	settings.clearPacket = USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_TRUE;
	settings.preparePacket = USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_FALSE;
	settings.sendPacket = USB_CORE_HANDLER_SETTINGS_PACKET_SEND_TRUE;
	settings.usbSetLowPriority = USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_TRUE;
	MAP_add(&usbCoreAnswers, 0x27, USB_CORE_0x1A, &settings);

	settings.clearPacket = USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_TRUE;
	settings.preparePacket = USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_TRUE;
	settings.sendPacket = USB_CORE_HANDLER_SETTINGS_PACKET_SEND_TRUE;
	settings.usbSetLowPriority = USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_FALSE;
	MAP_add(&usbCoreAnswers, 0x28, USB_CORE_0x1B, &settings);

	settings.clearPacket = USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_TRUE;
	settings.preparePacket = USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_TRUE;
	settings.sendPacket = USB_CORE_HANDLER_SETTINGS_PACKET_SEND_TRUE;
	settings.usbSetLowPriority = USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_TRUE;
	MAP_add(&usbCoreAnswers, 0x29, USB_CORE_0x1C, &settings);

	settings.clearPacket = USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_TRUE;
	settings.preparePacket = USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_TRUE;
	settings.sendPacket = USB_CORE_HANDLER_SETTINGS_PACKET_SEND_TRUE;
	settings.usbSetLowPriority = USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_FALSE;
	MAP_add(&usbCoreAnswers, 0x2A, USB_CORE_0x1D, &settings);

	settings.clearPacket = USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_TRUE;
	settings.preparePacket = USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_TRUE;
	settings.sendPacket = USB_CORE_HANDLER_SETTINGS_PACKET_SEND_TRUE;
	settings.usbSetLowPriority = USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_FALSE;
	MAP_add(&usbCoreAnswers, 0x2B, USB_CORE_0x1E, &settings);

	settings.clearPacket = USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_TRUE;
	settings.preparePacket = USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_TRUE;
	settings.sendPacket = USB_CORE_HANDLER_SETTINGS_PACKET_SEND_TRUE;
	settings.usbSetLowPriority = USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_FALSE;
	MAP_add(&usbCoreAnswers, 0x2C, USB_CORE_0x1F, &settings);

	settings.clearPacket = USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_TRUE;
	settings.preparePacket = USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_TRUE;
	settings.sendPacket = USB_CORE_HANDLER_SETTINGS_PACKET_SEND_TRUE;
	settings.usbSetLowPriority = USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_FALSE;
	MAP_add(&usbCoreAnswers, 0x2D, USB_CORE_0x20, &settings);

	settings.clearPacket = USB_CORE_HANDLER_SETTINGS_PACKET_CLEAR_FALSE;
	settings.preparePacket = USB_CORE_HANDLER_SETTINGS_PACKET_PREPARE_FALSE;
	settings.sendPacket = USB_CORE_HANDLER_SETTINGS_PACKET_SEND_FALSE;
	settings.usbSetLowPriority = USB_CORE_HANDLER_SETTINGS_SET_LOW_PRIORITY_FALSE;
	MAP_add(&usbCoreAnswers, 0xFE, USB_CORE_0xFE, &settings);
	}


