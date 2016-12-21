
#include "stdint.h"
#include "core.h"


typedef enum { ArincToMemory, MemoryToUsb, arincToUsb, UsbToArinc } tdEnumStreamDirection;


typedef struct
	{
	uint8_t *pData;
	uint16_t size;
 	uint16_t maxSize;
	} tdDataBuffer;


typedef struct
	{
	tdEnumStreamDirection direction;
	tdEnumDataType dataType;
	tdDataBuffer *buffer;

	} tdStream;


void DATA_STREAM_init(tdStream *_pStream, tdEnumStreamDirection _direction, tdEnumDataType _type)
	{
	};












//void HOLT_CORE_ReedAvailableData(tdEnumSpiDevice_0 _device, uint8_t *_pBuffer, uint32_t *_pCurrentSize, uint32_t _maxSize)



