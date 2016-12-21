#ifndef __MAP_H__
#define __MAP_H__



/*===============================================================================================*/
/*===============================================================================================*/
// Comments



/*===============================================================================================*/
/*===============================================================================================*/
// Includes
#include "stdint.h"
#include "usbCore.h"




/*===============================================================================================*/
/*===============================================================================================*/
// Defines
#define MAP_SIZE 32



/*===============================================================================================*/
/*===============================================================================================*/
// Data types, structures, ect.

typedef void ( *tdPMapFunction)(uint8_t *, uint32_t *);
typedef unsigned int tdMapKey;

typedef struct
	{
	uint8_t clearPacket;
	uint8_t preparePacket;
	uint8_t sendPacket;
	uint8_t usbSetLowPriority;
	} tdMapFunctionSettings;

typedef struct
	{
	tdMapKey key;
	tdPMapFunction function;
	tdMapFunctionSettings settings;
	} tdMapHandler;

typedef struct
	{
	tdMapHandler handler[ MAP_SIZE ];
	int size;
	} tdMap;



/*===============================================================================================*/
/*===============================================================================================*/
void MAP_init(tdMap *_map);

void MAP_add(tdMap *_map, tdMapKey _key, tdPMapFunction _function, tdMapFunctionSettings *_pSettings);

int MAP_binarySearch(tdMap *_map, tdMapKey _key);

void MAP_getElement(tdMapKey _key, tdMapHandler *_handler, tdMap *_map);


/*===============================================================================================*/
/*===============================================================================================*/



#endif
