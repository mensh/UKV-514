#include "map.h"

void MAP_init(tdMap *_map)
	{
	_map->size = 0;
	}

void MAP_add(tdMap *_map, tdMapKey _key, tdPMapFunction _function, tdMapFunctionSettings *_pSettings)
	{
	// Memory size is const(MAP_SIZE) so make sure that it's high enough.
	tdMapHandler *pHandler;
	tdMapHandler *pNextHandler;
	int index;

	if( _map->size < MAP_SIZE == 0 )
		return;

	if( _map->size == 0 )
		{
		index = 0;
		}
	else
		{
		if( _key < _map->handler[ 0 ].key )
			{
			index = 0;
			}
		else if( _key > _map->handler[ _map->size - 1 ].key )
			{
			index = _map->size;
			}
		else
			{
			index = MAP_binarySearch(_map, _key);

			if( _map->handler[ index ].key == _key )
				return;
			}

		pNextHandler = &_map->handler[ _map->size ];
		pHandler = &_map->handler[ _map->size - 1 ];

		for( int i = _map->size; i > index; i-- )
			{
			*pNextHandler-- = *pHandler--;
			}
		}

	pHandler = &_map->handler[ index ];
	pHandler->key = _key;
	pHandler->function = _function;
	pHandler->settings.clearPacket = _pSettings->clearPacket;
	pHandler->settings.preparePacket = _pSettings->preparePacket;
	pHandler->settings.sendPacket = _pSettings->sendPacket;
	pHandler->settings.usbSetLowPriority = _pSettings->usbSetLowPriority;

	_map->size++;
	}

int MAP_binarySearch(tdMap *_map, tdMapKey _key)
	{
	int first = 0;
	int last = _map->size - 1;
	int mid;

	if( _key < _map->handler[ first ].key )
		{
		last = first;
		}
	else if( _key > _map->handler[ last ].key )
		{
		last = last;
		}
	else
		{
		while( first < last )
			{
			mid = first + (last - first) / 2;
			if(  _key <= _map->handler[ mid ].key )
				{
				last = mid;
				}
			else
				{
				first = mid + 1;
				}
			}
		}
	return last;
	}

void MAP_getElement(tdMapKey _key, tdMapHandler *_handler, tdMap *_map)
	{
	int index = MAP_binarySearch( _map, _key);
	*_handler = _map->handler[ index ];
	}
