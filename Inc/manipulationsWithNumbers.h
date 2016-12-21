#ifndef __MANIPULATIONS_WITH_NUMBERS_H_
#define __MANIPULATIONS_WITH_NUMBERS_H_



/*===============================================================================================*/
/*===============================================================================================*/
// Comments



/*===============================================================================================*/
/*===============================================================================================*/
// Includes



/*===============================================================================================*/
/*===============================================================================================*/
// Defines
#ifdef BIG_ENDIAN
	#define MWN_M_getValue8FromBuffer(_buffer, _valuePosition) \
		(\
			( (_buffer[ _valuePosition + 0 ] & 0xFF) <<  0)  \
		)

	#define MWN_M_getValue16FromBuffer(_buffer, _valuePosition) \
		(\
			( (_buffer[ _valuePosition + 0 ] & 0xFF) <<  8) |\
			( (_buffer[ _valuePosition + 1 ] & 0xFF) <<  0)  \
		)

	#define MWN_M_getValue32FromBuffer(_buffer, _valuePosition) \
		(\
			( (_buffer[ _valuePosition + 0 ] & 0xFF) << 24) |\
			( (_buffer[ _valuePosition + 1 ] & 0xFF) << 16) |\
			( (_buffer[ _valuePosition + 2 ] & 0xFF) <<  8) |\
			( (_buffer[ _valuePosition + 3 ] & 0xFF) <<  0)  \
		)

	#define MWN_M_setValue8ToBuffer(_value, _buffer, _valuePosition) \
		{\
		_buffer[ _valuePosition + 0 ] = ( (_value >>  0) & 0xFF ); \
		}

	#define MWN_M_setValue16ToBuffer(_value, _buffer, _valuePosition) \
		{\
		_buffer[ _valuePosition + 0 ] = ( (_value >>  8) & 0xFF ); \
		_buffer[ _valuePosition + 1 ] = ( (_value >>  0) & 0xFF ); \
		}

	#define MWN_M_setValue32ToBuffer(_value, _buffer, _valuePosition) \
		{\
		_buffer[ _valuePosition + 0 ] = ( (_value >> 24) & 0xFF ); \
		_buffer[ _valuePosition + 1 ] = ( (_value >> 16) & 0xFF ); \
		_buffer[ _valuePosition + 2 ] = ( (_value >>  8) & 0xFF ); \
		_buffer[ _valuePosition + 3 ] = ( (_value >>  0) & 0xFF ); \
		}
#else
	#define MWN_M_getValue8FromBuffer(_buffer, _valuePosition) \
		(\
			( (_buffer[ _valuePosition + 0 ] & 0xFF) <<  0)  \
		)

	#define MWN_M_getValue16FromBuffer(_buffer, _valuePosition) \
		(\
			( (_buffer[ _valuePosition + 1 ] & 0xFF) <<  8) |\
			( (_buffer[ _valuePosition + 0 ] & 0xFF) <<  0)  \
		)

	#define MWN_M_getValue32FromBuffer(_buffer, _valuePosition) \
		(\
			( (_buffer[ _valuePosition + 3 ] & 0xFF) << 24) |\
			( (_buffer[ _valuePosition + 2 ] & 0xFF) << 16) |\
			( (_buffer[ _valuePosition + 1 ] & 0xFF) <<  8) |\
			( (_buffer[ _valuePosition + 0 ] & 0xFF) <<  0)  \
		)

	#define MWN_M_setValue8ToBuffer(_value, _buffer, _valuePosition) \
		{\
		_buffer[ _valuePosition + 0 ] = ( (_value >>  0) & 0xFF ); \
		}

	#define MWN_M_setValue16ToBuffer(_value, _buffer, _valuePosition) \
		{\
		_buffer[ _valuePosition + 1 ] = ( (_value >>  8) & 0xFF ); \
		_buffer[ _valuePosition + 0 ] = ( (_value >>  0) & 0xFF ); \
		}
		/*
		{\
		*(_buffer + _valuePosition) = (unsigned short)_value;\
		}
		*/

	#define MWN_M_setValue32ToBuffer(_value, _buffer, _valuePosition) \
		{\
		_buffer[ _valuePosition + 3 ] = ( (_value >> 24) & 0xFF ); \
		_buffer[ _valuePosition + 2 ] = ( (_value >> 16) & 0xFF ); \
		_buffer[ _valuePosition + 1 ] = ( (_value >>  8) & 0xFF ); \
		_buffer[ _valuePosition + 0 ] = ( (_value >>  0) & 0xFF ); \
		}
		/*
		{\
		*(_buffer + _valuePosition) = (unsigned int)_value;\
		}

		*/

#endif


#define MWN_M_BITS_IN_BYTE 8

/*===============================================================================================*/
/*===============================================================================================*/
// Data types, structures, ect.



/*===============================================================================================*/
/*===============================================================================================*/



/*===============================================================================================*/
/*===============================================================================================*/



#endif
