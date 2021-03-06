#ifndef __CRC_H_
#define __CRC_H_



/*===============================================================================================*/
/*===============================================================================================*/
// Comments



/*===============================================================================================*/
/*===============================================================================================*/
// Includes



/*===============================================================================================*/
/*===============================================================================================*/
// Defines
#define CRC_32_TABLE_SIZE 256
#define CRC_32_START_VALUE 0xAA0A4C5FL

#define CRC_8_TABLE_SIZE 256
#define CRC_8_START_VALUE 2


/*===============================================================================================*/
/*===============================================================================================*/
// Data types, structures, ect.



/*===============================================================================================*/
/*===============================================================================================*/
unsigned int CRC32( unsigned char const *_data, unsigned int _len, unsigned int _inCrc32 );
unsigned int CRC32_( unsigned char const *_data, unsigned int _len );

unsigned int CRC8( unsigned char const *_data, unsigned int _len, unsigned int _inCrc8 );
unsigned int CRC8_( unsigned char const *_data, unsigned int _len );

/*===============================================================================================*/
/*===============================================================================================*/



#endif
