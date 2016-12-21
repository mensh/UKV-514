#ifndef _STM32_DEBUG_PRINT_
#define _STM32_DEBUG_PRINT_

/*
NOTE: You must enable trace: Options for target -> Debug::Settsings -> Trace: set Core Cloc to the value of ProcessFreq and check "Trace Enable";
			Also call DWT_Init form  main and define DEBUG_PRINT in sorces or whole project
*/

#if defined( DEBUG_PRINT )
	#include "stdio.h"

	#define	DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
	#define	DWT_CONTROL   *(volatile unsigned long *)0xE0001000
	#define	SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

	#define ITM_Port8(n)  (*((volatile unsigned char *)(0xE0000000 + 4 * n)))
	#define ITM_Port16(n) (*((volatile unsigned short*)(0xE0000000 + 4 * n)))
	#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000 + 4 * n)))

	#define DEMCR         (*((volatile unsigned long *)(0xE000EDFC)))
	#define TRCENA        0x01000000

	extern void DWT_Init(void);
	extern unsigned int STM32_DEBUG_PRINT_MSG_NUMBER;

	#define PRINT(_MSG) 									{ printf(_MSG); printf("\n");}

	#define PRINT_VAL(_MSG, _val) 				{ printf(_MSG, _val); printf("\n");}

	#define PRINT_BUFFER(_ptrBuffer, _len)	\
		int _LEN_ = _len - 1;									\
		uint8_t* printBuf = _ptrBuffer;				\
		printf("Len: %d", _len);							\
		printf("\tBuffer: ");									\
		while( _LEN_ )												\
			{																		\
			printf("%x",*printBuf);							\
			printBuf++;													\
			_LEN_--;														\
			}																		\
		printf("\n");

	#define PRINT_ORDINAL(_MSG) 					{ printf("%d. ", STM32_DEBUG_PRINT_MSG_NUMBER++);	\
																					printf(_MSG"\n");																	}

	#define PRINT_ORDINAL_VAL(_MSG, _val)	{ printf("%d. ", STM32_DEBUG_PRINT_MSG_NUMBER++);	\
																					printf(_MSG"\n", _val);														}

	#define PRINT_FUNC 										{ printf("%d. ", STM32_DEBUG_PRINT_MSG_NUMBER++);	\
																					printf("Func: %s\n",__FUNCTION__); 						}

	#define PRINT_LOCATION 								{ printf("%d. ", STM32_DEBUG_PRINT_MSG_NUMBER++);	\
																					printf("File: %s", __FILE__); 									\
																					printf("\t Line: %d", __LINE__); 								\
																					printf("\t Func: %s\n",__FUNCTION__); 						}


	#define PRINT_LOCATION_AND(_MSG)			{ printf("%d. ", STM32_DEBUG_PRINT_MSG_NUMBER++);	\
																					printf("File: %s", __FILE__); 									\
																					printf("\t Line: %d", __LINE__); 								\
																					printf("\t foo: %s\t",__FUNCTION__); 						\
																					printf(_MSG"\n"); 																}

#else
	#define PRINT(_MSG)
	#define PRINT_VAL(_MSG, _val)
	#define PRINT_BUFFER(_ptrBuffer, _len)
	#define PRINT_ORDINAL(_MSG)
	#define PRINT_ORDINAL_VAL(_MSG, _val)
	#define PRINT_FUNC
	#define PRINT_LOCATION
	#define PRINT_LOCATION_AND(_MSG)

#endif

#endif
