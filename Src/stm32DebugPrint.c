#include "stm32DebugPrint.h"

#if defined( DEBUG_PRINT )
/*
void DWT_Init(void)
	{
	SCB_DEMCR  	|= 0x01000000;
	DWT_CYCCNT   = 0;
	DWT_CONTROL	|= 1; // enable the counter
	}
	*/

int fputc(int ch, FILE *f)
	{
	if( DEMCR & TRCENA )
		{
		while( ITM_Port32(0) == 0 );
		ITM_Port8(0) = ch;
		}
	return( ch );
	}

unsigned int STM32_DEBUG_PRINT_MSG_NUMBER = 0;

#endif
