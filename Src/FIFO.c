#include "stm32f2xx_hal.h"
#include "FIFO.h"
#include <stdlib.h>

#define false 0 
#define true 1

       
fifo_t* createFifoToHeap(uint32_t size)
{
    uint8_t* buffer = (uint8_t*)malloc(size);
		fifo_t* fifo = (fifo_t*)malloc(sizeof(fifo_t));
    if (buffer == NULL)
        return NULL;
 
   
 
    if (fifo == NULL)
    {
       free(buffer);
       return NULL;
    }
 
    fifo->buffer = buffer;
    fifo->head = 0;
    fifo->tail = 0;
    fifo->size = size;
 
    return fifo;
}
 
#define CHECK_FIFO_NULL(fifo) if (fifo == NULL) return 0
 
size_t fifoPushByte(fifo_t* fifo, uint8_t byte)
{
    CHECK_FIFO_NULL(fifo);
 
    if (fifoIsFull(fifo) == true)
       return 0;
 
    fifo->buffer[fifo->head] = byte;
 
    fifo->head++;
    if (fifo->head == fifo->size)
       fifo->head = 0;
 
    return 1;
}
 
uint8_t fifoPushBytes(fifo_t* fifo, uint8_t* bytes, uint32_t count)
{
	uint32_t i;
    CHECK_FIFO_NULL(fifo);
 
    for ( i = 0; i < count; i++)
    {
        if (fifoPushByte(fifo, bytes[i]) == 0)
            return i;
    }
 
    return count;
}
 
uint8_t fifoPopByte(fifo_t* fifo, uint8_t* byte)
{
    CHECK_FIFO_NULL(fifo);
 
    if (fifoIsEmpty(fifo) == true)
        return 0;
 
    *byte = fifo->buffer[fifo->tail];
 
    fifo->tail++;
    if (fifo->tail == fifo->size)
        fifo->tail = 0;
 
    return 1;
}
 
uint8_t fifoPopBytes(fifo_t* fifo, uint8_t* bytes, uint32_t count)
{
		uint32_t i;
    CHECK_FIFO_NULL(fifo);
 
    for (i = 0; i < count; i++)
    {
        if (fifoPopByte(fifo, bytes + i) == 0)
            return i;
    }
 
    return count;
}
 
uint8_t fifoIsFull(fifo_t* fifo)
{
    if ((fifo->head == (fifo->size - 1) && fifo->tail == 0) || (fifo->head == (fifo->tail - 1)))
        return true;
    else
        return false;
}
 
uint8_t fifoIsEmpty(fifo_t* fifo)
{
    if (fifo->head == fifo->tail)
        return true;
    else
        return false;
}
 
uint32_t fifoBytesFilled(fifo_t* fifo)
{
    if (fifo->head == fifo->tail)
        return 0;
    else if ((fifo->head == (fifo->size - 1) && fifo->tail == 0) || (fifo->head == (fifo->tail - 1)))
        return fifo->size;
    else if (fifo->head < fifo->tail)
        return (fifo->head) + (fifo->size - fifo->tail);
    else
        return fifo->head - fifo->tail;
}
