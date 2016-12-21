#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f2xx_hal.h"
typedef struct RingBuffer {
    char *buffer;
    volatile unsigned int writeIndex;
    volatile unsigned int readIndex;
    size_t capacity;
    size_t count;
    size_t eleSize;
} RingBuffer;

// Nelem will be rounded up to a power of 2
RingBuffer * rbInit(size_t nelem, size_t elemsize);
void rbDestroy(RingBuffer *rb);

void rbPush(RingBuffer *rb, uint8_t *elem);
void rbPushN(RingBuffer *rb, uint8_t *elem, int n);

void rbPop(RingBuffer *rb, uint8_t *elem);
void rbPopN(RingBuffer *rb, uint8_t *elem, int n);

//If you want to increment the read index a number other than n
void rbPopOption(RingBuffer *rb, uint8_t *elem, int n, int readIndexIncrement);

size_t rbWriteCapacity(RingBuffer *rb);
size_t rbReadCapacity(RingBuffer *rb);

int rbFull(RingBuffer *rb);
int rbEmpty(RingBuffer *rb);

int rbCanPush(RingBuffer *rb, int n);
int rbCanPop(RingBuffer *rb, int n);

void rbPrintInt(RingBuffer *rb);
void rbPrintFloat(RingBuffer *rb);

int rbRoundNextPowerof2(int inVal);
