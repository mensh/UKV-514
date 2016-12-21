
#include "ringbuffer.h"


int rbRoundNextPowerof2(int inVal) {
    int pow2 = 1;
    while (inVal > pow2 && pow2 < (1 << 30) - 1) {
        pow2 = pow2 << 1;
    }
    return pow2;  
}

RingBuffer * rbInit(size_t nelem, size_t elemsize) {
    RingBuffer *rb = malloc(sizeof(RingBuffer));
    rb->capacity = rbRoundNextPowerof2(nelem);
    rb->eleSize = elemsize;
    rb->readIndex = rb->writeIndex = 0;
    rb->buffer = calloc(rb->capacity, elemsize);
    
    return rb;
}

void rbDestroy(RingBuffer *rb) {
    free(rb->buffer);
    free(rb);
}

void rbPush(RingBuffer *rb, uint8_t *elem) {
    
    memmove(rb->buffer + (rb->writeIndex++ & (rb->capacity - 1))*rb->eleSize,elem , rb->eleSize);   
}

void rbPushN(RingBuffer *rb, uint8_t *elem, int n) { 
    int index = 0;
    while (index < n) {
        rbPush(rb, elem + (index++ * rb->eleSize));
    }
}

void rbPop(RingBuffer *rb, uint8_t *elem) { 
    memmove(elem, rb->buffer + (rb->readIndex++ & (rb->capacity - 1))*rb->eleSize, rb->eleSize);
}

void rbPopN(RingBuffer *rb, uint8_t *elem, int n) {
    int index = 0;
    while (index < n) {
        rbPop(rb, elem + (index++ * rb->eleSize));
    }
}

//If you want to increment the readIndex by a value other than N
void rbPopOption(RingBuffer *rb, uint8_t *elem, int n, int readIndexIncrement) {
    rbPopN(rb, elem, n);
    rb->readIndex -= n - readIndexIncrement;  
}

size_t rbWriteCapacity(RingBuffer *rb) {
    return rb->capacity - rbReadCapacity(rb);
}

size_t rbReadCapacity(RingBuffer *rb) {
    return rb->writeIndex - rb->readIndex;
}

int rbFull(RingBuffer *rb) {
    return rbWriteCapacity(rb) == 0 ? 1 : 0;
}
int rbEmpty(RingBuffer *rb) {
    return rbReadCapacity(rb) == 0 ? 1 : 0;
}

int rbCanPush(RingBuffer *rb, int n) { 
    return rbWriteCapacity(rb) >= n ? 1 : 0;
}

int rbCanPop(RingBuffer *rb, int n) {
    return rbReadCapacity(rb) >= 0 ? 1 : 0;
}

void rbPrintDetails(RingBuffer *rb) {
    printf("ri:%i wi:%i c:%i f:%i e:%i wc:%i rc:%i\n",(int)rb->readIndex,(int)rb->writeIndex,(int)rb->capacity,rbFull(rb),rbEmpty(rb),(int)rbWriteCapacity(rb),(int)rbReadCapacity(rb));
}

void rbPrintInt(RingBuffer *rb) {
    int i;
    void *bufferStart = rb->buffer;
    for (i = 0; i < rb->capacity; i++) {
        printf("%i,",*((int *)bufferStart + i));
    }
    printf("\n");
}

void rbPrintFloat(RingBuffer *rb) {
    int i;
    void *bufferStart = rb->buffer;
    for (i = 0; i < rb->capacity; i++) {
        printf("%f,",*((float *)bufferStart+ i));
    }
    printf("\n");
}
