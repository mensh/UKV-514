
//#define CHECK_FIFO_NULL(fifo) if (fifo == NULL) return 0

typedef struct {
     uint8_t * buffer;
	   uint32_t elemSize;
     uint32_t head;
     uint32_t tail;
     uint32_t size;
} fifo_t;


fifo_t* createFifoToHeap(uint32_t size);

uint8_t fifoPushElem(fifo_t* fifo, void* data);

uint8_t fifoPushBytes(fifo_t* fifo, uint8_t* bytes, uint32_t count);

void* fifoPopElem(fifo_t* fifo);

uint8_t fifoPopBytes(fifo_t* fifo, uint8_t* bytes, uint32_t count);

uint8_t fifoIsFull(fifo_t* fifo);

uint8_t fifoIsEmpty(fifo_t* fifo);

uint32_t fifoBytesFilled(fifo_t* fifo);
