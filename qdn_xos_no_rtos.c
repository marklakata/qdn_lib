#include "qdn_xos.h"
#include <string.h>

// RTOS less queue


void XOS_FixedQueueCreateEx(XOS_FixedQueue_t* queue,uint8_t nelements,uint8_t elemSize,uint8_t* buffer) {
    queue->buffer    = buffer;
    queue->elemSize  = elemSize;
    queue->ptrRead   = 0;
    queue->ptrWrite  = 0;
    queue->ptrEnd    = nelements*elemSize;
}

#if 0
void XOS_FixedQueueSendFromISREx(XOS_FixedQueue_t* queue, void* pElement)    { 
    uint16_t next;
    next = queue->ptrWrite + queue->elemSize;
    if (next >= queue->ptrEnd) next = 0;
    if (next != queue->read) { // only append if space is available
        memcpy( queue->buffer[queue->ptrWrite],(uint8_t*)pElement,queue->elemSize);
        queue->ptrWrite = next;
    }
}
#endif

void XOS_FixedQueueSendEx(XOS_FixedQueue_t* queue, void* pElement)    { 
    uint16_t next;
    next = queue->ptrWrite + queue->elemSize;
    if (next >= queue->ptrEnd) next = 0;

    while (next == queue->ptrRead) ; // spin until these differ.

    memcpy( &queue->buffer[queue->ptrWrite], pElement, queue->elemSize);
    queue->ptrWrite = next;
}


//lint -esym(438,timeout)
uint8_t XOS_FixedQueueReceiveTimedEx(XOS_FixedQueue_t* queue,void* pElement,uint16_t timeout) {
    timeout = timeout;
    if (queue->ptrRead != queue->ptrWrite) {
        uint16_t next;
        memcpy( pElement, &queue->buffer[queue->ptrRead], queue->elemSize);
        next = queue->ptrRead +1;
        if (next >= queue->ptrEnd) next = 0;
        queue->ptrRead = next;
        return 0;
    } else {
        return 1;
    }
}


