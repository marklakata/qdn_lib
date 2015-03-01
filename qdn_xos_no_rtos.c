/**************************************************************************
 *
 * Copyright (c) 2013, Qromodyn Corporation
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
 **************************************************************************/

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


#include <stm32f10x.h>
#include <core_cm3.h>

static volatile uint32_t systicks;

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
    systicks++;
}

/**
 * @brief  Returns current value of systicks
 * @param  None
 * @retval Milliseconds since boot
 */
uint32_t XOS_GetTime32(void)
{
    return systicks;
}

/**
 * @brief  blocks execution for some time
 * @param  ms : the number of milliseconds to wait
 * @retval None
 */
void XOS_DelayMs(uint32_t ms)
{
    uint32_t t0 = XOS_GetTime32();
    while (XOS_MillisecondElapsedU32(t0) < ms)
        ;

}


void XOS_Delay100Ns(uint16_t count)
{
    volatile uint32_t limit = DWT->CYCCNT;
    // assume 72 MHz, round up to 80 MHz and divide by 10 to get a factor of 8
    limit += count * 8;
    while (DWT->CYCCNT < limit)
        ;
}
