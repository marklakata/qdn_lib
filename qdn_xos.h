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

// generic OS hook
// Currently a wrapper around FreeRTOS and PowerPac

// To Use:
// define this in your project settings (-D option)
//
// #define USE_RTOS
//
// and one of the following in "xos_config."
//
//  #define POWERPAC
//  #define FREERTOS
//  #define UCOS1992 (not fully supported yet)
//  #define NO_RTOS


#ifndef _QDN_XOS_H_
#define _QDN_XOS_H_

#include <stdint.h>

#ifdef USE_RTOS

#include <xos_config.h>

#ifdef POWERPAC
/////////////////////// PowerPAC
//

#include "RTOS.h"
#ifndef OS_QDN_MODIFIED_RTOS_HEADER
#  error Please rename OS_Config.h (perhaps in C:\Program Files\IAR Systems\Embedded Workbench xxx\arm\PowerPac\RTOS\Inc to OS_ConfigTemplate.h)
#endif


typedef int         XOS_PeriodicTimer_t;
typedef OS_RSEMA    XOS_Mutex_t;
typedef OS_MAILBOX  XOS_FixedQueue_t;
typedef OS_TASK     XOS_TCB_t;
typedef OS_TIMER    XOS_Timer_t;
typedef OS_TIME     XOS_Time_t;
typedef OS_Q        XOS_VariableQueue_t;

_EXTERN_C
void    OS_InitHW_2(void); // custom version of OS_InitHW that does not reset the VICs. Defined in RTOSINIT_STR912.c
_END_EXTERN_C

#define XOS_TaskPrototype(name)   void name( void )
#define XOS_CurrentTCB()          OS_GetpCurrentTask()
#define XOS_TaskSelfDelete()      -not implemented-

#define XOS_DelayMs(x)            OS_Delay( x )
#define XOS_GetTimeU32()          OS_GetTime32()
#define XOS_GetTime32()           OS_GetTime32()

#define XOS_PeriodicDelayInit(a)  do { a = OS_GetTime(); } while (0)
#define XOS_PeriodicDelayMs(a,d)  do { OS_DelayUntil( a += d );   } while (0)

#define XOS_TaskCreate(tcb,func,name,stacksize,prio)  do { static uint32_t _stack[stacksize]; OS_CreateTask(&tcb, name, prio, func, _stack, sizeof(_stack), 2 ); } while (0)
#define XOS_TaskPriority(tcb,prio)                    do { OS_SetPriority(&tcb, prio); }  while(0)
#define XOS_IsTask(tcb)                               OS_IsTask(&tcb)
#define XOS_TaskExit()                                do { OS_Terminate (NULL); } while(0)
        
#define XOS_Init0()               do {OS_InitKern(); OS_INIT_SYS_LOCKS(); XOS_StackInit();} while(0)
#define XOS_Init1()               do {OS_InitHW_2();} while(0)
#define XOS_StartScheduler()      OS_Start()
#define XOS_ShutDown()            OS_DI()

// diasbles task switching (but interrupts still work)
#define XOS_EnterCritical()       OS_EnterRegion()
#define XOS_LeaveCritical()       OS_LeaveRegion()

// disables interrupts in a nested fasion

#define XOS_IncDI()               OS_IncDI()
#define XOS_DecRI()               OS_DecRI()

#define XOS_Schedule()                                   OS_Delay( 1 )                    /// allow other tasks to run

#define XOS_MutexCreate(semaphore)            do { OS_CREATERSEMA(&semaphore); } while (0)
#define XOS_MutexTake(semaphore)              do { OS_Use(&semaphore);         } while (0)
#define XOS_MutexRelease(semaphore)           do { OS_Unuse(&semaphore);       } while (0) 
#define XOS_MutexDestroy(semaphore)           do { OS_DeleteRSema(&semaphore); } while (0)

//typedef OS_SEMA OS_Semaphore_t;
//#define XOS_SemaphoreCreate(semaphore)        do { OS_CREATERSEMA?(&semaphore); } while (0)
//#define XOS_SemaphoreSignalFromISR(semaphore) do { static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE; xSemaphoreGiveFromISR(semaphore,&xHigherPriorityTaskWoken); } while (0)
//#define XOS_SemaphoreWait(semaphore)          do { xSemaphoreTake( semaphore,portMAX_DELAY); } while (0)

#define XOS_FixedQueueCreate(queue,nelements,elemSize)     do { static uint8_t _buffer[(nelements)*(elemSize)]; OS_CREATEMB(&queue,(elemSize),(nelements),(_buffer));  } while (0)
#define XOS_FixedQueueCreateEx(queue,nelements,elemSize,buffer)     do { OS_CREATEMB(&queue,(elemSize),(nelements),(buffer));  } while (0)
#define XOS_FixedQueueSendFromISR(queue,pElement)          OS_PutMailCond(&queue, (pElement) )
#define XOS_FixedQueueSend(queue,pElement)                 do { OS_PutMail(&queue, (pElement) ); } while (0) // will suspend the task until it completes
#define XOS_FixedQueueSendTimed(queue,pElemen,timeout)     do { OS_PutMailTimed(&queue, (pElement),(timeout) ); } while (0) // will suspend the task until it completes
#define XOS_FixedQueueReceive(queue,pElement)              do { OS_GetMail(&queue, (pElement) ); } while (0)
#define XOS_FixedQueueReceiveTimed(queue,pElement,timeout) OS_GetMailTimed(&queue,(pElement),(timeout)) // returns XOS_TIMEOUT_RESULT if there is a timeout
#define XOS_FixedQueueAvailable(queue)                     OS_GetMessageCnt(&queue)
#define XOS_TIMEOUT_RESULT                                 1 

#define XOS_SignalTaskEvent(event,task)               do { OS_SignalEvent((event),&task);} while(0)
#define XOS_WaitEvent(event)                      OS_WaitEvent(event)
#define XOS_WaitEventTimed(event,timeout)         OS_WaitEventTimed((event),(timeout))

#define XOS_RetriggerTimer(timer)                 do { OS_RetriggerTimer(&timer);} while(0)
#define XOS_CreateTimer(timer, isr, interval)     do { OS_CreateTimer(&timer,(isr),(interval)); } while(0)
#define XOS_StartTimer(timer)                     do { OS_StartTimer(&timer); } while(0)

#define XOS_VariableQueueCreate(queue,storage,totalsize)       do { OS_Q_Create(&queue, storage, totalsize); } while(0)
#define XOS_VariableQueueSend(queue,ptr,size)                  OS_Q_Put(&queue, (ptr),(size))
#define XOS_VariableQueueBlockingSend(queue,ptr,size,polltime) do { int stat; do { stat = OS_Q_Put(&queue, (ptr),(size)); if (stat != 0) { XOS_DelayMs(polltime); }} while (stat != 0); } while(0)
#define XOS_VariableQueueReceive(queue,pptr,size)              do { (size) = OS_Q_GetPtr(&queue, (void**)(pptr)); } while (0)
#define XOS_VariableQueueReceiveTimed(queue,pptr,size,timeout) do { (size) = OS_Q_GetPtrTimed(&queue, (void**)(pptr), timeout); } while (0)
#define XOS_VariableQueueRelease(queue)                        do { OS_Q_Purge(&queue); } while (0)

//
/////////////////////// PowerPAC

#elif defined(FREERTOS)

/////////////////////// FreeRTOS
//
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "assert.h"

typedef portTickType         XOS_PeriodicTimer_t;
typedef xSemaphoreHandle     XOS_Mutex_t;
typedef xSemaphoreHandle     XOS_Event_t;
typedef xQueueHandle         XOS_FixedQueue_t;
typedef xTaskHandle          XOS_TCB_t;

#define XOS_TaskPrototype(name)  void name( void *pvParameters )
extern void* pxCurrentTCB; // not normally visible
#define XOS_CurrentTCB()         pxCurrentTCB
#define XOS_TaskSelfDelete()     do {vTaskDelete(NULL); } while(0)

//#if (portTICK_RATE_MS) == 1
//#define XOS_DelayMs(x)           vTaskDelay( (x))       /// halt the task for 'x' milliseconds.
//#define XOS_GetTime32()          (xTaskGetTickCount())
//#else
#define XOS_DelayMs(x)           vTaskDelay( (x)/portTICK_RATE_MS)       /// halt the task for 'x' milliseconds.
#define XOS_GetTime32()          (xTaskGetTickCount()/portTICK_RATE_MS)
#define XOS_GetTimeU32()         (xTaskGetTickCount()/portTICK_RATE_MS)
//#endif

#define XOS_PeriodicDelayInit(a)   do {a = xTaskGetTickCount(); } while (0)
#define XOS_PeriodicDelayMs(a,d) vTaskDelayUntil(&a,d)

#define XOS_Init0()               /* does nothing. the OS is initialized in vTaskStartScheduler */
#define XOS_Init1()               /* does nothing. the OS is initialized in vTaskStartScheduler */
#define XOS_TaskCreate(tcb,func,name,stacksize,prio)   xTaskCreate(func,name, stacksize/sizeof(uint32_t), NULL, prio, &tcb )
#define Mode_SVC 0x0013
#ifdef __ARM_PROFILE_M__
#define ASSERT_SVC()
#else
#define ASSERT_SVC() do { if ((__get_CPSR() & 0x1F) != Mode_SVC) while(1) } while(0)
#endif
#define XOS_StartScheduler()     do { ASSERT_SVC(); vTaskStartScheduler(); } while (0)

#define XOS_EnterCritical()      taskENTER_CRITICAL()
#define XOS_LeaveCritical()      taskEXIT_CRITICAL()

#define XOS_Schedule()           vTaskDelay( 1 )                    /// allow other tasks to run

#define XOS_MutexCreate(semaphore)     do { semaphore = xSemaphoreCreateMutex(); assert(semaphore); } while (0)
// infinitely blocking
// Note, this will call vTaskSuspend indirectly.
#define XOS_MutexTake(semaphore)       do { xSemaphoreTake(semaphore,portMAX_DELAY); } while (0) // ignore return
#define XOS_MutexRelease(semaphore)    do { xSemaphoreGive(semaphore); } while (0) 

//#define XOS_SemaphoreCreate(semaphore)        do { vSemaphoreCreateBinary(semaphore); assert(semaphore); } while (0)
////#define OS_SemaphoreSignalFromISR(semaphore) do { static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;; xSemaphoreGiveFromISR(semaphore,&xHigherPriorityTaskWoken); portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );} while (0)
//#define XOS_SemaphoreSignalFromISR(semaphore) do { static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE; xSemaphoreGiveFromISR(semaphore,&xHigherPriorityTaskWoken); } while (0)
//#define XOS_SemaphoreWait(semaphore)          do { xSemaphoreTake( semaphore,portMAX_DELAY); } while (0)

#define XOS_FixedQueueCreate(queue,nelements,elemSize)     do { queue = xQueueCreate(nelements,elemSize); assert(queue); } while (0)
#define XOS_FixedQueueSendTimed(queue,pElement,timeout)         xQueueSend( queue, pElement, timeout ) // returns XOS_TIMEOUT_RESULT   if timeout
#define XOS_FixedQueueSend(queue,pElement)                 xQueueSend( queue, pElement, portMAX_DELAY ) // returns XOS_TIMEOUT_RESULT   if timeout
#define XOS_FixedQueueSendFromISR(queue,pElement)          do { static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE; xQueueSendFromISR( queue, pElement, &xHigherPriorityTaskWoken ); } while (0)
#define XOS_FixedQueueSendFromISREx(queue,pElement,rtosReturnedFlags)   xQueueSendFromISR( queue, pElement, rtosReturnedFlags )
#define XOS_FixedQueueReceive(queue,pElement)              do { xQueueReceive( queue, pElement, portMAX_DELAY ); } while (0)
#define XOS_FixedQueueReceiveTimed(queue,pElement,timeout)  xQueueReceive( queue, pElement, timeout ) // returns XOS_TIMEOUT_RESULT   if timeout
#define XOS_TIMEOUT_RESULT 0
#define XOS_FixedQueueReceiveFromISR(queue,pElement,rtosReturnedFlags)      xQueueReceiveFromISR( queue, pElement, rtosReturnedFlags ) // returns XOS_TIMEOUT_RESULT if queue is empty
#define XOS_FixedQueueAvailable(queue)                     uxQueueMessagesWaiting(queue)

// use XOS_Event_t
#define XOS_CreateEvent(event)                    do { vSemaphoreCreateBinary(event); } while (0)
#define XOS_SignalTaskEvent(event,task)               do { xSemaphoreGive(event); } while (0)
#define XOS_SignalTaskEventFromISR(event,task)               fix me
#define XOS_SignalEvent(event)                    do { xSemaphoreGive(event); } while (0)
#define XOS_WaitEvent(event)                      do { xSemaphoreTake( event,portMAX_DELAY); } while (0)
#define XOS_WaitEventTimed(event,timeout)         xSemaphoreTake( event,timeout) // returns non-zero if successful, 0 if failure (timeout)

//
///////////////////// FreeRTOS


#elif defined(NO_RTOS)

//ignore mutexes.
typedef void* XOS_Mutex_t;
typedef void  XOS_TCB_t;

#define XOS_MutexTake(x)
#define XOS_MutexRelease(x)

#define XOS_EnterCritical()
#define XOS_LeaveCritical()

#define XOS_EnterNestableInterrupt()
#define XOS_LeaveNestableInterrupt()

typedef struct  {
    void* head;
    void* tail;
} XOS_FixedQueue_o;
typedef XOS_FixedQueue_o* XOS_FixedQueue_t;

#define XOS_TIMEOUT_RESULT 1

// not implemented, just stubs
#define XOS_FixedQueueCreate(queue,nelements,elemSize)     do { /* not implemented*/ queue = 0; } while (0)
#define XOS_FixedQueueReceiveTimed(queue,pElement,timeout) XOS_TIMEOUT_RESULT
#define XOS_FixedQueueSendTimed(queue, pElement,timeout)        XOS_TIMEOUT_RESULT
#define XOS_FixedQueueSend(queue, pElement,timeout)        XOS_TIMEOUT_RESULT
#define XOS_FixedQueueReceiveFromISR(queue,pElement,rtosReturnedFlags)       XOS_TIMEOUT_RESULT
#define XOS_FixedQueueSendFromISREx(queue,pElement,rtosReturnedFlags)        XOS_TIMEOUT_RESULT
#define XOS_FixedQueueSendFromISR(queue,pElement,rtosReturnedFlags)          XOS_TIMEOUT_RESULT

#elif defined(UCOS1992)

#define XOS_EnterCritical()      OS_CPU_SR  cpu_sr; OS_ENTER_CRITICAL();
#define XOS_LeaveCritical()      OS_EXIT_CRITICAL()
#define XOS_CurrentTCB()         OSTCBCur

#else

#error need to define one of: POWERPAC, FREERTOS, NO_RTOS

#endif

typedef struct {
  XOS_Mutex_t  sem;
  XOS_TCB_t*   pTcb;
  uint32_t  localCounter;
#ifdef DEBUG_TASK_LOCK
  OS_TCB* pTcbDebug;
  uint32_t debugLockCounter;
  uint32_t debugUnlockCounter;
#endif
} XOS_TaskLock_t;


#ifdef __cplusplus
extern "C" {
#endif
void XOS_TaskLockCreate(XOS_TaskLock_t* lock);
void XOS_TaskLockTake(XOS_TaskLock_t* lock);
void XOS_TaskLockRelease(XOS_TaskLock_t* lock);
void XOS_TaskLockDestroy(XOS_TaskLock_t* lock);
void XOS_StackInit(void);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

class XOS_TaskLockable {
public:
    XOS_TaskLockable() {
        XOS_TaskLockCreate(&m_lock);
    }
    ~XOS_TaskLockable() {
        XOS_TaskLockDestroy(&m_lock);
    }
    void Lock() const {
        XOS_TaskLockTake(&m_lock);
    }
    void Unlock() const {
        XOS_TaskLockRelease(&m_lock);
    }
    
private:
    friend class XOS_TaskLock; 
    mutable XOS_TaskLock_t m_lock;
};

class XOS_TaskLockEngage {
public:
  XOS_TaskLockEngage(const XOS_TaskLockable& s) : m_s(s) { m_s.Lock(); }
  ~XOS_TaskLockEngage() { m_s.Unlock(); }
  const XOS_TaskLockable& m_s;
};


#endif // __cplusplus






#else // else not USE_RTOS

typedef int XOS_Mutex_t;

#define XOS_MutexCreate(x)
#define XOS_MutexTake(x)
#define XOS_MutexRelease(x)
#define XOS_ShutDown()
#define XOS_EnterCritical()
#define XOS_LeaveCritical()
#define XOS_TaskLockTake(x)
#define XOS_TaskLockRelease(x)
#define XOS_EnterNestableInterrupt()
#define XOS_LeaveNestableInterrupt()
  
#ifdef __cplusplus
extern "C" {
#endif
uint32_t XOS_GetTime32(void);      // please define based on jiffy timer or whatever in user code
void     XOS_DelayMs(uint32_t ms);
#ifdef __cplusplus
}
#endif

typedef struct {
    uint8_t* buffer;
    uint8_t  elemSize;
    uint8_t  ptrRead;
    uint8_t  ptrWrite;
    uint16_t ptrEnd;
} XOS_FixedQueue_t;

#define XOS_FixedQueueCreate(queue,nelements,elemSize)     do { static uint8_t _buffer[(uint16_t)(nelements)*(elemSize)]; XOS_FixedQueueCreateEx(&queue,(elemSize),(nelements),(_buffer));  } while (0)
#define XOS_FixedQueueSendFromISR(queue,pElement)          do { XOS_FixedQueueSendFromISREx(&queue, (pElement) ); } while (0)
#define XOS_FixedQueueSend(queue,pElement)                 do { XOS_FixedQueueSendEx(&queue, (pElement) ); } while (0) // will suspend the task until it completes
#define XOS_FixedQueueReceive(queue,pElement)              do { XOS_FixedQueueReceiveEx(&queue, (pElement) ); } while (0)
#define XOS_FixedQueueReceiveTimed(queue,pElement,timeout) XOS_FixedQueueReceiveTimedEx(&queue,(pElement),(timeout))
#define XOS_TIMEOUT_RESULT                                 1 

// these functions MUST NOT BE CALLED DIRECTLY. Use the macros above, ALWAYS.
void XOS_FixedQueueCreateEx(XOS_FixedQueue_t* queue,uint8_t nelements,uint8_t elemSize,uint8_t* buffer);
void XOS_FixedQueueSendFromISREx(XOS_FixedQueue_t* queue,void* pElement);
void XOS_FixedQueueSendEx(XOS_FixedQueue_t* queue,void* pElement)   ; 
void XOS_FixedQueueReceiveEx(XOS_FixedQueue_t* queue,void* pElement)  ;
uint8_t XOS_FixedQueueReceiveTimedEx(XOS_FixedQueue_t* queue,void* pElement,uint16_t timeout);



#endif // USE_RTOS

typedef int XOS_Flag_t;

#define XOS_MillisecondElapsedU32(t0)  ((uint32_t)(XOS_GetTime32() - t0))
#define XOS_FlagSet(x) do {XOS_EnterCritical(); x = 1; XOS_LeaveCritical();} while(0)
#define XOS_FlagReset(x) do {XOS_EnterCritical(); x = 0; XOS_LeaveCritical();} while (0)
#define XOS_FlagReadAndClear(x,y) {XOS_EnterCritical(); y = x; x=0; XOS_LeaveCritical();} while (0)

#endif // _OS_H_

