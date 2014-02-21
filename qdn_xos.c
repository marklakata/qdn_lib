/// G8 XOS Task Locking
///
/// Sometimes it is necessary to prevent 2 tasks from executing the same code. The
/// TaskLock will only allow one thread to execute AND it will allow the same thread 
/// to repeatedly lock the same lock.
///
/// Usage:
///  void init() {
///    XOS_TaskLockCreate(&lock)
///   ...
///  }
///
///  void foo() {
///    XOS_TaskLockTake(&lock);
///    do stuff
///    XOS_TaskLockRelease(&lock);
///  }
///  void bar() {
///    XOS_TaskLockTake(&lock);
///    do other stuff
///    foo();
///    XOS_TaskLockRelease(&lock);
///  }


#include "qdn_xos.h"
#include <assert.h>

#ifndef USE_RTOS
#error USE_RTOS must be define to compile this source file
#endif

#define INVALID_TCB ((XOS_TCB_t*)0xFFFFFFFF)

void XOS_TaskLockCreate(XOS_TaskLock_t* lock) {
    XOS_MutexCreate(lock->sem); // allow one instance
    lock->pTcb = INVALID_TCB;
    lock->localCounter = 0;
#ifdef DEBUG_TASK_LOCK
    lock->debugLockCounter=0;
    lock->debugUnlockCounter=0;
#endif  
}

void XOS_TaskLockDestroy(XOS_TaskLock_t* lock) {
    XOS_MutexDestroy(lock->sem); // allow one instance
}

void XOS_TaskLockTake(XOS_TaskLock_t* lock) {
  XOS_EnterCritical();
  
  assert(lock->localCounter < 100); // prevent run away same-thread recursion
  
  if (lock->pTcb == XOS_CurrentTCB()) {
    lock->localCounter++;
    XOS_LeaveCritical();
// do nothing if we have already grabbed this mutex in this context.
  } else {
    // first nested locking of the mutex
    XOS_LeaveCritical();
    XOS_MutexTake(lock->sem); // no timeout.
    lock->pTcb = XOS_CurrentTCB();
  }
#ifdef DEBUG_TASK_LOCK
  lock->debugLockCounter++;
#endif  
}

void XOS_TaskLockRelease(XOS_TaskLock_t* lock) {
  XOS_EnterCritical();
#ifdef DEBUG_TASK_LOCK
  lock->debugUnlockCounter++;
#endif  
  if (lock->localCounter > 0) {
    lock->localCounter--;
    XOS_LeaveCritical();
  } else {
//    assert(mm->sem.OSSemCnt < 1); // double check for random badness
#ifdef DEBUG_TASK_LOCK
    lock->pTcbDebug = lock->pTcb;
#endif    
    lock->pTcb = INVALID_TCB;
    XOS_LeaveCritical();
    XOS_MutexRelease(lock->sem);
  } 
}


#pragma section = "ABT_STACK"


void XOS_StackInit(void) {

#if   __CORE__ == __ARM5E__   // STR912 only
    
//  Initialize the Stack with the 0xCD pattern, so that we can detect the size of the stack that is actually used.
//  Of course, this doesn't work if 0xCDCDCDCD is actually a real data word...
    
    uint32_t* ptr = (uint32_t*) __segment_begin("ABT_STACK");
    if (ptr) {
        while(ptr != (uint32_t*)  __segment_end("ABT_STACK")) {
          *ptr = 0xCDCDCDCD;
          ptr++;
        }
    }
#endif
    
}
    
