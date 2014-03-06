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

/// XOS Task Locking
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
    
