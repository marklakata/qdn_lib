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

#ifndef _QDN_FIFO_H
#define _QDN_FIFO_H

#include "qdn_cpu.h"

#ifdef __cplusplus

template <class ELEMENT>
class QDN_FifoBase {
private:
	QDN_FifoBase(){} // abstract class

protected:
	QDN_FifoBase(volatile ELEMENT* buffer0, uint32_t size0)
		: buffer(buffer0)
		,wptr(0)
		,rptr(0)
		,size(size0)
		,overflows(0)
	{
	}
public:
	int Enqueue(ELEMENT e)
	{
		uint32_t next = wptr +1;
		if (next >= size) next=0;
		if (rptr != next)
		{
			buffer[wptr] = e;
			wptr = next;
			return 0;
		} else {
			return -1;
		}
	}
	int Dequeue(ELEMENT& e)
	{
		uint32_t next = rptr +1;
		if (next >= size) next=0;
		if (rptr != wptr)
		{
			e = buffer[rptr];
			rptr = next;
			return 0;
		} else {
			return -1;
		}
	}
	uint32_t Count(void) {
		volatile int32_t c = wptr;
		c -= rptr;
		if (c < 0) c += size;
		return c;
	}

private:
	volatile ELEMENT *buffer;
	volatile uint32_t wptr;
	volatile uint32_t rptr;
public:
	const uint32_t size;
	int overflows;
};

template <class ELEMENT, int QUEUE_SIZE>
class QDN_Fifo : public QDN_FifoBase<ELEMENT> {
public:
	QDN_Fifo()
		:QDN_FifoBase<ELEMENT>(buffer_allocation,QUEUE_SIZE)
	{

	}

private:
	volatile ELEMENT buffer_allocation[QUEUE_SIZE];
};
#endif

typedef void* QDN_FIFO_t;
// C interface
// Constructor must be made in C++ space

QDN_EXTERN_C
int QDN_FifoEnqueue(QDN_FIFO_t* fifo, uint8_t value);
int QDN_FifoDequeue(QDN_FIFO_t* fifo, uint8_t* value);
QDN_END_EXTERN_C

#endif
