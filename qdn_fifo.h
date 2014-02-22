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
	volatile ELEMENT* const (&buffer);
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
