#include "qdn_fifo.h"

extern "C"
int QDN_FifoEnqueueUInt8(QDN_FIFO_t* fifo0, uint8_t value)
{
	QDN_FifoBase<uint8_t>* f = (QDN_FifoBase<uint8_t>*)fifo0;
	return f->Enqueue(value);
}

extern "C"
int QDN_FifoDequeueUInt8(QDN_FIFO_t* fifo0, uint8_t* value)
{
	QDN_FifoBase<uint8_t>* f = (QDN_FifoBase<uint8_t>*)fifo0;
	return f->Dequeue(*value);
}
