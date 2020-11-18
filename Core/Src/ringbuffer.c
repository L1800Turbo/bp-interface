#include "ringbuffer.h"

ringbuf_dt * ringInit(uint8_t size)
{
	ringbuf_dt * buf = (ringbuf_dt*) malloc(sizeof(ringbuf_dt));
	
	buf->writeInd = 0;
	buf->readInd  = 0;
	
	buf->bufSize  = size;
	
	buf->msg = (bp_msg_dt*) malloc(sizeof(bp_msg_dt) * (size + 1));
	
	return buf; /* Return ptr to buffer */
}

void ringClear(ringbuf_dt * buf)
{
	buf->writeInd = 0;
	buf->readInd  = 0;
}

void ringNextWriteInd(ringbuf_dt * buf)
{
	buf->writeInd = (buf->writeInd+1) % (buf->bufSize /*+ 1*/);
}

void ringNextReadInd(ringbuf_dt * buf)
{
	buf->readInd = (buf->readInd+1) % (buf->bufSize /*+ 1*/);
}

ringbuf_status_en ringReadAvailable(ringbuf_dt * buf)
{
	if (buf->writeInd != buf->readInd)
	{
		return RINGBUF_OK;
	}
	return RINGBUF_NO_DATA;
}

void ringAdd(ringbuf_dt * buf, bp_msg_dt msg)
{
	while(buf->busyIndicator == RINGBUF_WRITE_BUSY);
	
	buf->busyIndicator = RINGBUF_WRITE_BUSY; // TODO: Zeit messen, wie lange das max. dauert
	if(buf)
	{
		/* Place msg into buffer */
		buf->msg[buf->writeInd] = msg;
		
		/* Next index if wanted */ // TODO: hat das mit dem ++ nicht nicht funktioniert?
		ringNextWriteInd(buf);
		
		/* After whole ring no reading? Add one to read index */
		if(buf->readInd == buf->writeInd)
		{
			buf->readInd = (buf->readInd+1) % (buf->bufSize /*+ 1*/);
		}
	}
	buf->busyIndicator = RINGBUF_OK;
}

ringbuf_status_en ringGet(ringbuf_dt * buf, bp_msg_dt * msg, ringbuf_next_en nextItem)
{
	if(buf)
	{
		if(buf->readInd == buf->writeInd) /* Mustn't read from a position where it is written in */
		{
			return RINGBUF_NO_DATA;
		}
		else
		{
			*msg = buf->msg[buf->readInd];
			
			if(nextItem == RINGBUF_NEXT_ITEM)
			{
				/* Inc read index */
				buf->readInd = (buf->readInd+1) % (buf->bufSize);
			}
			
			return RINGBUF_OK;
		}
	}
	else
	{
		return RINGBUF_NO_DATA;
	}
}
