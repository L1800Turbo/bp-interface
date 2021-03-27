/*
 * Si46xx_firmware_transfer.c
 *
 *  Created on: Mar 26, 2021
 *      Author: kai
 */

#include "Si46xx.h"

firmwareBuffer_dt * fwBufferInit(uint32_t size)
{
	firmwareBuffer_dt * buf = (firmwareBuffer_dt*) malloc(sizeof(firmwareBuffer_dt));

	buf->writeInd = 0;
	buf->readInd  = 0;

	buf->bufSize  = size;
	buf->data = (uint8_t*) malloc(sizeof(uint8_t) * (size + 1));

	return buf; /* Return ptr to buffer */
}

void fwBufferClear(ringbuf_dt * buf)
{
	buf->writeInd = 0;
	buf->readInd  = 0;
}

uint32_t fwBufferCurrentSize(firmwareBuffer_dt * buf)
{
	if(buf->writeInd >= buf->readInd)
	{
		return buf->writeInd - buf->readInd;
	}
	else
	{
		return buf->bufSize - buf->writeInd  - buf->readInd; // TODO völlig ungetestet!!!
	}
}


firmwareBuffer_state_dt fwBufferGet(firmwareBuffer_dt * buf, uint8_t * bufPtr)
{
	if(buf)
	{
		if(buf->readInd == buf->writeInd) /* Mustn't read from a position where it is written in */
		{
			return FWBUF_NO_DATA;
		}
		else
		{
			*bufPtr = buf->data[buf->readInd];

			buf->readInd = (buf->readInd + 1) % buf->bufSize;

			return FWBUF_OK;
		}
	}

	return FWBUF_NO_DATA;
}

void fwBufferWrite(firmwareBuffer_dt * buf, uint8_t * bufPtr, uint32_t size)
{
	if(buf)
	{
		while(size--)
		{
			buf->data[buf->writeInd] = *bufPtr;

			buf->writeInd = (buf->writeInd+1) % (buf->bufSize);
			bufPtr++;
		}
	}
}



