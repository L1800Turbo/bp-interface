#ifndef __RINGBUFFER_H
#define __RINGBUFFER_H

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "bp_messages.h"

typedef enum {
	RINGBUF_OK = 0,
	RINGBUF_NO_DATA,
	RINGBUF_WRITE_BUSY
}ringbuf_status_en;

/* Whether we want to move to the next item or keep the pointer on this one */
typedef enum {
	RINGBUF_NEXT_ITEM = 0,
	RINGBUF_KEEP_ITEM
}ringbuf_next_en;

typedef struct {
	uint8_t writeInd;
	uint8_t readInd;	
	
	uint8_t bufSize;
	
	bp_msg_dt * msg;

	ringbuf_status_en busyIndicator;

	uint32_t waitTime;
} ringbuf_dt;

ringbuf_dt * ringInit(uint8_t size);

void ringClear(ringbuf_dt * buf);

ringbuf_status_en ringReadAvailable(ringbuf_dt * buf);

/* Insert a complete message */
void ringAdd(ringbuf_dt * buf, bp_msg_dt msg);

/* Get a complete message */
ringbuf_status_en ringGet(ringbuf_dt * buf, bp_msg_dt * msg, ringbuf_next_en nextItem);

void ringNextWriteInd(ringbuf_dt * buf);
void ringNextReadInd(ringbuf_dt * buf);

#endif
