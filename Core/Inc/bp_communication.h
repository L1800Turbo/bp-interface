/*
 * bp_communication.h
 *
 *  Created on: Nov 13, 2020
 *      Author: kai
 */

#ifndef INC_BP_COMMUNICATION_H_
#define INC_BP_COMMUNICATION_H_

/* Includes ------------------------------------------------------------------*/
#include <bp_display.h>
#include <stdio.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "bp_messages.h"
#include "main.h"
#include "ringbuffer.h"

enum bp_comm_state
{
  BP_UNINITIALIZED = 0,
  BP_INIT_4800,		/* Initialization after turning on radio with 4800 Baud       */
  BP_INIT_9600,		/* Second initialization step after turning on with 9600 Baud */
  BP_IDLE,			/* Idle state after initialization, DAB not active            */
  BP_SEND_WAIT,
  BP_SEND
};

enum bp_msg_rcv_pos {
	MSG_ADDRESS = 0,
	MSG_LENGTH,
	MSG_CMD,
	MSG_DATA
};

enum bp_msg_response_en {
	MSG_RESPONSE_INACTIVE = 0,
	MSG_RESPONSE_ACTIVE
};

enum bp_msg_direction_en {
	MSG_DIRECTION_RECEIVE = 0,
	MSG_DIRECTION_SEND
};

//#define BP_MAX_MESSAGES  10

typedef struct {
	uint16_t rx_data[2];
	uint16_t rx_data_debug[2]; // TODO über Define ausblenden

	uint16_t tx_data[20];
	uint8_t	 tx_data_pos;
}bp_uart_data_t;

typedef struct {
	enum bp_msg_response_en responseActive;	/* If we currently respond to received bytes (usually when in receive mode */
	uint8_t currentDataByte;	            /* In which position of the data byte are we */
	enum bp_msg_rcv_pos receivePosition;
	enum bp_msg_direction_en direction;		/* If we currently wait for messages (rcv) or send */

	ringbuf_dt * readBuf;

	ringbuf_dt * debugBuffer; // TODO: durch DEFINE ausbauen..

	bp_msg_dt curReadMsg;					/* The message buffer we are currently writing in during receivement */

	ringbuf_dt * writeBuf; // TODO: msgRing oder so? Buf ist doof, rx und tx buffer haben wir ja schon...

	uint32_t msgReceivedTime;			    /* Timestamp when last message was received completely */
	uint32_t waitTickMs;					/* Store timestamp if further waiting is needed */

	bp_uart_data_t uart;					/* Contains UART buffers */

	//bp_msg_dt checkMsg;

	//bp_msg_dt messages[BP_MAX_MESSAGES];	/* Buffer of messages*/
	//uint8_t currentMessagePos;				/* The current position in the buffer to write   */
	//uint8_t processMessagePos;				/* The current position in the buffer to work on */
}bp_msg_state_dt;

void bpCommInit(void);
void bpCommTasks(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void setSendWait(void);

bp_msg_error processBpMsg(bp_msg_dt * message);
ringbuf_status_en sendRingMessage(bp_msg_state_dt * msgState, ringbuf_next_en nextItem);

void bpDebugPrint(void);

#endif /* INC_BP_COMMUNICATION_H_ */
