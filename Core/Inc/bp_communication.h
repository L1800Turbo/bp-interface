/*
 * bp_communication.h
 *
 *  Created on: Nov 13, 2020
 *      Author: kai
 */

#ifndef INC_BP_COMMUNICATION_H_
#define INC_BP_COMMUNICATION_H_

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "bp_messages.h"
#include "main.h"
#include "ringbuffer.h"

enum bp_comm_state {

  BP_UNINITIALIZED = 0,
  BP_INIT_4800,		/* Initialization after turning on radio with 4800 Baud       */
  BP_INIT_9600,		/* Second initialization step after turning on with 9600 Baud */
  BP_IDLE,			/* Idle state after initialization, DAB not active            */
  //BP_ACTIVATE,		/* Send initial parameters after activation by radio          */
  BP_SEND_WAIT,
  BP_SEND,
  BP_RUNNING,
  BP_SEARCH_PROGRAM, // oder so ähnlich...
  BP_MENU,
  BP_DEACTIVATE
};

enum bp_msg_rcv_pos {
	MSG_ADDRESS = 0,
	MSG_LENGTH,
	MSG_CMD,
	MSG_DATA
};

typedef enum {
	MSG_ERR_NONE = 0,
	MSG_UNKNOWN,
	MSG_ERR_TRANSMISSION
}bp_msg_error;

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


/* Message definition */
typedef enum {
	BP_MSG_UNKNOWN = 0,

	/* Messages to be received */
	BP_MSG_INIT_STATE,

	BP_MSG_BUT_1,
	BP_MSG_BUT_2,
	BP_MSG_BUT_3,
	BP_MSG_BUT_4,
	BP_MSG_BUT_5,
	BP_MSG_BUT_6,
	BP_MSG_BUT_SRC,
	BP_MSG_BUT_DOWN,
	BP_MSG_BUT_UP,
	BP_MSG_BUT_LEFT,
	BP_MSG_BUT_RIGHT,
	BP_MSG_BUT_DSC,
	BP_MSG_BUT_LD,
	BP_MSG_BUT_AUD,
	BP_MSG_BUT_RELEASED_178,
	BP_MSG_BUT_RELEASED_17C,
	BP_MSG_BUT_RELEASED_17D,
	BP_MSG_BUT_SCA,
	BP_MSG_BUT_PS,
	BP_MSG_BUT_MIX,
	BP_MSG_BUT_GEO,
	BP_MSG_BUT_TA,
	BP_MSG_BUT_lo,
	BP_MSG_BUT_AF,
	BP_MSG_BUT_RM,
	BP_MSG_BUT_dx,
	BP_MSG_BUT_FM,
	BP_MSG_BUT_TS,
	BP_MSG_BUT_dB,
	BP_MSG_BUT_VOL_MIN,
	BP_MSG_BUT_VOL_PLUS,

	BP_MSG_LEAVE_VOL,
	BP_MSG_ENTER_AUD,
	BP_MSG_LEAVE_AUD,
	BP_MSG_LEAVE_MUTE,

	/* Messages to be sent */
	BP_MSG_ACK_BUT_DOWN,
	BP_MSG_ACK_BUT_UP,
	BP_MSG_ACK_BUT_LEFT,
	BP_MSG_ACK_BUT_RIGHT,
	BP_MSG_ACK_BUT_AUD,
	BP_MSG_ACK_REASED_17D,
	BP_MSG_ACK_BUT_GEO,
	BP_MSG_ACK_BUT_dB,
	BP_MSG_ACK_VOL_MIN,
	BP_MSG_ACK_VOL_PLUS,

	BP_MSG_TA_ACTIVE,
	BP_MSG_TA_INACTIVE,
	BP_MSG_STATION_NOT_FOUND,
	BP_MSG_STATION_FOUND,
	BP_MSG_TEXT,
	BP_MSG_SIGNAL_TA_ON,
	BP_MSG_SIGNAL_TA_OFF,

	BP_MSG_SIGNAL_CHAN,

	BP_MSG_SIGNAL_TS_I,
	BP_MSG_SIGNAL_TS_II,
	BP_MSG_SIGNAL_TS_T,

	BP_MSG_SIZE
}bp_msg_en;


void bpCommInit(void);
void bpCommTasks(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

bp_msg_error compareMessages(bp_msg_dt * msg1, bp_msg_dt * msg2);
bp_msg_en findMessage(bp_msg_dt * message);

bp_msg_error processBpMsg(bp_msg_dt * message);

bp_msg_dt buildMessage(uint16_t address, uint8_t dataLen, uint8_t command, uint8_t * data, uint32_t waitMs);
bp_msg_dt buildTextMessage(char * text, uint32_t waitMs);

ringbuf_status_en sendRingMessage(bp_msg_state_dt * msgState, ringbuf_next_en nextItem);

void bpDebugPrint(void);

#endif /* INC_BP_COMMUNICATION_H_ */
