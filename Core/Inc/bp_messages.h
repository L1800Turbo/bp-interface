/*
 * bp_messages.h
 *
 *  Created on: Nov 13, 2020
 *      Author: kai
 */

#ifndef INC_BP_MESSAGES_H_
#define INC_BP_MESSAGES_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#define BP_MSG_MAX_BYTES  9

/* Generic message definition for receiving messages */
#define BP_MSG_END              0x14F  // End of a message
#define BP_ADD_RADIO_178 		0x178  // Assuming we're a DAB device
#define BP_ADD_RADIO_17C 		0x17C  // Assuming we're a DAB device
#define BP_ADD_RADIO_BUTTON_17D 0x17D  // Radio -> DAB: Which button

enum bp_msg_state_en {
	MSG_INCOMPLETE = 0,
	MSG_COMPLETE,
	MSG_COMPLETE_RESPONSE /* This message is flagged as a response to our message */
};

typedef struct {
  uint32_t timeStamp_ms;		            /* Timestamp of the message beginning or waiting time */
  uint32_t waitAfter_ms;
  uint16_t address;
  uint8_t dataLen;
  uint8_t command;
  //uint8_t dataLenUsed; // if data length is transferred with message (DAB communication)
  uint8_t data[BP_MSG_MAX_BYTES];
  enum bp_msg_state_en messageState;	    /* Do we have a complete message? */
}bp_msg_dt;


void utf2bp(const char * inBuf, uint8_t inBufLen, char * outBuf, uint8_t outBufLen);


#endif /* INC_BP_MESSAGES_H_ */
