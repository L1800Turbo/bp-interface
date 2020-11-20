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

// Can be combined by OR
enum bp_msg_signal_channel_numbers {
	SIG_FM_I   = 0x00,
	SIG_FM_II  = 0x10,
	/*
			 * Bit configuration for data[0]:
			 * Upper 4 bits: Frequency/saved range
			 *  0	FM I
			 *  1	FM II
			 *  2	FM III
			 *  3	FM T
			 *  4	M
			 *  5	L
			 *  6	M (like 4)
			 *  7	-none-
			 *  8	FM I (flashing)
			 *  9	FM II (flashing)
			 *  A	FM III (flashing)
			 *  B	FM T (flashing)
			 *  C	M (flashing)
			 *  D	L (flashing)
			 *  E	M (flashing, like C)
			 *  F	-none-
			 *
			 * Lower 4 bits: Channel numbers
			 * 0	Numbers off
			 * 1	"1"
			 * 2	"2"
			 * 3	"3"
			 * 4	"4"
			 * 5	"5"
			 * 6	"6"
			 * 7	"7" (although no 7 on keyboard)
			 * 8	Numbers off (like 0)
			 * 9	"1" (flashing)
			 * A	"2" (flashing)
			 * B	"3" (flashing)
			 * C	"4" (flashing)
			 * D	"5" (flashing)
			 * E	"6" (flashing)
			 * F	"7" (flashing)
			 *
			 * TODO: Enum machen?
			 */
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
