/*
 * bp_messages.h
 *
 *  Created on: Nov 13, 2020
 *      Author: kai
 */

#ifndef INC_BP_MESSAGES_H_
#define INC_BP_MESSAGES_H_

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
//#include "bp_communication.h"

#define BP_MSG_MAX_BYTES  9

/* Generic message definition for receiving messages */
#define BP_MSG_END              0x14F  // End of a message
#define BP_ADD_RADIO_178 		0x178  // Assuming we're a DAB device
#define BP_ADD_RADIO_17C 		0x17C  // Assuming we're a DAB device
#define BP_ADD_RADIO_BUTTON_17D 0x17D  // Radio -> DAB: Which button

enum bp_msg_state_en {
	MSG_UNINITIALIZED = 0,
	MSG_INCOMPLETE,
	MSG_COMPLETE,
	MSG_COMPLETE_RESPONSE, /* This message is flagged as a response to our message */
	MSG_FAIL
};

typedef enum {
	MSG_ERR_NONE = 0,
	MSG_UNKNOWN,
	MSG_ERR_TRANSMISSION
}bp_msg_error;

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
	BP_MSG_BUT_SRC_RELEASED,
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
	BP_MSG_ACK_ACTIVATE,
	BP_MSG_ACK_DEACTIVATE,
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

// Can be combined by OR
enum bp_msg_signal_channel_numbers {
	SIG_FM_I            = 0x00,
	SIG_FM_II           = 0x10,
	SIG_FM_III          = 0x20,
	SIG_FM_T            = 0x30,
	SIG_M	            = 0x40,
	SIG_L               = 0x50,
	SIG_NO_BAND         = 0x70,
	SIG_FM_I_FLASHING   = 0x80,
	SIG_FM_II_FLASHING  = 0x90,
	SIG_FM_III_FLASHING = 0xA0,
	SIG_FM_T_FLASHING   = 0xB0,
	SIG_M_FLASHING	    = 0xC0,
	SIG_L_FLASHING      = 0xD0,

	SIG_CHAN_NONE       = 0x00,
	SIG_CHAN_1          = 0x01,
	SIG_CHAN_2          = 0x02,
	SIG_CHAN_3          = 0x03,
	SIG_CHAN_4          = 0x04,
	SIG_CHAN_5          = 0x05,
	SIG_CHAN_6          = 0x06,
	SIG_CHAN_7          = 0x07,
	SIG_CHAN_1_FLASHING = 0x09,
	SIG_CHAN_2_FLASHING = 0x0A,
	SIG_CHAN_3_FLASHING = 0x0B,
	SIG_CHAN_4_FLASHING = 0x0C,
	SIG_CHAN_5_FLASHING = 0x0D,
	SIG_CHAN_6_FLASHING = 0x0E,
	SIG_CHAN_7_FLASHING = 0x0F

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


bp_msg_error compareMessages(bp_msg_dt * msg1, bp_msg_dt * msg2);
bp_msg_en findMessage(bp_msg_dt * message);

bp_msg_dt buildMessage(uint16_t address, uint8_t dataLen, uint8_t command, uint8_t * data, uint32_t waitMs);
bp_msg_dt buildTextMessage(char * text, uint32_t waitMs);

// in bp_charset.c
void utf2bp(const char * inBuf, uint8_t inBufLen, char * outBuf, uint8_t outBufLen);


#endif /* INC_BP_MESSAGES_H_ */
