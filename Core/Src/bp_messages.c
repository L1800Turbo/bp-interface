#include "bp_messages.h"

const bp_msg_dt bpMessages[BP_MSG_SIZE] = {
		{0}, /* BP_MSG_UNKNOWN */

		/* Messages to be received */
		{.address = 0x175, .command = 0x48}, /* BP_MSG_INIT_STATE */

		/* Buttons */
		{.address = 0x17D, .command = 0x01}, /* BP_MSG_BUT_1        */
		{.address = 0x17D, .command = 0x02}, /* BP_MSG_BUT_2        */
		{.address = 0x17D, .command = 0x03}, /* BP_MSG_BUT_3        */
		{.address = 0x17D, .command = 0x04}, /* BP_MSG_BUT_4        */
		{.address = 0x17D, .command = 0x05}, /* BP_MSG_BUT_5        */
		{.address = 0x17D, .command = 0x06}, /* BP_MSG_BUT_6        */
		{.address = 0x178, .command = 0x0B}, /* BP_MSG_BUT_SRC,  Andere Adresse! */
		{.address = 0x17D, .command = 0x0F}, /* BP_MSG_BUT_DOWN     */
		{.address = 0x17D, .command = 0x10}, /* BP_MSG_BUT_UP       */
		{.address = 0x17D, .command = 0x12}, /* BP_MSG_BUT_LEFT     */
		{.address = 0x17D, .command = 0x13}, /* BP_MSG_BUT_RIGHT    */
		{.address = 0x17D, .command = 0x14}, /* BP_MSG_BUT_DSC      */
		{.address = 0x17D, .command = 0x15}, /* BP_MSG_BUT_LD       */
		{.address = 0x17D, .command = 0x16}, /* BP_MSG_BUT_AUD      */
		{.address = 0x178, .command = 0x22}, /* BP_MSG_BUT_SRC_RELEASED, Knopf los gelassen 0x178 */
		{.address = 0x17C, .command = 0x22}, /* BP_MSG_BUT_RELEASED_17C, Knopf los gelassen 0x17C */
		{.address = 0x17D, .command = 0x22}, /* BP_MSG_BUT_RELEASED_17D, Knopf los gelassen 0x17D */
		{.address = 0x17D, .command = 0x23}, /* BP_MSG_BUT_SCA      */
		{.address = 0x17D, .command = 0x25}, /* BP_MSG_BUT_PS       */
		{.address = 0x17D, .command = 0x26}, /* BP_MSG_BUT_MIX      */
		{.address = 0x17D, .command = 0x27}, /* BP_MSG_BUT_GEO      */
		{.address = 0x17C, .command = 0x2A}, /* BP_MSG_BUT_TA,  Andere Adresse! */
		{.address = 0x17D, .command = 0x2B}, /* BP_MSG_BUT_lo       */
		{.address = 0x17D, .command = 0x2C}, /* BP_MSG_BUT_AF       */
		{.address = 0x17D, .command = 0x2D}, /* BP_MSG_BUT_RM       */
		{.address = 0x17D, .command = 0x2E}, /* BP_MSG_BUT_dx       */
		{.address = 0x17D, .command = 0x31}, /* BP_MSG_BUT_FM       */
		{.address = 0x17D, .command = 0x38}, /* BP_MSG_BUT_TS       */
		{.address = 0x17D, .command = 0x3F}, /* BP_MSG_BUT_dB       */
		{.address = 0x17D, .command = 0x60}, /* BP_MSG_BUT_VOL_MIN  */
		{.address = 0x17D, .command = 0x61}, /* BP_MSG_BUT_VOL_PLUS */

		{.address = 0x178, .command = 0x14, .dataLen = 2, .data = {0x80, 0x39}}, /* BP_MSG_LEAVE_VOL  */
		{.address = 0x178, .command = 0x14, .dataLen = 2, .data = {0x01, 0x3A}}, /* BP_MSG_ENTER_GEO_MIX  */
		{.address = 0x178, .command = 0x14, .dataLen = 2, .data = {0x80, 0x3A}}, /* BP_MSG_LEAVE_GEO_MIX  */
		{.address = 0x178, .command = 0x14, .dataLen = 2, .data = {0x01, 0x3B}}, /* BP_MSG_ENTER_AUD  */
		{.address = 0x178, .command = 0x14, .dataLen = 2, .data = {0x80, 0x3B}}, /* BP_MSG_LEAVE_AUD  */
		{.address = 0x178, .command = 0x14, .dataLen = 2, .data = {0x01, 0x3C}}, /* BP_MSG_ENTER_FAD (inside GEO menu)  */
		{.address = 0x178, .command = 0x14, .dataLen = 2, .data = {0x80, 0x3C}}, /* BP_MSG_LEAVE_FAD (inside GEO menu)  */
		{.address = 0x178, .command = 0x14, .dataLen = 2, .data = {0x80, 0x3F}}, /* BP_MSG_LEAVE_MUTE */

		/* Messages to be sent */
		{.address = 0x175, .command = 0x30, .dataLen = 1, .data[0] = 0x01,      .waitAfter_ms = 60}, /* BP_MSG_ACK_ACTIVATE   */
		{.address = 0x175, .command = 0x30, .dataLen = 1, .data[0] = 0x80,      .waitAfter_ms = 60}, /* BP_MSG_ACK_DEACTIVATE */
		{.address = 0x175, .command = 0x30, .dataLen = 2, .data = {0x09, 0x0F}, .waitAfter_ms = 10}, /* BP_MSG_ACK_BUT_DOWN   */
		{.address = 0x175, .command = 0x30, .dataLen = 2, .data = {0x09, 0x10}, .waitAfter_ms = 10}, /* BP_MSG_ACK_BUT_UP     */
		{.address = 0x175, .command = 0x30, .dataLen = 2, .data = {0x09, 0x12}, .waitAfter_ms = 10}, /* BP_MSG_ACK_BUT_LEFT   */
		{.address = 0x175, .command = 0x30, .dataLen = 2, .data = {0x09, 0x13}, .waitAfter_ms = 10}, /* BP_MSG_ACK_BUT_RIGHT  */
		{.address = 0x175, .command = 0x30, .dataLen = 2, .data = {0x09, 0x16}, .waitAfter_ms = 10}, /* BP_MSG_ACK_BUT_AUD    */
		{.address = 0x175, .command = 0x30, .dataLen = 2, .data = {0x09, 0x22}, .waitAfter_ms = 10}, /* BP_MSG_ACK_REASED_17D */
		{.address = 0x175, .command = 0x30, .dataLen = 2, .data = {0x09, 0x27}, .waitAfter_ms = 10}, /* BP_MSG_ACK_BUT_GEO    */
		{.address = 0x175, .command = 0x30, .dataLen = 2, .data = {0x09, 0x3F}, .waitAfter_ms = 10}, /* BP_MSG_ACK_BUT_dB     */
		{.address = 0x175, .command = 0x30, .dataLen = 2, .data = {0x09, 0x60}, .waitAfter_ms = 10}, /* BP_MSG_ACK_VOL_MIN    */
		{.address = 0x175, .command = 0x30, .dataLen = 2, .data = {0x09, 0x61}, .waitAfter_ms = 10}, /* BP_MSG_ACK_VOL_PLUS   */

		{.address = 0x175, .command = 0x2A, .dataLen = 1, .data[0] = 0x01, .waitAfter_ms = 20}, /* BP_MSG_TA_ACTIVE         */
		{.address = 0x175, .command = 0x2A, .dataLen = 1, .data[0] = 0x80, .waitAfter_ms = 10}, /* BP_MSG_TA_INACTIVE       */
		{.address = 0x175, .command = 0x3F, .dataLen = 1, .data[0] = 0x01, .waitAfter_ms = 10}, /* BP_MSG_STATION_NOT_FOUND */
		{.address = 0x175, .command = 0x3F, .dataLen = 1, .data[0] = 0x80, .waitAfter_ms = 10}, /* BP_MSG_STATION_FOUND     */
		{.address = 0x175, .command = 0x50								 , .waitAfter_ms = 20}, /* BP_MSG_TEXT              */
		{.address = 0x175, .command = 0x56, .dataLen = 1, .data[0] = 0x10, .waitAfter_ms = 20}, /* BP_MSG_SIGNAL_TA_ON      */
		{.address = 0x175, .command = 0x57, .dataLen = 1, .data[0] = 0x10, .waitAfter_ms = 20}, /* BP_MSG_SIGNAL_TA_OFF     */

		{.address = 0x175, .command = 0x70, .dataLen = 1, .data[0] = (SIG_NO_BAND|SIG_CHAN_NONE), .waitAfter_ms = 20}, /* BP_MSG_SIGNAL_CHAN */
		/* declaration according enum bp_msg_signal_channel_numbers */

		{.address = 0x175, .command = 0x72, .dataLen = 1, .data[0] = 0x00, .waitAfter_ms = 20}, /* BP_MSG_SIGNAL_TS_I       */
		{.address = 0x175, .command = 0x72, .dataLen = 1, .data[0] = 0x10, .waitAfter_ms = 20}, /* BP_MSG_SIGNAL_TS_II      */
		{.address = 0x175, .command = 0x72, .dataLen = 1, .data[0] = 0x30, .waitAfter_ms = 20}, /* BP_MSG_SIGNAL_TS_T       */
};

bp_msg_error compareMessages(bp_msg_dt * msg1, bp_msg_dt * msg2)
{
	if(msg1->address == msg2->address && msg1->dataLen == msg2->dataLen && msg1->command == msg2->command)
	{
		for(uint8_t i=0; i<msg1->dataLen; i++)
		{
			if(msg1->data[i] != msg2->data[i])
			{
				return MSG_ERR_TRANSMISSION; // Return that there seems to be something wrong with the transmission
			}
		}

		return MSG_ERR_NONE;
	}
	return MSG_ERR_TRANSMISSION;
}

bp_msg_en findMessage(bp_msg_dt * message)
{
	uint8_t found = 0;

	for(bp_msg_en i=1; i<BP_MSG_SIZE; i++) // start with 1, 0 is "unknown message"
	{
		if(message->address == bpMessages[i].address)
		{
			if(message->command == bpMessages[i].command)
			{
				found = 1;
				// TODO: should just jump over if datalen <1 or not mentioned -> Testen
				for(uint8_t j=0; j<message->dataLen; j++) // if dataLen is mentioned and we need to compare data
				{
					if(message->data[j] != bpMessages[i].data[j])
					{
						//return BP_MSG_UNKNOWN;
						found = 0;
						break;
					}
					else
					{
						found = 1;
					}
				}
				if(found == 1)
				{
					return i;
				}
			}
		}
	}

	return BP_MSG_UNKNOWN;
}


/* Create a custom message (mostly for unknown msgs) */
bp_msg_dt buildMessage(uint16_t address, uint8_t dataLen, uint8_t command, uint8_t * data, uint32_t waitMs)
{
	bp_msg_dt msg;

	msg.address = address;
	msg.dataLen = dataLen;
	msg.command = command;
	memcpy(msg.data, data, dataLen);

	msg.timeStamp_ms = HAL_GetTick();
	msg.waitAfter_ms = waitMs;
	msg.messageState = MSG_COMPLETE;

	return msg;
}
