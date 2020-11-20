/*
 * bp_communication.c
 *
 *  Created on: Nov 13, 2020
 *      Author: kai
 */

#include "bp_communication.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

enum bp_comm_state bpCommState = BP_UNINITIALIZED;

bp_msg_state_dt bpMsgState;
#ifdef DAB_DEBUG_ACTIVE
bp_msg_state_dt bpMsgState_DAB; /* For debugging and listening between DAB and radio */
#endif

static const bp_msg_dt bpMessages[BP_MSG_SIZE] = {
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
		{.address = 0x178, .command = 0x22}, /* BP_MSG_BUT_RELEASED_178, Knopf los gelassen 0x178 */
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
		{.address = 0x178, .command = 0x14, .dataLen = 2, .data = {0x80, 0x3B}}, /* BP_MSG_LEAVE_AUD  */
		{.address = 0x178, .command = 0x14, .dataLen = 2, .data = {0x80, 0x3F}}, /* BP_MSG_LEAVE_MUTE */

		/* Messages to be sent */
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
		{.address = 0x175, .command = 0x2A, .dataLen = 1, .data[0] = 0x80, .waitAfter_ms = 20}, /* BP_MSG_TA_INACTIVE       */
		{.address = 0x175, .command = 0x3F, .dataLen = 1, .data[0] = 0x01, .waitAfter_ms = 20}, /* BP_MSG_STATION_NOT_FOUND */
		{.address = 0x175, .command = 0x3F, .dataLen = 1, .data[0] = 0x80, .waitAfter_ms = 20}, /* BP_MSG_STATION_FOUND     */
		{.address = 0x175, .command = 0x50								 , .waitAfter_ms = 20}, /* BP_MSG_TEXT              */
		{.address = 0x175, .command = 0x56, .dataLen = 1, .data[0] = 0x10, .waitAfter_ms = 20}, /* BP_MSG_SIGNAL_TA_ON      */
		{.address = 0x175, .command = 0x57, .dataLen = 1, .data[0] = 0x10, .waitAfter_ms = 20}, /* BP_MSG_SIGNAL_TA_OFF     */

		{.address = 0x175, .command = 0x70, .dataLen = 1, .data[0] = 0x70, .waitAfter_ms = 20}, /* BP_MSG_SIGNAL_CHAN -> data[0]&1...6 -> Saved channel no */

		{.address = 0x175, .command = 0x72, .dataLen = 1, .data[0] = 0x00, .waitAfter_ms = 20}, /* BP_MSG_SIGNAL_TS_I       */
		{.address = 0x175, .command = 0x72, .dataLen = 1, .data[0] = 0x10, .waitAfter_ms = 20}, /* BP_MSG_SIGNAL_TS_II      */
		{.address = 0x175, .command = 0x72, .dataLen = 1, .data[0] = 0x30, .waitAfter_ms = 20}, /* BP_MSG_SIGNAL_TS_T       */
};

void bpCommInit(void)
{
	/* Initialize ring buffers */
	bpMsgState.writeBuf = ringInit(30);
	bpMsgState.readBuf  = ringInit(20);

#ifdef DAB_DEBUG_ACTIVE
	bpMsgState_DAB.readBuf  = ringInit(20);
#endif

	bpMsgState.direction = MSG_DIRECTION_RECEIVE;

	/* Initialize interrupt receivement for UART */
	HAL_UART_Receive_IT(&huart2, (uint8_t*) bpMsgState.uart.rx_data, 1);
#ifdef DAB_DEBUG_ACTIVE
	HAL_UART_Receive_IT(&huart3, (uint8_t*) bpMsgState.uart.rx_data_debug, 1);
#endif
}


uint8_t idle_zwischen = 0; // TODO raus

void bpCommTasks(void)
{
	bp_msg_dt msg;

	switch(bpCommState)
	{
		case BP_UNINITIALIZED:

			bpCommState = BP_INIT_4800; // just skip directly to 4800baud init
			// Later: activation when 5V from radio o.æ.
			//break;

		case BP_INIT_4800:

			//if(bpMsgState.processMessagePos != bpMsgState.currentMessagePos) TODO: weg wenn Ringbuffer geht
			if(ringGet(bpMsgState.readBuf, &msg, RINGBUF_NEXT_ITEM) == RINGBUF_OK)
			{
				// TODO: Werte irgendwie ablegen, soweit sinnvoll

				/* after cmd 0x48 with data 0x02 radio switches to 9600 baud */
				if(msg.command == 0x48 && msg.data[0] == 0x02)
				{
					HAL_UART_DeInit(&huart2);
					//HAL_UART_Abort(&huart2);

					huart2.Init.BaudRate = 9600;
					if (HAL_UART_Init(&huart2) != HAL_OK)
					{
						Error_Handler();
					}
					__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
					HAL_UART_Receive_IT(&huart2, (uint8_t*) bpMsgState.uart.rx_data, 1);

#ifdef DAB_DEBUG_ACTIVE
					HAL_UART_DeInit(&huart3);

					huart3.Init.BaudRate = 9600;
					if (HAL_UART_Init(&huart3) != HAL_OK)
					{
						Error_Handler();
					}
					__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
					HAL_UART_Receive_IT(&huart3, (uint8_t*) bpMsgState.uart.rx_data_debug, 1);
#endif

					bpCommState = BP_INIT_9600;
					printf("Finished init 4800\r\n");
					//while(((USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData))->TxState!=0);
					//CDC_Transmit_FS(buffer_tmp, strlen(buffer_tmp));
				}
				// TODO: done by ringget oben
				//bpMsgState.processMessagePos = (bpMsgState.processMessagePos+1) % (BP_MAX_MESSAGES);
		    }
			break;

		case BP_INIT_9600:
			//if(bpMsgState.processMessagePos != bpMsgState.currentMessagePos)
			if(ringGet(bpMsgState.readBuf, &msg, RINGBUF_NEXT_ITEM) == RINGBUF_OK)
			{
				// TODO: Werte irgendwie ablegen, soweit sinnvoll, so wie oben

				/* Seems to be last command for initialization */
				if(msg.command == 0x0B /*&& msg.data[0] == 0x01*/) //-> mit Aux kann es auch 0x10 sein
				{
					bpCommState = BP_IDLE;
					printf("Finished init 9600\r\n");
					//while(((USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData))->TxState!=0);
					//CDC_Transmit_FS(buffer_tmp, strlen(buffer_tmp));
				}

				//bpMsgState.processMessagePos = (bpMsgState.processMessagePos+1) % (BP_MAX_MESSAGES);
			}
			break;

		case BP_IDLE:

#ifndef DAB_DEBUG_ACTIVE
			if(ringGet(bpMsgState.readBuf, &msg, RINGBUF_NEXT_ITEM) == RINGBUF_OK)
			{
				if(msg.command == 0x0B && msg.dataLen == 0) // TODO: abfragen, ob msg complete?
				{
					idle_zwischen++;
				}

				if(msg.command == 0x22 && msg.dataLen == 0) // TODO: kann raus oder? Knopf losgelassen? -> demnächst raus werfen
				{
					idle_zwischen++;
				}
			}
#endif

			if(idle_zwischen == 2)
			{
				idle_zwischen = 0;
				/* Prepare to send messages in next step */
				uint8_t data[2];

				data[0] = 0x01;
				ringAdd(bpMsgState.writeBuf, buildMessage(0x175, 1, 0x30, data, 65));
				data[0] = 0x42;
				ringAdd(bpMsgState.writeBuf, buildMessage(0x170, 1, 0x0B, data, 85));
				ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_STATION_NOT_FOUND]);
				data[0] = 0x80;
				ringAdd(bpMsgState.writeBuf, buildMessage(0x175, 1, 0x14, data, 70));
				data[0] = 0x80;
				ringAdd(bpMsgState.writeBuf, buildMessage(0x175, 1, 0x9F, data, 1));
				data[0] = 0x01;
				ringAdd(bpMsgState.writeBuf, buildMessage(0x175, 1, 0x30, data, 1));
				data[0] = 0x42;
				ringAdd(bpMsgState.writeBuf, buildMessage(0x170, 1, 0x0B, data, 1));
				data[0] = 0x70;
				ringAdd(bpMsgState.writeBuf, buildMessage(0x175, 1, 0x70, data, 1));
				data[0] = 0xEF;
				data[1] = 0xFF;
				ringAdd(bpMsgState.writeBuf, buildMessage(0x175, 2, 0x57, data, 1));

				msg = buildTextMessage("Simu v03", 10);
				ringAdd(bpMsgState.writeBuf, msg);

				ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_TA_INACTIVE]);
				//ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_SIGNAL_TS_T]); // TODO: Sinnvoll verwenden
				data[0] = 0xEF;
				data[1] = 0xFF;
				ringAdd(bpMsgState.writeBuf, buildMessage(0x175, 2, 0x57, data, 1));
				ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_SIGNAL_TA_OFF]);

				//ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_VOL_PLUS]);
				ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_REASED_17D]);

				//msg = buildTextMessage("Simu v03", 1);
				msg = buildTextMessage("Plüm2000", 10);
				ringAdd(bpMsgState.writeBuf, msg);

				//uswusw...

				data[0] = 0x42;
				ringAdd(bpMsgState.writeBuf, buildMessage(0x170, 1, 0x0B, data, 65));
				data[0] = 0x80;
				ringAdd(bpMsgState.writeBuf, buildMessage(0x175, 1, 0x3F, data, 70));

				printf("Activating our communication\r\n");
				//while(((USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData))->TxState!=0);
				//CDC_Transmit_FS(buffer_tmp, strlen(buffer_tmp));

				/* Wait for ???ms *///TODO Raus schmeißen
				//bpMsgState.waitMs = HAL_GetTick() + 10; //TODO wie lange? Ist erst nur aus einer Aufzeichnung raus

				bpCommState = BP_SEND_WAIT;
			}

			break;

		case BP_SEND_WAIT:
			/* Wait until we can send the next message */
			if(HAL_GetTick() > bpMsgState.waitTickMs)
			{
				//printf("BP_SEND_WAIT: Time to send\r\n");

				/* Go to sendig state if there is no further waiting nec. */
				//if(sendRingMessage(RINGBUF_KEEP_ITEM) != RINGBUF_WAIT) // TODO sollte ok sein.. leer wäre auch komisch
				if(sendRingMessage(&bpMsgState, RINGBUF_KEEP_ITEM) == RINGBUF_OK)
				{
					bpCommState = BP_SEND;
				}
				else
				{
					// TODO errorhandling
				}
			}
			break;

		case BP_SEND:

			/* First message sent in state before, wait for response from radio. Check if the answers match */
			if(ringGet(bpMsgState.readBuf, &msg, RINGBUF_NEXT_ITEM) == RINGBUF_OK)
			{
				//printf("Nachricht bekommen...\r\n");

				/* Is this message a response to our/a message? */
				if(msg.messageState == MSG_COMPLETE_RESPONSE) // TODO: Wenn der direkt antwortet, ist das keine Response...
				{
					bp_msg_dt msg_sent;

					/* And we can get a message from the send buffer? */
					if(ringGet(bpMsgState.writeBuf, &msg_sent, RINGBUF_KEEP_ITEM) == RINGBUF_OK)
					{
						/* Compare if the answer of our transmitted msg matches */
						if(compareMessages(&msg, &msg_sent) != MSG_ERR_NONE) // TODO: prüfen, ob der wirklich meckert, wenn die sich unterscheiden
						{
							printf("Answer from radio doesn't match our message (Rec: %X %X %X)!\r\n",
									msg.address, msg.dataLen, msg.command);
							//while(((USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData))->TxState!=0);
							//CDC_Transmit_FS(buffer_tmp, strlen(buffer_tmp));

							/* Resend this message... */
						}
						else
						{
							/* Jump to next message to send */
							ringNextReadInd(bpMsgState.writeBuf);

							printf("Debug: Got echo to our message...\r\n");
						}

						/* Check for next message (or current again), or go to next state when no messages left */
						if(ringReadAvailable(bpMsgState.writeBuf) == RINGBUF_NO_DATA)
						{
							printf("Going to running state\r\n");

							/* All messages seem to be sent correctly, go to next state */
							bpCommState = BP_RUNNING;

							//HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
						}
						else
						{
							/* Go back to wait loop if further waiting is needed after this message */
							bpMsgState.waitTickMs = (bpMsgState.msgReceivedTime + msg_sent.waitAfter_ms);

							/* Go to waiting state */
							bpCommState = BP_SEND_WAIT;
						}
					}
					else
					{
						/* Some bigger kind of fault... */
						printf("Got an answer, but no sent messages available.\r\n");

						/* Clear ringbuffer for sending */
						ringClear(bpMsgState.writeBuf);

						bpCommState = BP_IDLE;
						//break;
					}
				}
				else
				{
					printf("Message not a response (Rec: %X %X %X). Pushing it back...\r\n",
									msg.address, msg.dataLen, msg.command);

					/* Put the message back on the ring, we are expecting an answer; will go out here as soon as send ring is empty */
					ringAdd(bpMsgState.readBuf, msg);

					/* Skip directly if the write buffer is empty */
					if(ringReadAvailable(bpMsgState.writeBuf) == RINGBUF_NO_DATA)
					{
						bpCommState = BP_RUNNING;
					}

					//printf("Unexpected message! Expected msg to be a response...\r\n");

					//Error_Handler();

					/* Clear ringbuffer for sending */
					//ringClear(bpMsgState.writeBuf);

					//bpCommState = BP_IDLE; TODO Testweise raus

					//break;
				}

			}

			break;

		case BP_RUNNING:
			if(ringGet(bpMsgState.readBuf, &msg, RINGBUF_NEXT_ITEM) == RINGBUF_OK)
			{
				if(processBpMsg(&msg) == MSG_ERR_NONE)
				{

				}
				else
				{
					// Unbekannte nachrichten fürs erste ignorieren?
				}

				//bpMsgState.processMessagePos = (bpMsgState.processMessagePos+1) % (BP_MAX_MESSAGES);
			}

			// TODO: Periodisch Text senden, oder?
			// hier ist dann auch der Teil, dass er beim RingbufferSenden erst schauen soll,
			// ob gerade eine Nachricht empfangen wird, bzw. die 14F lange genug her ist

			break;
		case BP_SEARCH_PROGRAM:
		case BP_MENU:
		case BP_DEACTIVATE: /* Send states after deactivation */
			// TODO: noch deinitialisierungssachen einbauen...

			bpCommState = BP_IDLE;
			break;
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	bp_msg_state_dt * currState;
	uint16_t * cur_rx_data;

#ifdef DAB_DEBUG_ACTIVE
	/* Attach the current Message to the according USART, debugging communication between DAB and Radio */
	if(huart->Instance == USART2)
	{
		currState = &bpMsgState;
		cur_rx_data = bpMsgState.uart.rx_data;

		//HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	}
	else if(huart->Instance == USART3) // Debug-UART
	{
		currState = &bpMsgState_DAB;
		cur_rx_data = bpMsgState.uart.rx_data_debug;
	}
#else
	currState = &bpMsgState;
	cur_rx_data = bpMsgState.uart.rx_data;
#endif

	HAL_GPIO_TogglePin(LED_BLUE_Port, LED_BLUE_Pin);

	switch(currState->receivePosition)
	{
		case MSG_ADDRESS:
			if(bpCommState > BP_INIT_9600 ||
			   (cur_rx_data[0] == BP_ADD_RADIO_178 ||
				cur_rx_data[0] == BP_ADD_RADIO_17C ||
				cur_rx_data[0] == BP_ADD_RADIO_BUTTON_17D ) ) // TODO für debug das hier nicht?
			{
				//currState->readBuf->msg[currState->readBuf->writeInd].address      = cur_rx_data[0];
				currState->curReadMsg.address      = cur_rx_data[0];
				currState->curReadMsg.timeStamp_ms = HAL_GetTick(); // Get current timestamp
				//currState->readBuf->msg[currState->readBuf->writeInd].timeStamp_ms = HAL_GetTick(); // Get current timestamp

				// TODO: hier sollte die current message gelöscht werden.... memset?

				currState->curReadMsg.dataLen = 99;
				//currState->readBuf->msg[currState->readBuf->writeInd].dataLen = 99;
				for(uint8_t i=0; i<sizeof(currState->curReadMsg.data); i++)
				//for(uint8_t i=0; i<sizeof(currState->readBuf->msg[currState->readBuf->writeInd].data); i++)
				{
					currState->curReadMsg.data[i] = 0;
					//currState->readBuf->msg[currState->readBuf->writeInd].data[i] = 0;
				}

				/* Reply to received bytes */
				if(currState->direction == MSG_DIRECTION_RECEIVE)
				{
					currState->responseActive = MSG_RESPONSE_ACTIVE;
				}
				currState->curReadMsg.messageState = MSG_INCOMPLETE;
				//currState->readBuf->msg[currState->readBuf->writeInd].messageState = MSG_INCOMPLETE;

				currState->receivePosition = MSG_LENGTH;
			}
			break;

		case MSG_LENGTH:
			currState->curReadMsg.dataLen = (uint8_t) cur_rx_data[0];
			//currState->readBuf->msg[currState->readBuf->writeInd].dataLen = (uint8_t) cur_rx_data[0];

			currState->receivePosition = MSG_CMD;
			break;

		case MSG_CMD:
			currState->curReadMsg.command = (uint8_t) cur_rx_data[0];
			//currState->readBuf->msg[currState->readBuf->writeInd].command = (uint8_t) cur_rx_data[0];

			currState->currentDataByte = 0;
			currState->receivePosition = MSG_DATA;
			break;

		case MSG_DATA:
			if(cur_rx_data[0] != BP_MSG_END) // As long as we receive anything but an END
			{
				if(currState->currentDataByte < BP_MSG_MAX_BYTES)
				{
					currState->curReadMsg.data[currState->currentDataByte++] = (uint8_t) cur_rx_data[0];
					//currState->readBuf->msg[currState->readBuf->writeInd].data[currState->currentDataByte++] = (uint8_t) cur_rx_data[0];
				}
				else
				{
					Error_Handler(); // TODO: bei Interrupts besser: Error-Flag setzen
					HAL_GPIO_TogglePin(LED_RED_Port, LED_RED_Pin);
				}
			}
			else if(huart->Instance == USART2)
			{
				// TODO: Abfrage, ob länge und counter übereinstimmen: haben wir alles empfangen?
				// TODO: Ringbuffer mit allen NAchrichtne?
				// TODO: Errorhandler

				/* Timestamp for last received message */
				currState->msgReceivedTime = HAL_GetTick();

				currState->responseActive = MSG_RESPONSE_INACTIVE;

				if(bpMsgState.direction == MSG_DIRECTION_SEND) /* If this message is expected to be a response from the radio to our msg */
				{
					currState->curReadMsg.messageState = MSG_COMPLETE_RESPONSE;
					//currState->readBuf->msg[currState->readBuf->writeInd].messageState = MSG_COMPLETE_RESPONSE;
				}
				else
				{
					currState->curReadMsg.messageState = MSG_COMPLETE;
					//currState->readBuf->msg[currState->readBuf->writeInd].messageState = MSG_COMPLETE;
				}

				// Next position in buffer
				//currState->currentMessagePos     = (currState->currentMessagePos+1) % (BP_MAX_MESSAGES);
				//ringNextWriteInd(currState->readBuf);

				/* Add message to ring */
				ringAdd(currState->readBuf, currState->curReadMsg);

				/* Switch back to receive, ringbuffer writer can turn it back, if necessary */
				bpMsgState.direction = MSG_DIRECTION_RECEIVE;

				currState->receivePosition = MSG_ADDRESS; // Start again

#ifdef DAB_DEBUG_ACTIVE
				// end both states...

				//bpMsgState_DAB.readBuf->msg[bpMsgState_DAB.readBuf->writeInd].messageState  = MSG_COMPLETE;
				bpMsgState_DAB.receivePosition			 = MSG_ADDRESS; // zusätzlich, da kein 0x14F

				ringAdd(bpMsgState_DAB.readBuf, bpMsgState_DAB.curReadMsg);
				//ringNextWriteInd(bpMsgState_DAB.readBuf);
#endif
			}

	}

	if(currState->responseActive == MSG_RESPONSE_ACTIVE)
	{
		HAL_UART_Transmit_IT(&huart2, (uint8_t *)cur_rx_data, 1);
	}

	/* If we're in sending condition, send next byte */
	if(currState->direction == MSG_DIRECTION_SEND)
	{
		HAL_UART_Transmit_IT(&huart2, (uint8_t *)&bpMsgState.uart.tx_data[bpMsgState.uart.tx_data_pos], 1);
		// TODO: Anfragen, ob der jetzt überhaupt senden darf: Letzte Empfangene Botschaft sollte xy her sein
		bpMsgState.uart.tx_data_pos++;
	}

	HAL_UART_Receive_IT(huart, (uint8_t*) cur_rx_data, 1);
}



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

	for(bp_msg_en i=1; i<BP_MSG_SIZE; i++) // start with 1, 0 is "unknown message"
	{
		if(message->address == bpMessages[i].address)
		{
			if(message->command == bpMessages[i].command)
			{
				// TODO: should just jump over if datalen <1 or not mentioned -> Testen
				for(uint8_t j=0; j<message->dataLen; j++) // if dataLen is mentioned and we need to compare data
				{
					if(message->data[j] != bpMessages[i].data[j])
					{
						return BP_MSG_UNKNOWN;
					}
				}
				return i;
			}
		}
	}

	return BP_MSG_UNKNOWN;
}

/**
  * @brief Process a message from the radio
  * @param received message
  * @retval state of message process
  */
uint8_t cnt = 0x01; // TODO rausschmeißen
bp_msg_error processBpMsg(bp_msg_dt * message)
{
	bp_msg_error retVal = MSG_ERR_NONE;
	bp_msg_en messageIndex = findMessage(message);

	bp_msg_dt msg;

	uint8_t buf[8] = {0,};
	uint8_t buf2[2] = {0,};
//	uint8_t cnt = 0x01;

	switch(messageIndex)
	{
		case BP_MSG_UNKNOWN:
			// TODO: Loggen?
			retVal = MSG_UNKNOWN;
			break;

		case BP_MSG_BUT_6:
			buf[0] = 0xFE;
			buf[1] = 0xFF;
			msg = buildMessage(0x175, 2, 0x77, buf, 10);
			ringAdd(bpMsgState.writeBuf, msg);
			bpCommState = BP_SEND_WAIT;
			break;

		case BP_MSG_BUT_5:
			buf[0] = 0xFD;
			buf[1] = 0xFF;
			msg = buildMessage(0x175, 2, 0x77, buf, 10);
			ringAdd(bpMsgState.writeBuf, msg);
			bpCommState = BP_SEND_WAIT;
			break;

		case BP_MSG_BUT_4:
			cnt+=8;
		case BP_MSG_BUT_3:
			printf("Button 3 pressed... \r\n");

			if(++cnt > 0xFF)
			{
				cnt = 0;
			}


			buf2[0] = 0x00 + cnt<<4;
			msg = buildMessage(0x175, 1, 0x70, buf2, 10);
			ringAdd(bpMsgState.writeBuf, msg);

			sprintf(buf,"0x%02X", buf2[0]);
			msg = buildTextMessage(buf, 20);
			ringAdd(bpMsgState.writeBuf, msg);
			bpCommState = BP_SEND_WAIT;
			break;

		case BP_MSG_BUT_2:
			cnt+=8;
		case BP_MSG_BUT_1:
			//for(uint8_t i=0; i<20; i++)
			{
				for(uint8_t j=0; j<8;j++)
				{
					buf[j]=cnt++;
				}
				msg = buildTextMessage(buf, 100);
				ringAdd(bpMsgState.writeBuf, msg);
			}

			bpCommState = BP_SEND_WAIT;
			break;

		case BP_MSG_BUT_DOWN:
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_BUT_DOWN]);
			bpCommState = BP_SEND_WAIT;
			break;

		case BP_MSG_BUT_UP:
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_BUT_UP]);
			bpCommState = BP_SEND_WAIT;
			break;

		case BP_MSG_BUT_LEFT:
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_BUT_LEFT]);
			bpCommState = BP_SEND_WAIT;
			break;

		case BP_MSG_BUT_RIGHT:
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_BUT_RIGHT]);
			bpCommState = BP_SEND_WAIT;
			break;


		//case BP_MSG_BUT_DSC:
		//case BP_MSG_BUT_LD:
		//	break;
		case BP_MSG_BUT_AUD:
			printf("AUD gedrückt... \r\n");

			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_BUT_AUD]);
			bpCommState = BP_SEND_WAIT;
			break;

		case BP_MSG_BUT_RELEASED_17C:
			//TODO: Braucht das auch ein ACK?

			break;
		case BP_MSG_BUT_RELEASED_17D:
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_REASED_17D]);
			bpCommState = BP_SEND_WAIT;
			break;

		case BP_MSG_BUT_SCA:
			printf("\033[1;33mSCA gedrückt...\033[0m\r\n");
			buf[0] = cnt;
			buf[1] = 0x00;
			msg = buildMessage(0x175, 2, 0x56, buf, 10);
			ringAdd(bpMsgState.writeBuf, msg);
			sprintf(buf, "N %X", cnt);
			msg = buildTextMessage(buf, 10);
			ringAdd(bpMsgState.writeBuf, msg);
			cnt = cnt*2;
			bpCommState = BP_SEND_WAIT;
			break;

		case BP_MSG_BUT_PS:
			buf[0] = 0xEF;
			buf[1] = 0xFF;
			msg = buildMessage(0x175, 2, 0x57, buf, 10);
			ringAdd(bpMsgState.writeBuf, msg);
			bpCommState = BP_SEND_WAIT;
			break;
		case BP_MSG_BUT_MIX:
		case BP_MSG_BUT_GEO:
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_BUT_GEO]);
			bpCommState = BP_SEND_WAIT;
			break;

		case BP_MSG_BUT_TA:
		case BP_MSG_BUT_lo:
		case BP_MSG_BUT_AF:
		case BP_MSG_BUT_RM:
		case BP_MSG_BUT_dx:
		case BP_MSG_BUT_FM:
		case BP_MSG_BUT_TS:
			break;

		case BP_MSG_BUT_dB:
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_BUT_dB]);
			bpCommState = BP_SEND_WAIT;
			break;

		case BP_MSG_BUT_VOL_MIN:
			printf("\033[1;33mVOL- gedrückt...\033[0m\r\n");
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_VOL_MIN]);
			bpCommState = BP_SEND_WAIT;
			break;

		case BP_MSG_BUT_VOL_PLUS:
			printf("\033[1;33mVOL+ gedrückt...\033[0m\r\n");
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_VOL_PLUS]);
			bpCommState = BP_SEND_WAIT;
			break;

		case BP_MSG_LEAVE_VOL:
			printf("\033[1;33mVerlasse VOL-Menü\033[0m\r\n");
			msg = buildTextMessage("VOL verl.", 10);
			ringAdd(bpMsgState.writeBuf, msg);
			bpCommState = BP_SEND_WAIT;
			break;

		case BP_MSG_LEAVE_AUD:
			printf("\033[1;33mVerlasse AUD-Menü\033[0m\r\n");
			break;

		case BP_MSG_LEAVE_MUTE:
			printf("\033[1;33mVerlasse MUTE-Menü\033[0m\r\n");
			break;

		default:
			printf("\033[1;33m(noch) nicht implementierter, bekannter Befehl... \033[0m\r\n");

			break;
	}

	return retVal;
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

bp_msg_dt buildTextMessage(char * text, uint32_t waitMs) // TODO: zunächst nur einfache Texte, die den Bildschirm 1x füllen
{
	uint8_t textLen = 0;
	char outbuf[25];

	utf2bp(text, strlen(text), outbuf, sizeof(outbuf));

	textLen = strlen(outbuf);
	if(textLen > 8)
	{
		textLen = 8;
		// TODO ausrichten (zentrieren, links, ..) und so...
		// später: Texte mit > 8 Zeichen nacheinander ausgeben? Je nach Status, ob wir active sind? Buffer bauen und der reihe nach ausgeben?

	}

	//return buildMessage(bpMessages[BP_MSG_TEXT].address, textLen, bpMessages[BP_MSG_TEXT].command, text, waitMs);
	return buildMessage(bpMessages[BP_MSG_TEXT].address, textLen, bpMessages[BP_MSG_TEXT].command, outbuf, waitMs);
}

/* Send a ring message, if available */
ringbuf_status_en sendRingMessage(bp_msg_state_dt * msgState, ringbuf_next_en nextItem)
{
	bp_msg_dt msg;

	ringbuf_status_en ringbufStatus = ringGet(msgState->writeBuf, &msg, RINGBUF_KEEP_ITEM);

	printf("Debug: Sending %X;%d;%02X\r\n", msg.address, msg.dataLen, msg.command);

	if(ringbufStatus == RINGBUF_NO_DATA)
	{
		return RINGBUF_NO_DATA;
	}

	//if(HAL_GetTick() < bpMsgState.msgReceivedTime + msg.waitAfter_ms) // minus mininimale-ZEit?
	//{
	//	return RINGBUF_WAIT;
	//}


	if(ringbufStatus == RINGBUF_OK)
	{
		uint8_t i = 0;

		bpMsgState.uart.tx_data_pos = 0;

		//uint16_t txData[3+msg.dataLen];
		bpMsgState.uart.tx_data[0] = msg.address;
		bpMsgState.uart.tx_data[1] = msg.dataLen;
		bpMsgState.uart.tx_data[2] = msg.command;
		for(i=0; i<msg.dataLen; i++)
		{
			bpMsgState.uart.tx_data[3+i] = msg.data[i];
		}
		bpMsgState.uart.tx_data[3+i] = BP_MSG_END;

		/* We are sending now.. */
		bpMsgState.direction = MSG_DIRECTION_SEND;

		//bpMsgState.checkMsg = msg; // TODO: Nachricht, die zum prüfen genommen wird

		HAL_UART_Transmit_IT(&huart2, (uint8_t *)&bpMsgState.uart.tx_data[bpMsgState.uart.tx_data_pos], 1); // TODO: Anfragen, ob der jetzt überhaupt senden darf!!!!
		bpMsgState.uart.tx_data_pos++;

		/* Switch to next item if wanted */
		if(nextItem == RINGBUF_NEXT_ITEM)
		{
			ringNextReadInd(bpMsgState.writeBuf);
		}

		//return RINGBUF_OK; // TODO: Auch Rückgabe anders, wenn der noch am senden ist...
	}

	return ringbufStatus; // no data
}

void bpDebugPrint(void)
{
	bp_msg_dt msg_debug1;
#ifdef DAB_DEBUG_ACTIVE
	bp_msg_dt msg_debug2;
#endif

	uint8_t debug_Print = 0; // TODO schöner machen

	ringbuf_next_en keepItem = RINGBUF_KEEP_ITEM;

#ifdef DAB_DEBUG_ACTIVE
	keepItem = (bpCommState > BP_INIT_9600 ? RINGBUF_NEXT_ITEM : RINGBUF_KEEP_ITEM);
#endif

	if(ringGet(bpMsgState.readBuf, &msg_debug1, keepItem) == RINGBUF_OK)
		  {
			  debug_Print |= 0x01;
		  }

	#ifdef DAB_DEBUG_ACTIVE

		  //if(printMessagePos_debug != bpMsgState_DAB.readBuf->readInd)
		  if(ringGet(bpMsgState_DAB.readBuf, &msg_debug2, RINGBUF_NEXT_ITEM) == RINGBUF_OK)
		  {
			  debug_Print |= 0x02;
		  }

		  /*if(debug_Print & 0x03 && (msg_debug1.timeStamp_ms & 0xFFFFFFF8) == (msg_debug2.timeStamp_ms & 0xFFFFFFF8))
		  {
			  debug_Print = 0;
			  printf("\033[1;35m%lu;%X;%X;%X;%X;%X;%X;%X;;%X;%d;%X;%X;%X;%X\033[0m\r\n",
			  				  msg_debug1.timeStamp_ms,
			  				  msg_debug1.address,
			  				  msg_debug1.dataLen,
			  				  msg_debug1.command,
			  				  msg_debug1.data[0],
			  				  msg_debug1.data[1],
			  				  msg_debug1.data[2],
			  				  msg_debug1.data[3],
							  msg_debug2.address,
							  msg_debug2.dataLen,
							  msg_debug2.command,
							  msg_debug2.data[0],
							  msg_debug2.data[1],
							  msg_debug2.data[2],
							  msg_debug2.data[3]
			  				  );
		  }*/

		  if(debug_Print & 0x02 && (msg_debug2.address == 0x170 || msg_debug2.address == 0x175))
		  {
			  debug_Print &= ~(1<<1);

			  printf("\033[1;34m%lu;;;;;;;;;%X;%d;%02X;%02X;%02X;%02X;%02X\033[0m\r\n",
			  				  msg_debug2.timeStamp_ms,
			  				  msg_debug2.address,
			  				  msg_debug2.dataLen,
			  				  msg_debug2.command,
			  				  msg_debug2.data[0],
			  				  msg_debug2.data[1],
			  				  msg_debug2.data[2],
			  				  msg_debug2.data[3]);
		  }
	#endif

		  if(debug_Print & 0x01 && (msg_debug1.address == 0x178 || msg_debug1.address == 0x17C || msg_debug1.address == 0x17D))
		  {
			  debug_Print &= ~(1<<0);
			  printf("\033[1;33m%lu;%X;%d;%02X;%02X;%02X;%02X;%02X;;;;;;;;\033[0m\r\n",
					  msg_debug1.timeStamp_ms,
					  msg_debug1.address,
					  msg_debug1.dataLen,
					  msg_debug1.command,
					  msg_debug1.data[0],
					  msg_debug1.data[1],
					  msg_debug1.data[2],
					  msg_debug1.data[3]
					  );
		  }
	/*
		  if(printMessagePos != bpMsgState.readBuf->readInd) // Message to show available?
		  {
			  if(ringGet(bpMsgState.readBuf, &msg_debug1, RINGBUF_KEEP_ITEM) == RINGBUF_OK)
			  {
				  printf("%lu;%X;%X;%X;%X;%X;%X;%X\r\n",
						  msg_debug1.timeStamp_ms,
						  msg_debug1.address,
						  msg_debug1.dataLen,
						  msg_debug1.command,
						  msg_debug1.data[0],
						  msg_debug1.data[1],
						  msg_debug1.data[2],
						  msg_debug1.data[3]
						  );
			  }

			  printMessagePos = (printMessagePos+1) % (bpMsgState.readBuf->bufSize);
		  }*/
}
