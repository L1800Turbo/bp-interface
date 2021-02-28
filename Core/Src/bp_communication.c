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

extern const bp_msg_dt bpMessages[]; // TODO noch auslagern?

void bpCommInit(void)
{
	/* Initialize ring buffers */
	bpMsgState.writeBuf = ringInit(30);
	bpMsgState.readBuf  = ringInit(20);

	bpMsgState.debugBuffer = ringInit(20);

#ifdef DAB_DEBUG_ACTIVE
	//bpMsgState_DAB.readBuf  = ringInit(20);
	bpMsgState_DAB.debugBuffer = ringInit(20);
#endif

	bpMsgState.direction = MSG_DIRECTION_RECEIVE;

	/* Initialize interrupt receivement for UART */
	HAL_UART_Receive_IT(&huart2, (uint8_t*) bpMsgState.uart.rx_data, 1);
#ifdef DAB_DEBUG_ACTIVE
	HAL_UART_Receive_IT(&huart3, (uint8_t*) bpMsgState.uart.rx_data_debug, 1);
#endif

	bpDisplayInit();
}

/* Set so SEND_WAIT state unless a message is already being sent, in this case the message would be taken from stack */
void setSendWait(void)
{
	if(bpCommState != BP_SEND)
	{
		bpCommState = BP_SEND_WAIT;
	}
}

void bpCommTasks(void)
{
	bp_msg_dt msg;

	// Check for message timeouts, if we're receiving a message
	if(bpMsgState.curReadMsg.messageState == MSG_INCOMPLETE &&
			(HAL_GetTick()-bpMsgState.curReadMsg.timeStamp_ms) > 2500) //TODO: generischen Timeout festlegen
	{
		printf("E: Timeout while receiving message!\r\n");
		bpMsgState.curReadMsg.messageState = MSG_TIMEOUT;
	}

	switch(bpCommState)
	{
		case BP_UNINITIALIZED:

			bpCommState = BP_INIT_4800; // just skip directly to 4800baud init
			// Later: activation when 5V from radio o.æ.
			//break;

		case BP_INIT_4800:
			if(ringGet(bpMsgState.readBuf, &msg, RINGBUF_NEXT_ITEM) == RINGBUF_OK)
			{
				// TODO:
				// Initialization includes also these 4800 Baud commands
				// 178	1	48	01	00	00	00
				// 17C	1	48	01	00	00	00
				// 17C	1	48	01	00	00	00
				// 17D	1	48	01	00	00	00

				/* after cmd 0x48 with data 0x02 radio switches to 9600 baud */
				if(msg.command == 0x48 && msg.data[0] == 0x02)
				{
					HAL_UART_DeInit(&huart2);

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
				}
		    }
			break;

		case BP_INIT_9600:
			if(ringGet(bpMsgState.readBuf, &msg, RINGBUF_NEXT_ITEM) == RINGBUF_OK)
			{
				// TODO: Werte irgendwie ablegen, soweit sinnvoll, so wie oben

				/* Seems to be last command for initialization */
				if(msg.command == 0x0B /*&& msg.data[0] == 0x01*/) //-> mit Aux kann es auch 0x10 sein
				{
					bpCommState = BP_IDLE;
					printf("Finished init 9600\r\n");
				}

				//bpMsgState.processMessagePos = (bpMsgState.processMessagePos+1) % (BP_MAX_MESSAGES);
			}
			break;

		case BP_IDLE:
#ifndef DAB_DEBUG_ACTIVE
			if(ringGet(bpMsgState.readBuf, &msg, RINGBUF_NEXT_ITEM) == RINGBUF_OK)
			{
				if(processBpMsg(&msg) == MSG_ERR_NONE)
				{

				}
				else
				{
					// Unbekannte nachrichten fürs erste ignorieren?
				}
			}
#endif
			break;

		case BP_SEND_WAIT:
			/* Wait until we can send the next message */
			if(HAL_GetTick() > bpMsgState.waitTickMs)
			{
				/* Go to sendig state if there is no further waiting nec. */
				if(sendRingMessage(&bpMsgState, RINGBUF_KEEP_ITEM) == RINGBUF_OK)
				{
					// TODO: hier müsste Timeouthandling engebaut werden. Er würde sonst ggf. Jahre auf eine Antwort warten...
					bpCommState = BP_SEND;
				}
				else
				{
					// TODO errorhandling
				}
			}
			break;

		case BP_SEND:

			/* Resend if timeout */
			if(bpMsgState.curReadMsg.messageState == MSG_TIMEOUT)
			{
				bpCommState = BP_SEND_WAIT;

				printf("E: Message timeout while sending. Back to BP_SEND_WAIT\r\n");

				// TEST: raus werfen
				//ringNextReadInd(bpMsgState.writeBuf);

				// TODO abbrechen, wenn das mehrmals schief geht?
				// oder den Senderingbuffer weg werfen?
			}

			/* First message sent in state before, wait for response from radio. Check if the answers match */
			if(ringGet(bpMsgState.readBuf, &msg, RINGBUF_NEXT_ITEM) == RINGBUF_OK)
			{
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

							// TODO: Errorhandling? Neu senden?
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

							/* All messages seem to be sent correctly, go back to idle */
							bpCommState = BP_IDLE;
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
					printf("Message not a response (Rec: %X %X %X). Pushing it back... Messages on write-ring: %d\r\n",
									msg.address, msg.dataLen, msg.command, ringCurrentSize(bpMsgState.writeBuf));

					/* Put the message back on the ring, we are expecting an answer; will go out here as soon as send ring is empty */
					ringAdd(bpMsgState.readBuf, msg);

					/* Skip directly if the write buffer is empty */
					if(ringReadAvailable(bpMsgState.writeBuf) == RINGBUF_NO_DATA)
					{
						bpCommState = BP_IDLE;
					}

					//bpCommState = BP_IDLE; TODO Testweise raus

					//break;
				}

			}
			break;
	}

	bpDisplayTasks();
}

/**
 * void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 * UART communication to BP radio
 *
 * Inputs:
 * UART_HandleTypeDef *huart: matching UART handle, currently HW dependend
 *
 */
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

	/* Restart if there was a timeout in a previous message */
	if(currState->curReadMsg.messageState == MSG_TIMEOUT)
	{
		currState->direction = MSG_DIRECTION_RECEIVE;
		currState->receivePosition = MSG_ADDRESS;
	}

	switch(currState->receivePosition)
	{
		case MSG_ADDRESS:
			if(bpCommState > BP_INIT_9600 ||
			   (cur_rx_data[0] == BP_ADD_RADIO_178 ||
				cur_rx_data[0] == BP_ADD_RADIO_17C ||
				cur_rx_data[0] == BP_ADD_RADIO_BUTTON_17D ) ) // TODO für debug das hier nicht?
			{
				currState->curReadMsg.address      = cur_rx_data[0];
				currState->curReadMsg.timeStamp_ms = HAL_GetTick(); // Get current timestamp

				/* Set to default values */
				currState->curReadMsg.dataLen = 99;
				for(uint8_t i=0; i<sizeof(currState->curReadMsg.data); i++)
				{
					currState->curReadMsg.data[i] = 0;
				}

				/* Reply to received bytes */
				if(currState->direction == MSG_DIRECTION_RECEIVE)
				{
					currState->responseActive = MSG_RESPONSE_ACTIVE;
				}

				currState->curReadMsg.messageState = MSG_INCOMPLETE;
				currState->receivePosition = MSG_LENGTH;
			}
			break;

		case MSG_LENGTH:
			currState->curReadMsg.dataLen = (uint8_t) cur_rx_data[0];

			currState->receivePosition = MSG_CMD;
			break;

		case MSG_CMD:
			currState->curReadMsg.command = (uint8_t) cur_rx_data[0];

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
				}
				else
				{
					currState->curReadMsg.messageState = MSG_COMPLETE;
				}

				/* Add message to ring */
				ringAdd(currState->readBuf, currState->curReadMsg);

				//TODO durch DEFINE oder einzelne Config ausblenden
				ringAdd(currState->debugBuffer, currState->curReadMsg);

				/* Switch back to receive, ringbuffer writer can turn it back, if necessary */
				bpMsgState.direction = MSG_DIRECTION_RECEIVE;

				currState->receivePosition = MSG_ADDRESS; // Start again

#ifdef DAB_DEBUG_ACTIVE
				// end both states...

				//bpMsgState_DAB.readBuf->msg[bpMsgState_DAB.readBuf->writeInd].messageState  = MSG_COMPLETE;
				bpMsgState_DAB.receivePosition			 = MSG_ADDRESS; // zusätzlich, da kein 0x14F

				ringAdd(bpMsgState_DAB.debugBuffer, bpMsgState_DAB.curReadMsg);
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

	uint8_t buf[9] = {0,};
	uint8_t buf2[2] = {0,};
//	uint8_t cnt = 0x01;

	switch(messageIndex)
	{
		case BP_MSG_UNKNOWN:
			// TODO: Loggen?

			printf("\033[1;202mUnkown message: %X %d %02X %02X %02X %02X %02X\033[0m\r\n",
				message->address,
				message->dataLen,
				message->command,
				message->data[0],
				message->data[1],
				message->data[2],
				message->data[3]
				);

			retVal = MSG_UNKNOWN;
			break;

		case BP_MSG_BUT_SRC: // TODO: nur auf losgelassen reagieren. Richtig?
			break;

		case BP_MSG_BUT_SRC_RELEASED:
			//if(bpCommState == BP_IDLE) // Run actication sequence TODO: und wenn er gerade sendet oder so?
			if(stateFlags.bpDabActive == bp_DAB_inactive)
			{
				uint8_t data[2];

				/* Prepare to send messages in next step */
				ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_ACTIVATE]);
				//data[0] = 0x01;
				//ringAdd(bpMsgState.writeBuf, buildMessage(0x175, 1, 0x30, data, 65));

				//data[0] = 0x42;
				//ringAdd(bpMsgState.writeBuf, buildMessage(0x170, 1, 0x0B, data, 85));
				// -> wenn man DAB verlässt, macht er AUX, wenn das hier aktiviert ist ?!? Irgendwelche Botschaften unten noch relevant

				ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_STATION_NOT_FOUND]);

				//data[0] = 0x80;
				//ringAdd(bpMsgState.writeBuf, buildMessage(0x175, 1, 0x14, data, 70));

				//data[0] = 0x80;
				//ringAdd(bpMsgState.writeBuf, buildMessage(0x175, 1, 0x9F, data, 1));

				ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_ACTIVATE]);

				//data[0] = 0x42;
				//ringAdd(bpMsgState.writeBuf, buildMessage(0x170, 1, 0x0B, data, 10));

				msg = bpMessages[BP_MSG_SIGNAL_CHAN];
				//msg.data[0] = SIG_NO_BAND | SIG_CHAN_NONE; -> Already activated by default values
				ringAdd(bpMsgState.writeBuf, msg);

				//data[0] = 0xEF;
				//data[1] = 0xFF;
				//ringAdd(bpMsgState.writeBuf, buildMessage(0x175, 2, 0x57, data, 10));

				//msg = buildTextMessage("Simu v03", 10);
				//ringAdd(bpMsgState.writeBuf, msg);

				//ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_TA_INACTIVE]);
				//ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_SIGNAL_TS_T]); // TODO: Sinnvoll verwenden
				//data[0] = 0xEF;
				//data[1] = 0xFF;
				//ringAdd(bpMsgState.writeBuf, buildMessage(0x175, 2, 0x57, data, 10));
				//ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_SIGNAL_TA_OFF]);

				//ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_VOL_PLUS]);
				//ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_REASED_17D]);

				//msg = buildTextMessage("Simu v03", 1);
				msg = buildTextMessage("Plüm2000_etwasLänger!", BP_DISPLAY_TIMEOUT_DEFAULT); // TODO aus Standardnachricht raus holen
				ringAdd(bpMsgState.writeBuf, msg);

				//uswusw...
				//msg = buildTextMessage("Yo → ß⮟→", 10);
				//ringAdd(bpMsgState.writeBuf, msg);

				//data[0] = 0x42;
				//ringAdd(bpMsgState.writeBuf, buildMessage(0x170, 1, 0x0B, data, 65));
				//data[0] = 0x80;
				//ringAdd(bpMsgState.writeBuf, buildMessage(0x175, 1, 0x3F, data, 70));

				stateFlags.bpDabActive = bp_DAB_active;

				printf("Activating our communication\r\n");
			}
			else
			{
				ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_DEACTIVATE]);
				ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_DEACTIVATE]);
				stateFlags.bpDabActive = bp_DAB_inactive;

				printf("Deactivating DAB mode\r\n");
			}

			setSendWait();
			break;

		case BP_MSG_BUT_6:
			buf[0] = 0xFE;
			buf[1] = 0xFF;
			msg = buildMessage(0x175, 2, 0x77, buf, 10);
			ringAdd(bpMsgState.writeBuf, msg);
			setSendWait();
			break;

		case BP_MSG_BUT_5:
			buf[0] = 0xFD;
			buf[1] = 0xFF;
			msg = buildMessage(0x175, 2, 0x77, buf, 10);
			ringAdd(bpMsgState.writeBuf, msg);
			setSendWait();
			break;

		case BP_MSG_BUT_4:
			cnt+=8;
		case BP_MSG_BUT_3:
			printf("Button 3 pressed... \r\n");

			if(++cnt > 0xFF)
			{
				cnt = 0;
			}


			buf2[0] = 0x00 + (cnt<<4);
			msg = buildMessage(0x175, 1, 0x70, buf2, 10);
			ringAdd(bpMsgState.writeBuf, msg);

			sprintf(buf,"0x%02X", buf2[0]);
			msg = buildTextMessage(buf, BP_DISPLAY_TIMEOUT_DEFAULT);
			ringAdd(bpMsgState.writeBuf, msg);
			setSendWait();
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
				msg = buildTextMessage(buf, BP_DISPLAY_TIMEOUT_DEFAULT);
				ringAdd(bpMsgState.writeBuf, msg);
			}

			setSendWait();
			break;

		case BP_MSG_BUT_DOWN:
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_BUT_DOWN]);
			setSendWait();

			controlBpMenu(MENU_CONTROL_DOWN);
			break;

		case BP_MSG_BUT_UP:
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_BUT_UP]);
			setSendWait();

			controlBpMenu(MENU_CONTROL_UP);
			break;

		case BP_MSG_BUT_LEFT:
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_BUT_LEFT]);
			setSendWait();

			controlBpMenu(MENU_CONTROL_LEFT);
			break;

		case BP_MSG_BUT_RIGHT:
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_BUT_RIGHT]);
			setSendWait();

			controlBpMenu(MENU_CONTROL_RIGHT);
			break;


		case BP_MSG_BUT_DSC:
			setBpMenu(MENU_DSC); // TODO rückgabewert abfragen
			break;
		//case BP_MSG_BUT_LD:
		//	break;
		case BP_MSG_BUT_AUD:
			printf("AUD gedrückt... \r\n");

			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_BUT_AUD]);
			setSendWait();
			break;

		case BP_MSG_BUT_RELEASED_17C:
			//TODO: Braucht das auch ein ACK?

			break;
		case BP_MSG_BUT_RELEASED_17D:
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_REASED_17D]);
			setSendWait();
			break;

		case BP_MSG_BUT_SCA:
			printf("\033[1;33mSCA gedrückt...\033[0m\r\n");
			buf[0] = cnt;
			buf[1] = 0x00;
			msg = buildMessage(0x175, 2, 0x56, buf, 10);
			ringAdd(bpMsgState.writeBuf, msg);
			sprintf(buf, "N %X", cnt);
			msg = buildTextMessage(buf, BP_DISPLAY_TIMEOUT_DEFAULT);
			ringAdd(bpMsgState.writeBuf, msg);
			cnt = cnt*2;
			setSendWait();
			break;

		case BP_MSG_BUT_PS:
			buf[0] = 0xEF;
			buf[1] = 0xFF;
			msg = buildMessage(0x175, 2, 0x57, buf, 10);
			ringAdd(bpMsgState.writeBuf, msg);
			setSendWait();
			break;
		case BP_MSG_BUT_MIX:
		case BP_MSG_BUT_GEO:
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_BUT_GEO]);
			setSendWait();
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
			// TODO VOL-Menü?
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_BUT_dB]);
			setSendWait();
			break;

		case BP_MSG_BUT_VOL_MIN:
			setBpMenu(MENU_VOL); // TODO rückgabewert abfragen

			printf("\033[1;33mVOL- gedrückt...\033[0m\r\n");
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_VOL_MIN]);
			setSendWait();
			break;

		case BP_MSG_BUT_VOL_PLUS:
			setBpMenu(MENU_VOL); // TODO rückgabewert abfragen
			printf("\033[1;33mVOL+ gedrückt...\033[0m\r\n");
			ringAdd(bpMsgState.writeBuf, bpMessages[BP_MSG_ACK_VOL_PLUS]);
			setSendWait();
			break;

		case BP_MSG_LEAVE_VOL:
			setBpMenu(MENU_NONE); // TODO rückgabewert abfragen
			printf("\033[1;33mVerlasse VOL-Menü\033[0m\r\n");
			//msg = buildTextMessage("VOL verl.", 10);
			ringAdd(bpMsgState.writeBuf, buildTextMessage(stateFlags.currentDisplayMessage, BP_DISABLE));
			setSendWait();
			break;

		case BP_MSG_ENTER_GEO_MIX:
			setBpMenu(MENU_GEO_MIX); // TODO rückgabewert abfragen
			printf("\033[1;33mGehe in GEO- oder MIX-Menü\033[0m\r\n");
			break;

		case BP_MSG_LEAVE_GEO_MIX:
			setBpMenu(MENU_NONE); // TODO rückgabewert abfragen
			printf("\033[1;33mVerlasse GEO- oder MIX-Menü\033[0m\r\n");

			ringAdd(bpMsgState.writeBuf, buildTextMessage(stateFlags.currentDisplayMessage, BP_DISABLE));
			setSendWait();
			break;

		case BP_MSG_ENTER_AUD:
			setBpMenu(MENU_AUD); // TODO rückgabewert abfragen
			printf("\033[1;33mGehe in AUD-Menü\033[0m\r\n");
			break;

		case BP_MSG_LEAVE_AUD:
			setBpMenu(MENU_NONE); // TODO rückgabewert abfragen
			printf("\033[1;33mVerlasse AUD-Menü\033[0m\r\n");
			break;

		case BP_MSG_ENTER_FAD:
			setBpMenu(MENU_FAD); // TODO rückgabewert abfragen
			printf("\033[1;33mGehe in FAD-Menü\033[0m\r\n");
			break;

		case BP_MSG_LEAVE_FAD:
			setBpMenu(MENU_NONE); // TODO rückgabewert abfragen
			printf("\033[1;33mVerlasse FAD-Menü\033[0m\r\n");
			ringAdd(bpMsgState.writeBuf, buildTextMessage(stateFlags.currentDisplayMessage, BP_DISABLE));
			setSendWait();
			break;

		case BP_MSG_LEAVE_MUTE:
			printf("\033[1;33mVerlasse MUTE-Menü\033[0m\r\n");
			break;

		default:
			printf("\033[1;33m(noch) nicht implementierter, bekannter Befehl, Index %d \033[0m\r\n", messageIndex);

			break;
	}

	return retVal;
}

// TODO: Also wie folgt: Der Eintragstext wird gezegt, der jeweilige Menüwert kommt aus der ptr-Funktion

/* Send a ring message, if available */
ringbuf_status_en sendRingMessage(bp_msg_state_dt * msgState, ringbuf_next_en nextItem)
{
	bp_msg_dt msg;

	if(bpMsgState.curReadMsg.messageState == MSG_INCOMPLETE)
	{
		printf("Debug: Already receiving a\r\n");
		return RINGBUF_WRITE_BUSY; // TODO: ungetestet: müsste anderer Status, er empfängt gerade was und sollte später erst senden
	}

	ringbuf_status_en ringbufStatus = ringGet(msgState->writeBuf, &msg, RINGBUF_KEEP_ITEM);

	printf("Debug: Sending %X;%d;%02X\r\n", msg.address, msg.dataLen, msg.command);

	if(ringbufStatus == RINGBUF_NO_DATA)
	{
		return RINGBUF_NO_DATA;
	}

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

		HAL_UART_Transmit_IT(&huart2, (uint8_t *)&bpMsgState.uart.tx_data[bpMsgState.uart.tx_data_pos], 1);
		// TODO: Anfragen, ob der jetzt überhaupt senden darf!!!!
		bpMsgState.uart.tx_data_pos++;

		/* Initiate the current read message as incomplete (timeout starts) */
		bpMsgState.curReadMsg.messageState = MSG_INCOMPLETE;
		bpMsgState.curReadMsg.timeStamp_ms = HAL_GetTick();

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

	if(ringGet(bpMsgState.debugBuffer, &msg_debug1, RINGBUF_NEXT_ITEM) == RINGBUF_OK)
	{
		if(msg_debug1.address == 0x178 || msg_debug1.address == 0x17C || msg_debug1.address == 0x17D) // Just dump other addresses
		{
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
	}

#ifdef DAB_DEBUG_ACTIVE
	if(ringGet(bpMsgState_DAB.debugBuffer, &msg_debug2, RINGBUF_NEXT_ITEM) == RINGBUF_OK)
	{
		if(msg_debug2.address == 0x170 || msg_debug2.address == 0x175) // Just dump other addresses
		{
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
	}
#endif

}
