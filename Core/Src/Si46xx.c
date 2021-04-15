/*
 * Si46xx.c
 *
 * Generic functions to work with Si46xx
 *
 *  Created on: 21.03.2021
 *      Author: kai

TODO:
- Generische Funktion, die SPI-Befehl absetzt
- Generische Funktion, die Antwort-Befehl absetzt und Antwort auswertet
	- die Länge der Rückgaben und so müsste man dann noch als Parameter übergeben (in irgendwie eine Config packen?

Befehl Senden IT -> IT vom Si46, wenn Takko, dann Befehl SPI holen

Programmierung über DMA UART->SPI??

so zS bauen?
   Configure MEMS: data rate, power mode, full scale, self test and axes
    ctrl = (uint16_t) (lis302dl_initstruct.Output_DataRate | lis302dl_initstruct.Power_Mode | \
                       lis302dl_initstruct.Full_Scale | lis302dl_initstruct.Self_Test | \
                       lis302dl_initstruct.Axes_Enable);

Strukt bauen, der
- den Pointer zur Funktion enthält
- die Angabe über eine Rückgabeanzahl
- und eine Funktion zum antwort auswerten
- eine Typische Wartezeit? ODer besser Interrupt nehmen?
oder:
- einfache Funktionen nur über SPI-Befehl, da immer 2 byte. aber nicht so cool, oder?

 */
#include "Si46xx.h"

struct Si46xx_Config Si46xxCfg;

#define SI46XX_CS_ON()  HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_RESET)
#define SI46XX_CS_OFF() HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_SET)

#define SI46XX_INTB_STATE HAL_GPIO_ReadPin(Si46xx_INTB_GPIO_Port, Si46xx_INTB_Pin)

#define SI46XX_RST_ON()  HAL_GPIO_WritePin(Si46xx_RSTB_GPIO_Port, Si46xx_RSTB_Pin, GPIO_PIN_RESET); // Reset LOW
#define SI46XX_RST_OFF() HAL_GPIO_WritePin(Si46xx_RSTB_GPIO_Port, Si46xx_RSTB_Pin, GPIO_PIN_SET);
#define SI46XX_RST_WAIT_BEFORE 10 // ms
#define SI46XX_RST_WAIT_AFTER   4 // 3.2ms

#define SI46XX_CURRENT_WAIT_TIME (HAL_GetTick() - Si46xxCfg.timeoutVal)
#define SI46XX_DEFAULT_SPI_WAIT 10 // ms, when looping and polling for an SPI ready state from uC or Si46xx

char firmwareStepTexts[FW_STEP_SIZE][20] =
{
		"NONE",
		"BOOTLOADER_PATCH",
		"FIRMWARE"
};


/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_GPIO_TogglePin(LED_ORANGE_Port, LED_ORANGE_Pin);
	if(GPIO_Pin == Si46xx_INTB_Pin)
	{
		Si46xxCfg.interruptFlag = Si46xx_INT_FLAG_SET;
	}
}*/

void Si46xx_Set_AnswerState(enum Si46xx_States current, enum Si46xx_States after, uint16_t answerBytes);

uint8_t spiBuffer[4096];

/**
 * Send command via SPI
 */
HAL_StatusTypeDef Si46xx_SPIsend(SPI_HandleTypeDef * hspi, uint8_t * data, uint16_t len)
{

	// TODO Abfrage, ob sendebereit
	memcpy(spiBuffer, data, len);


	SI46XX_CS_ON();
	//HAL_SPI_TransmitReceive_IT(hspi, spiBuffer, (uint8_t *)NULL, len); // geht niht, NULL macht HAL_ERROR
	HAL_SPI_Transmit_IT(hspi, spiBuffer, len); //TODO: keine Pausen...
	//SI46XX_CS_OFF(); -> Durch Interrupt...

	return HAL_OK;
}

/* Callback for finished transmit */
void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef * hspi)
{
	/* Deactivate chip select */
	SI46XX_CS_OFF();
}

/**
 * Get status reply from last command
 */
HAL_StatusTypeDef Si46xx_SPIgetStatus(SPI_HandleTypeDef * hspi, uint8_t * data, uint16_t len)
{
	if(len < 4)
	{
		len = 4;
	}

	uint8_t zeroes[1+len]; // {0, ??? TODO dynamisch mit memalloc oder sowas
	memset(zeroes, 0, 1+len);

	HAL_StatusTypeDef state = HAL_OK;
	// TODO Abfrage, ob sendebereit

	//HAL_SPI_STATE_BUSY_RX

	SI46XX_CS_ON();
	state = HAL_SPI_TransmitReceive(hspi, zeroes, data, len+1, 100); // TODO irgendwann als IT Version? TODO status ist kein si46xxx.... muss gewandelt!!
	SI46XX_CS_OFF();

	//printf("State: %d\r\n",state);

	return state;
}

HAL_StatusTypeDef Si46xx_InitConfiguration(SPI_HandleTypeDef * hspi)
{
	SI46XX_CS_OFF();
	SI46XX_RST_ON();

	// Initial config values
	struct Si46xx_Init_Values init =
	{
			.CTS_InterruptEnable = Si46xx_ENABLE,
			.CLK_MODE = Si46xx_XOSC,
			.XOSC_TR_SIZE = 0x07,
			.IBIAS = 0x48,
			.IBIAS_RUN = 0,
			.XTAL_Freq = 19200000, // Hz (0x0124F900 ??)
			.CTUN = 0x1F
	};

	Si46xxCfg.initConfig = init;
	Si46xxCfg.hspi = hspi;

	Si46xxCfg.intstate = SI46XX_INTB_STATE;

	Si46xxCfg.state = Si46xx_STATE_SAFE_OFF; //Si46xx_STATE_STARTUP;

	Si46xxCfg.firmwareBuf = fwBufferInit(8192);
	Si46xxCfg.firmwareBuf->fwStep = FW_NONE;
//	Si46xxCfg.firmwareBuf->fwStep = FW_BOOTLOADER_PATCH;

	return HAL_OK;
}

HAL_StatusTypeDef Si46xx_Send_PowerUp(void)
{
	HAL_StatusTypeDef state = HAL_OK;
	uint8_t data[16];

	// Prepare data into SPI buffer
	data[0]  = SI46XX_POWER_UP;
	data[1]  = (Si46xxCfg.initConfig.CTS_InterruptEnable << 7);
	data[2]  = (Si46xxCfg.initConfig.CLK_MODE << 4) | Si46xxCfg.initConfig.XOSC_TR_SIZE;
	data[3]  = (Si46xxCfg.initConfig.IBIAS & 0x7F);
	data[4]  =  Si46xxCfg.initConfig.XTAL_Freq 		   & 0xFF;
	data[5]  = (Si46xxCfg.initConfig.XTAL_Freq >>  8 ) & 0xFF;
	data[6]  = (Si46xxCfg.initConfig.XTAL_Freq >> 16 ) & 0xFF;
	data[7]  = (Si46xxCfg.initConfig.XTAL_Freq >> 24 ) & 0xFF;
	data[8]  = (Si46xxCfg.initConfig.CTUN & 0x3F);
	data[9]  = 0x00 | (1 << 4);
	data[10] = 0x00;
	data[11] = 0x00;
	data[12] = 0x00;
	data[13] = (Si46xxCfg.initConfig.IBIAS_RUN & 0x7F);
	data[14] = 0x00;
	data[15] = 0x00;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 16);

	Si46xxCfg.timeoutVal = HAL_GetTick(); // For first waiting time before triggering RST pin

	return state;
}

HAL_StatusTypeDef Si46xx_Send_LoadInit(void)
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_OK;

	data[0] = SI46XX_LOAD_INIT;
	data[1] = 0x00;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);

	return state;
}

HAL_StatusTypeDef Si46xx_Send_Boot(void)
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_OK;

	data[0] = SI46XX_BOOT;
	data[1] = 0x00;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);

	return state;
}

HAL_StatusTypeDef Si46xx_Send_GetSysState(void)
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_OK;

	data[0] = SI46XX_GET_SYS_STATE;
	data[1] = 0x00;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);

	return state;
}

void Si46xx_Send_Reset(void)
{
	SI46XX_RST_ON();

	Si46xxCfg.timeoutVal = HAL_GetTick();
	Si46xxCfg.state = Si46xx_STATE_STARTUP;
	Si46xxCfg.firmwareBuf->fwStep = FW_BOOTLOADER_PATCH;
}

HAL_StatusTypeDef Si46xx_HostLoad_Flash(uint8_t * bufferPtr, uint32_t size)
{
	HAL_StatusTypeDef state = HAL_OK;
	//uint8_t data[4096] = {0,};
	uint32_t i = 0;

	if(size > 4092)
	{
		state = HAL_ERROR;
		return state;
	}

	spiBuffer[0] = SI46XX_HOST_LOAD;
	spiBuffer[1] = 0x00;
	spiBuffer[2] = 0x00;
	spiBuffer[3] = 0x00;

	for(i=0; i<size; i++)
	{
		spiBuffer[i+4] = *bufferPtr;

		bufferPtr++;
	}

	SI46XX_CS_ON();
	state = HAL_SPI_Transmit_IT(Si46xxCfg.hspi, spiBuffer, size + 4);
	// CS_OFF by INT

	return state;
}

firmwareBuffer_state_dt Si46xx_GetFW(uint8_t * buffer, uint32_t length)
{
	if(Si46xxCfg.firmwareBuf != NULL)
	{
		if(fwBufferCurrentSize(Si46xxCfg.firmwareBuf) + length > Si46xxCfg.firmwareBuf->bufSize)
		{
			return FWBUF_SIZE; // TODO: der holt ihn dann ein und bekommt es scheinbar auch ncht hin, aufzuholen... Kann man USB irgenwie delayen? usbd_cdc.c macht irgendwie direkt ein OK ...
			// wenn kein Workaround gefunden werden kann: Hier einen Fehler werfen

			// TODO: Größe kann 0 sein, wenn die andere routine busy ist. unbedingt behandeln!
		}
		fwBufferWrite(Si46xxCfg.firmwareBuf, buffer, length);
		Si46xxCfg.firmwareBuf->receiveTimestamp = HAL_GetTick(); // for timeout measurement
	}

	return FWBUF_OK;
}

void Si46xx_Set_AnswerState(enum Si46xx_States current, enum Si46xx_States after, uint16_t answerBytes)
{
	Si46xxCfg.answerBytes = answerBytes;

	Si46xxCfg.timeoutVal = HAL_GetTick();

	Si46xxCfg.stateBefore = current;
	Si46xxCfg.state       = Si46xx_STATE_ANSWER;
	Si46xxCfg.stateAfter  = after;
}

void progress_StatusBytes(uint8_t * data)
{
	struct Si46xx_Status_Values * status = &Si46xxCfg.currentStatus;

	/* Progress into status variables */
	status->CTS      = (data[0] & 0x80)>>7;
	status->ERR_CMD  = (data[0] & 0x40)>>6;
	status->PUP      = (data[3] & 0xC0)>>6;
	status->RFFE_ERR = (data[3] & 0x20)>>5;
	status->REPOFERR = (data[3] & 0x08)>>3;
	status->CMDOFERR = (data[3] & 0x04)>>2;
	status->ARBERR   = (data[3] & 0x02)>>1;
	status->ERRNR    = (data[3] & 0x01);
}

void Si46xx_Tasks()
{
	uint32_t currentSize; // TODO Temp
	uint32_t currentTimeout = 0; // TODO Temp

	switch(Si46xxCfg.state)
	{
		case Si46xx_STATE_SAFE_OFF:
			SI46XX_RST_ON();
			break;

		/* Release reset after a mininmum time */
		case Si46xx_STATE_STARTUP:
			if(SI46XX_CURRENT_WAIT_TIME > SI46XX_RST_WAIT_BEFORE /*&& HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1*/)
			{
				printf("\033[1;36mSi46xx_Tasks: STARTUP\033[0m\r\n");

				SI46XX_RST_OFF();
				Si46xxCfg.timeoutVal = HAL_GetTick();

				Si46xxCfg.state = Si46xx_STATE_INIT;
			}
			break;

		case Si46xx_STATE_INIT:
			if(SI46XX_CURRENT_WAIT_TIME < SI46XX_RST_WAIT_AFTER)
			{
				break;
			}

			printf("\033[1;36mSi46xx_Tasks: INIT\033[0m\r\n");

			if(Si46xx_Send_PowerUp() == HAL_OK)
			{
				Si46xx_Set_AnswerState(Si46xxCfg.state, Si46xx_STATE_LOAD_INIT, 4);
			}
			else
			{
				Si46xxCfg.state = Si46xx_STATE_SAFE_OFF;
			}
			break;

		case Si46xx_STATE_ANSWER:

			// TODO: Workaround TEST
			if(Si46xxCfg.hspi->State != HAL_SPI_STATE_READY) // NACHDEM SPI fertig ist soll ja gewartet werden....
			{
				Si46xxCfg.timeoutVal = HAL_GetTick();
			}
			if(SI46XX_CURRENT_WAIT_TIME < 5)
			{
				break;
			}

			if(Si46xxCfg.stateBefore == Si46xx_STATE_LOAD_FW_FROM_FLASH)
			{
				HAL_GPIO_TogglePin(LED_ORANGE_Port, LED_ORANGE_Pin);
			}

			if(Si46xxCfg.stateBefore == Si46xx_STATE_INIT && SI46XX_CURRENT_WAIT_TIME < 20)
			{
				break;
			}

			if(Si46xxCfg.stateBefore == Si46xx_STATE_BOOT && SI46XX_CURRENT_WAIT_TIME < 300)
			{
				break;
			}
			//if(Si46xxCfg.interruptFlag == Si46xx_INT_FLAG_SET)
			//if(Si46xxCfg.intstate != SI46XX_INTB_STATE)
			if(Si46xxCfg.hspi->State == HAL_SPI_STATE_READY) // TODO Temporätr
				// Die lange ZEit benötigt er wohl nur beim starten, später dann nicht mehr...
			{
				uint8_t data[1 + Si46xxCfg.answerBytes]; // TODO: Geht das??
				//uint8_t * data;
				/* Answer is ready after interrupt */ //TODO interrupt/polling driven Auswahl? Bei fehlern auch INT?
				Si46xxCfg.intstate = SI46XX_INTB_STATE;

				Si46xxCfg.timeoutVal = HAL_GetTick(); // TODO temp weiter warten, das INT-Flag tut irgendwie noch nicht

				//data = malloc(Si46xxCfg.answerBytes); // TODO: lässt malloc die auch selbst wieder los?
				// free(data);

				if(Si46xx_SPIgetStatus(Si46xxCfg.hspi, data, Si46xxCfg.answerBytes) == HAL_OK) // TODO ohne Else loopt er dann, wenn er busy ist
				{
					progress_StatusBytes(&data[1]); // TODO: Wenn alles FF ist ,auch noch warten

					/* If there are more answer bytes: back to buffer */
					if(sizeof(data) > 5)
					{
						uint8_t dataSize = sizeof(data)-5;
						if(dataSize > sizeof(spiBuffer))
						{
							dataSize = sizeof(spiBuffer);
						}
						memcpy(spiBuffer, (data+5), dataSize);
					}

					if // Reset device
					(
						Si46xxCfg.currentStatus.ARBERR == Si46xx_ERR_ERROR || /* An arbiter overflow has occurred. The only way to recover is for the user to reset the chip. */
						Si46xxCfg.currentStatus.ERRNR  == Si46xx_ERR_ERROR /* Fatal error has occurred. The only way to recover is for the user to reset the chip.*/
					)
					{
						printf("ARBERR/ERRNR: Reseting device\r\n");
						Si46xx_Send_Reset();
						break;
					}
					else if // Resend last command
					(
							Si46xxCfg.currentStatus.CMDOFERR == Si46xx_ERR_ERROR || /* The command interface has overflowed, and data has been lost */
							Si46xxCfg.currentStatus.REPOFERR == Si46xx_ERR_ERROR    /* The reply interface has underflowed, and bad data has been returned to the user */
					)
					{
						Si46xxCfg.state       = Si46xxCfg.stateBefore;
						Si46xxCfg.stateBefore = Si46xx_STATE_SAFE_OFF; // als 0
						Si46xxCfg.stateAfter  = Si46xx_STATE_SAFE_OFF; // als 0

						printf("CMDOFERR/REPOFERR: Going to previous state\r\n");

						break;
					}

					if(Si46xxCfg.currentStatus.CTS == Si46xx_CTS_READY) // Ready for next command
					{
						Si46xxCfg.state 	  = Si46xxCfg.stateAfter;
						Si46xxCfg.stateBefore = Si46xx_STATE_SAFE_OFF; // als 0
						Si46xxCfg.stateAfter  = Si46xx_STATE_SAFE_OFF; // als 0

						// TODO: Aber wenn andere Befehle ausgeführt werden? ERR_CMD??
					}

				}
			}
			break;

		case Si46xx_STATE_LOAD_INIT: /* Prepare to load firmware */
			if(SI46XX_CURRENT_WAIT_TIME < SI46XX_DEFAULT_SPI_WAIT)
			{
				break;
			}

			/* Go to next state if all firmware is programmed */
			if(Si46xxCfg.firmwareBuf->fwStep > FW_FIRMWARE)
			{
				printf("Trying to boot...\r\n");

				Si46xxCfg.state = Si46xx_STATE_BOOT;
				Si46xxCfg.firmwareBuf->fwStep = FW_NONE;
				break;
			}

			switch(Si46xx_Send_LoadInit())
			{
				case HAL_OK:
					printf("Firmware: %s\r\n", firmwareStepTexts[Si46xxCfg.firmwareBuf->fwStep]);
					if(Si46xxCfg.firmwareBuf->fwStep == FW_BOOTLOADER_PATCH)
					{
						Si46xxCfg.fwBufferPtr = (uint8_t *) &Si46xx_Rom00Patch016;
						Si46xxCfg.fwBufferSize = sizeof(Si46xx_Rom00Patch016);
						Si46xx_Set_AnswerState(Si46xxCfg.state, Si46xx_STATE_LOAD_FW_FROM_FLASH, 4);
					}
					else if(Si46xxCfg.firmwareBuf->fwStep == FW_FIRMWARE)
					{
						Si46xxCfg.fwBufferPtr = (uint8_t *) &Si46xx_Firmware;
						Si46xxCfg.fwBufferSize = sizeof(Si46xx_Firmware);
						Si46xx_Set_AnswerState(Si46xxCfg.state, Si46xx_STATE_LOAD_FW_FROM_FLASH, 4);

						/* für usb ab hier...
						Si46xxCfg.firmwareBuf->receiveTimestamp = 0; // inactive until first transfer

						//fwBufferClear(Si46xxCfg.firmwareBuf); // TODO: warum ist der Readint denn ncht 0??
						//WORKAROUND testen...
						Si46xxCfg.firmwareBuf->readInd = 0;
						Si46xxCfg.firmwareBuf->writeInd = 0;

						Si46xxCfg.firmwareBuf->byteCount = 0;


						Si46xx_Set_AnswerState(Si46xxCfg.state, Si46xx_STATE_LOAD_FIRMWARE_FROM_BUFFER, 4);
						*/
					}
					else
					{
						// Shouldn't come here, errorhandling
					}
					break;

				case HAL_BUSY:
					Si46xxCfg.timeoutVal = HAL_GetTick();
					break;

				default: // Problems...
					Si46xx_Send_Reset();
					break;
			}
			break;

		case Si46xx_STATE_LOAD_FW_FROM_FLASH:
			if(SI46XX_CURRENT_WAIT_TIME < 4)
			{
				break;
			}

			switch(Si46xx_HostLoad_Flash(Si46xxCfg.fwBufferPtr, (Si46xxCfg.fwBufferSize > 4092 ? 4092 : Si46xxCfg.fwBufferSize)))
			{
				case HAL_OK:
					if(Si46xxCfg.fwBufferSize > 4092) // Still packages to send
					{
						Si46xxCfg.fwBufferPtr = Si46xxCfg.fwBufferPtr + 4092;
						Si46xxCfg.fwBufferSize -= 4092;
						Si46xx_Set_AnswerState(Si46xxCfg.state, Si46xx_STATE_LOAD_FW_FROM_FLASH, 4);
					}
					else // Nothing more to send, go back to load with next FW package or finish
					{
						printf("Firmware from flash written: %s\r\n", firmwareStepTexts[Si46xxCfg.firmwareBuf->fwStep]);

						Si46xxCfg.fwBufferSize = 0;
						Si46xxCfg.firmwareBuf->fwStep++;
						Si46xx_Set_AnswerState(Si46xxCfg.state, Si46xx_STATE_LOAD_INIT, 4);
					}
					break;

				case HAL_BUSY:
					Si46xxCfg.timeoutVal = HAL_GetTick();
					break;
				default: // Problems... Try again
					Si46xxCfg.timeoutVal = HAL_GetTick();
					Si46xxCfg.state = Si46xx_STATE_LOAD_INIT;
					break;
			}

			break;

		case Si46xx_STATE_LOAD_FIRMWARE_FROM_BUFFER:
			//Si46xxCfg.state = Si46xx_STATE_LOAD_FIRMWARE_WAIT;

			//Si46xxCfg.firmwareBuf = fwBufferInit(8192);
			//Si46xxCfg.firmwareBuf->fwStep = FW_BOOTLOADER_PATCH; // Start by transferring the patch
			//Si46xxCfg.firmwareBuf->fwStep++; // Start with first / next firmware file
			//Si46xxCfg.timeoutVal = HAL_GetTick();



			// Ringbuffer bauen, der alles empfängt vom USB
			// SPI soll immer Senden, wenn das mit Faktor 4 im Buffer liegt
			// Ringbuffer wieder kaputt machen, wenn fertig

			currentSize = fwBufferCurrentSize(Si46xxCfg.firmwareBuf); // TODO temp
// !!!!			manchmal kommen hier negateive Ergebnisse raus: warum? ist dann natürlich große Zahl
			//currentTick = HAL_GetTick();
			//currentTimeout = (currentTick - Si46xxCfg.firmwareBuf->receiveTimestamp);// TODO temp

			//if(HAL_GetTick() > Si46xxCfg.firmwareBuf->receiveTimestamp)
			// Workaround: Im interrupt wird der gesetzt, wenn ich x-x abziehe, habe ich nciht 0, sondern -1
			if(HAL_GetTick() > Si46xxCfg.firmwareBuf->receiveTimestamp)
			{
				currentTimeout = (HAL_GetTick() - Si46xxCfg.firmwareBuf->receiveTimestamp);// TODO temp
			}
			else
			{
				//Si46xxCfg.firmwareBuf->receiveTimestamp = 0;
			}

			if
			(
			  (
			    currentSize >= (4096-4) ||
				(
			      (currentTimeout > 200) && /* TODO Zeit definieren, sorgt aber auch für ein riesiges Gap in der Übertragung! */
				  (Si46xxCfg.firmwareBuf->receiveTimestamp != 0)
				)
			  ) &&
				(Si46xxCfg.hspi->State == HAL_SPI_STATE_READY)
			) // TODO: Timeout sinnvoll definieren -> der muss aber erstmal ne erste Aussage haben...
			{
				uint8_t data[4096] = {0,};
				uint32_t i = 0;

				data[0] = SI46XX_HOST_LOAD;
				data[1] = 0x00;
				data[2] = 0x00;
				data[3] = 0x00;

				for(i=4; i<sizeof(data); i++)
				{
					/* Get next byte or break if empty */
					if(fwBufferGet(Si46xxCfg.firmwareBuf, &data[i]) == FWBUF_NO_DATA)
					{
						break;
					}

					Si46xxCfg.firmwareBuf->byteCount++; // TODO test
				}

				SI46XX_CS_ON();
				HAL_SPI_Transmit_IT(Si46xxCfg.hspi, data, i);
				// CS_OFF by INT

				if(i < 4096) // TODO: Wenn letzte Botschaft kleiner ist, aber was ist, wenn exakt 4kB-Msg übertragen werden?
				{
					Si46xxCfg.firmwareBuf->fwStep++;
					// Start with the next FW file
					//Si46xx_Set_AnswerState(Si46xxCfg.state, Si46xx_STATE_ANSWER, Si46xx_STATE_LOAD_FIRMWARE, 4);
					Si46xx_Set_AnswerState(Si46xxCfg.state, Si46xx_STATE_LOAD_INIT, 4);

					printf("Sent fw bytes: %ld\r\n", Si46xxCfg.firmwareBuf->byteCount);

					//TODO: der Buffer muss noch kaputt gemacht werden..?
				}
				else // Get SPI state after 4096 bytes
				{
					Si46xx_Set_AnswerState(Si46xxCfg.state, Si46xx_STATE_LOAD_FIRMWARE_FROM_BUFFER, 4);
				}
			}
			break;

		case Si46xx_STATE_BOOT:
			/*if(SI46XX_CURRENT_WAIT_TIME < 300)
			{
				break;
			}*/

			switch(Si46xx_Send_Boot())
			{
				case HAL_OK:
					Si46xx_Set_AnswerState(Si46xxCfg.state, Si46xx_STATE_IDLE, 4);
					break;

				case HAL_BUSY:
					Si46xxCfg.timeoutVal = HAL_GetTick();
					break;

				default: // Problems...
					Si46xx_Send_Reset();
					break;
			}
			break;

		case Si46xx_STATE_IDLE:
			if(SI46XX_CURRENT_WAIT_TIME < 1500)
			{
				break;
			}

			printf("Aktueller State: \r\n");
			printf("CTS: %d, PUP: %d, ERR_CMD: %d\r\n", Si46xxCfg.currentStatus.CTS, Si46xxCfg.currentStatus.PUP, Si46xxCfg.currentStatus.ERR_CMD);
			printf("Image: %d\r\n", spiBuffer[0]);

			switch(Si46xx_Send_GetSysState())
			{
				case HAL_OK:
					Si46xx_Set_AnswerState(Si46xxCfg.state, Si46xx_STATE_IDLE, 5);
					break;

				case HAL_BUSY:
					Si46xxCfg.timeoutVal = HAL_GetTick();
					break;

				default: // Problems...
					//Si46xx_Send_Reset();
					break;
			}
			break;

		//case DAB konfiguieren: analogen Ausgang, Lautstärke 100%
		//case Sendersuchlauf (beim ersten MAl aufrufen, wnen nichts abgespeichert ist, oder manuell aufgerufen wird?)
		//case Tune auf einen Sender (letzten gespeicherten oder 0)
		//case Change Frequenz und Channel und so (später dann mit den Pfeiltasten...
		//case Get Station Text: Lied und sowas
	}
}



