/*
 * Si46xx.c
 *
 * Generic functions to work with Si46xx
 *
 *  Created on: 21.03.2021
 *      Author: kai

 */
#include "Si46xx.h"

struct Si46xx_Config Si46xxCfg;
uint8_t spiBuffer[4096];

Si46xx_msg_dt currentWorkingMsg;

/* Private functions ------------------------------------------- */
Si46xx_statusType Si46xx_SPIgetAnalyzeStatus(uint8_t * data, uint16_t len);
extern void Si46xx_Boot(void);

/* Helper functions -------------------------------------------- */
void set_Si46xx_ISR(void)
{
	Si46xxCfg.isrState = ISR_SET;
}


/* Message functions to be called from stack ------------------- */
HAL_StatusTypeDef Si46xx_Msg_RefreshSysState_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		data[0] = SI46XX_GET_SYS_STATE;
		data[1] = 0x00;

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);

		// As maximum wait time:
		//Si46xx_SetWaitTime(100);
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_RefreshSysState_receiveFunc()
{
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(spiBuffer, 5);

	if(state != Si46xx_OK)
	{
		return state;
	}

	Si46xxCfg.image = spiBuffer[5];

	printf("\033[1;36mSi46xx: Image %d\033[0m\r\n", Si46xxCfg.image);

	return state;
}

/* SI46XX_MSG_GET_DIGITAL_SERVICE_LIST */
HAL_StatusTypeDef Si46xx_Msg_GetDigitalServiceList_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		// If tune is not marked as complete, check once if it is now
		if(Si46xxCfg.deviceStatus.STCINT == Si46xx_STCINT_INCOMPLETE)
		{
			/*state = */Si46xx_SPIgetAnalyzeStatus(spiBuffer, 5); // State nicht aktualisieren, soll busy bleiben
		}

		if(Si46xxCfg.deviceStatus.STCINT == Si46xx_STCINT_COMPLETE)
		{
			data[0] = SI46XX_GET_DIGITAL_SERVICE_LIST;
			data[1] = 0x00;

			state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);
		}

		// As maximum wait time:
		//Si46xx_SetWaitTime(100); // TODO Timeout und so...
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_GetDigitalServiceList_receiveFunc()
{
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(spiBuffer, 6);
	uint16_t length = 0;

	if(state != Si46xx_OK)
	{
		return state;
	}

	length = (uint16_t) spiBuffer[5] + (spiBuffer[6] << 8);
	printf("\033[1;36mSi46xx: Size of service list: %d bytes \033[0m\r\n", length);

	//Si46xxCfg.image = spiBuffer[5];

	state = Si46xx_SPIgetAnalyzeStatus(spiBuffer, 6 + length);

	if(state != Si46xx_OK || length == 0)
	{
		return state;
	}

	/* TODO: Digitale Serviceliste auswerten:
	 * 8.5
 Finding a Digital Service
Before starting a data service, the host must collect information about the services that exist in the ensemble. This is
done using the GET_DIGITAL_SERVICE_LIST host command. The service list definitions for DAB is described in
“8.5.1. DAB/DMB Radio Service List”, the following table shows the format of the GET_DIGITAL_SERVICE_LIST
host command.
	 *
	 */

	uint8_t * bufPtr = spiBuffer + 5; // Point to List Size
	length = bufPtr[0] + (bufPtr[1] << 8);

	uint8_t numberServices = bufPtr[4];
	printf("\033[1;36mSi46xx: # of services: %d \033[0m\r\n", numberServices);

	uint8_t position = 8; // initial...
	bufPtr += position;

	// Loop through services
	for(uint8_t i=0; i<numberServices; i++)
	{

		uint32_t serviceID = bufPtr[0] + (bufPtr[1] << 8) + (bufPtr[2] << 16) + (bufPtr[3] << 24);
		uint8_t numberComponents = bufPtr[5] & 0x0F;
		char serviceLabel[16+1];
		memcpy(serviceLabel, &bufPtr[8], 16);
		serviceLabel[16] = '\0';

		printf("\033[1;36mSi46xx: ServiceID: %lu, Label: %s \033[0m\r\n", serviceID, serviceLabel);
		printf("\033[1;36mSi46xx: # of components: %d \033[0m\r\n", numberComponents);

		// Jump to components
		//position += 24;
		bufPtr += 24;

		// Loop through components in the service
		for(uint8_t j=0; j<numberComponents; j++)
		{
			// Adjust pointer to current component
			//bufPtr += position;

			uint16_t componentID = bufPtr[0] + (bufPtr[1] << 8);

			printf("\033[1;36mSi46xx:       ComponentID: %u\033[0m\r\n", componentID);

			// Jump to next component in current service block
			//position += 4;
			bufPtr += 4;
		}
	}

	return state;
}

//SI46XX_MSG_DAB_TUNE_FREQ
HAL_StatusTypeDef Si46xx_Msg_DABtuneFreq_sendFunc()
{
	uint8_t data[6];
	HAL_StatusTypeDef state = HAL_BUSY;

	// Check configuration
	if(Si46xxCfg.freqIndex > 47)
	{
		return HAL_ERROR;
	}

	Si46xxCfg.deviceStatus.STCINT = Si46xx_STCINT_COMPLETE; // für den ersten... TODO

	// No other tuning should be running
	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY  && Si46xxCfg.deviceStatus.STCINT == Si46xx_STCINT_COMPLETE)
	{
		data[0] = SI46XX_DAB_TUNE_FREQ;
		data[1] = 0x00;
		data[2] = Si46xxCfg.freqIndex;
		data[3] = 0x00;

		/* ANTCAP[15:0] (data[4] und data[5])
    		When non-zero this parameter sets the antenna tuning capacitor value to (ANTCAP-1)*250 fF (31.75 pF Max).
    		If left at 0, the capacitor is automatically determined.
			Type:		U16
			Min:		0
			Max:		128
			Default:	0
			Unit:		250 fF
		 */
		data[4] = 0;
		data[5] = 0;

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 6);

		// As maximum wait time:
		//Si46xx_SetWaitTime(100); // TODO Timeout und so...
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_DABtuneFreq_receiveFunc()
{
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(spiBuffer, 4);

	if(state != Si46xx_OK)
	{
		return state;
	}

	//Si46xxCfg.image = spiBuffer[5];

	if(Si46xxCfg.deviceStatus.STCINT == Si46xx_STCINT_INCOMPLETE)
	{
		printf("\033[1;36mSi46xx: Tune not complete... \033[0m\r\n");
	}
	else
	{
		printf("\033[1;36mSi46xx: Tune complete! \033[0m\r\n");
	}


	return state;
}


// SI46XX_MSG_GET_FREQ_LIST
HAL_StatusTypeDef Si46xx_Msg_GetFreqList_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		data[0] = SI46XX_GET_FREQ_LIST;
		data[1] = 0x00;

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);

		// As maximum wait time:
		//Si46xx_SetWaitTime(100); // TODO Timeout und so... muss eine andere Funktion gebaut werden
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_GetFreqList_receiveFunc()
{
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(spiBuffer, 200);

	if(state != Si46xx_OK)
	{
		return state;
	}

	//Si46xxCfg.image = spiBuffer[5];


	printf("\033[1;36mSi46xx: Frequencies: %d\033[0m\r\n", spiBuffer[5]);

	for(uint8_t i=0x08; i<0xC7; i+=4)
	{
		printf("\033[1;36mSi46xx: Freq at %d: %lu \033[0m\r\n", i,
				(uint32_t) spiBuffer[i+1]+(spiBuffer[i+2]<<8)+(spiBuffer[i+3]<<16)+(spiBuffer[i+4]<<24));
	}

	return state;
}



const Si46xx_msg_dt Si46xx_messages[SI46XX_MSG_SIZE] = {
		{0}, /* SI46XX_MSG_NONE */
		{
				.msgIndex    = SI46XX_MSG_REFRESH_SYS_STATE,
				.sendFunc    = Si46xx_Msg_RefreshSysState_sendFunc,
				.receiveFunc = Si46xx_Msg_RefreshSysState_receiveFunc
		},	/* SI46XX_MSG_REFRESH_SYS_STATE */
		{
				.msgIndex    = SI46XX_MSG_GET_DIGITAL_SERVICE_LIST,
				.sendFunc    = Si46xx_Msg_GetDigitalServiceList_sendFunc,
				.receiveFunc = Si46xx_Msg_GetDigitalServiceList_receiveFunc
		},  /* SI46XX_MSG_GET_DIGITAL_SERVICE_LIST */
		{
				.msgIndex    = SI46XX_MSG_DAB_TUNE_FREQ,
				.sendFunc    = Si46xx_Msg_DABtuneFreq_sendFunc,
				.receiveFunc = Si46xx_Msg_DABtuneFreq_receiveFunc
		},  /* SI46XX_MSG_DAB_TUNE_FREQ */
		{
				.msgIndex    = SI46XX_MSG_GET_FREQ_LIST,
				.sendFunc    = Si46xx_Msg_GetFreqList_sendFunc,
				.receiveFunc = Si46xx_Msg_GetFreqList_receiveFunc
		}	/* SI46XX_MSG_GET_FREQ_LIST     */
};

extern Si46xx_state_en Si46xx_Boot_Tasks(void); // TODO temporär
void Si46xx_TestFunctions(uint8_t param)
{
	switch(param)
	{
		case 1:
			cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[SI46XX_MSG_REFRESH_SYS_STATE]);
			break;

		case 2:
			cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[SI46XX_MSG_GET_FREQ_LIST]);
			break;

		case 3:
			Si46xxCfg.freqIndex = 2; // TODO Sollte dann auf Kanal 5C liegen
			cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[SI46XX_MSG_DAB_TUNE_FREQ]);
			cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[SI46XX_MSG_GET_DIGITAL_SERVICE_LIST]);
			break;

		case 4:
			cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[SI46XX_MSG_GET_DIGITAL_SERVICE_LIST]);
			break;

	}
}
GPIO_PinState pinBuf; // TODO temporär

void Si46xx_Tasks(void)
{

	if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1 && HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)!= pinBuf)
	{
		pinBuf=HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
		//Si46xx_msg_dt msgTest = {.cmd = SI46XX_GET_SYS_STATE };
		cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[SI46XX_MSG_REFRESH_SYS_STATE]);
		//cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[SI46XX_MSG_GET_FREQ_LIST]);

		//Si46xxCfg.deviceStatus.STCINT = Si46xx_STCINT_COMPLETE; // für den ersten...
		Si46xxCfg.freqIndex = 2; // TODO Sollte dann auf Kanal 5C liegen
		cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[SI46XX_MSG_DAB_TUNE_FREQ]);

		cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[SI46XX_MSG_GET_DIGITAL_SERVICE_LIST]);
	}

	// TODO: INTB-Pin auswerten, später mit Interrupt... geht nicht!! Pulldown an INTB?
	//if(HAL_GPIO_ReadPin(Si46xx_INTB_GPIO_Port, Si46xx_INTB_Pin) == GPIO_PIN_SET)
	{
	//	Si46xxCfg.isrState = ISR_SET;
	}

	if(Si46xx_RemainingTimeLeft() == TIME_LEFT)
	{
		return;
	}

	switch(Si46xxCfg.state)
	{
		case Si46xx_STATE_IDLE:

			// hier dann mit ISRs arbeiten...

			// if flag & getSysState: befehl ausführen, größe der Rückgabewerte einsammeln, in wait-state gehen
			// ISR bringt ihn zurück in Antwort-Auswerten-State, aber dann ist auch eine Auswertefunktion nötig...
			// vielleicht einfach ein Struct, wo dann die Senden und Antworten-Funktion gepointert werden?
			// dann eine Aufrufe-Funktion für das jeweilige Feature Si46xx_GetSysState -> Si46xx_Send_GetSysState und Si46xx_Rcv_GetSysState
			// allgemeinen Reply-Pointer bauen? Also auf die Antwortbytes, dass er die Ablegt? Aber dann müsste er in der allgemeinen Auswertefkt ja wissen, was er damit soll...

			if(cb_pop_front(&Si46xxCfg.cb, &currentWorkingMsg) == CB_OK)
			{
				printf("\033[1;36mSi46xx: Befehl liegt auf dem Stack... (count: %d)\033[0m\r\n", Si46xxCfg.cb.count);

				if(currentWorkingMsg.sendFunc() == HAL_OK)
				{
					Si46xxCfg.state = Si46xx_STATE_BUSY;
					Si46xxCfg.isrState = ISR_UNSET;
				}
				else // otherwise put it back on stack
				{
					cb_push_back(&Si46xxCfg.cb, &currentWorkingMsg);
					Si46xx_SetWaitTime(100);
				}
			}

			break;

		case Si46xx_STATE_BOOTING:
			if(Si46xx_Boot_Tasks() == Si46xx_STATE_IDLE)
			{
				Si46xxCfg.state = Si46xx_STATE_IDLE;
			}
			break;

		case Si46xx_STATE_BUSY:
			if(Si46xxCfg.isrState == ISR_SET &&
			   (HAL_GPIO_ReadPin(Si46xx_INTB_GPIO_Port, Si46xx_INTB_Pin) == GPIO_PIN_RESET)
			/*|| Si46xx_RemainingTimeLeft() == TIME_OVER*/) // TODO: Time over geht noch nicht. auch nicht hier hin, oder?
			{
				Si46xxCfg.isrState = ISR_UNSET;

				switch(currentWorkingMsg.receiveFunc())
				{
					case Si46xx_OK:
						printf("\033[1;36mSi46xx: Receive function OK, back to idle...\033[0m\r\n");
						Si46xxCfg.state = Si46xx_STATE_IDLE;
						break;

					case Si46xx_BUSY: // Busy even if INTB pin set?
					case Si46xx_SPI_ERROR:
					case Si46xx_MESSAGE_ERROR:
						printf("\033[1;36mSi46xx: Error from receive function, msg back on stack\033[0m\r\n");
						cb_push_back(&Si46xxCfg.cb, &currentWorkingMsg); // otherwise put it back on stack
						Si46xx_SetWaitTime(100);
						break;

					case Si46xx_DEVICE_ERROR: // Serious error, reset device
						printf("\033[1;36mSi46xx: Device error in receive function, resetting Si46xx...\033[0m\r\n");
						Si46xx_Boot();
						break;
				}
			}

			// TODO: hier Timeoutfunktion
			break;
	}
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

	//Si46xxCfg.intstate = SI46XX_INTB_STATE;

	Si46xxCfg.state = Si46xx_STATE_IDLE;

	// Initialize ring buffer
	cb_init(&Si46xxCfg.cb, 10, sizeof(Si46xx_msg_dt));

	//Si46xxCfg.timeoutValue = SI46XX_DEFAULT_SPI_WAIT;

	Si46xxCfg.isrState = ISR_UNSET;

	 pinBuf = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin); // Temporär TODO

	return HAL_OK;
}

void progress_StatusBytes(Si46xx_Status_Values_dt * status, uint8_t * data)
{
	/* Progress into status variables */
	status->CTS      = (data[0] & 0x80)>>7;
	status->ERR_CMD  = (data[0] & 0x40)>>6;
	status->STCINT   = (data[0] & 0x01);
	status->PUP      = (data[3] & 0xC0)>>6;
	status->RFFE_ERR = (data[3] & 0x20)>>5;
	status->REPOFERR = (data[3] & 0x08)>>3;
	status->CMDOFERR = (data[3] & 0x04)>>2;
	status->ARBERR   = (data[3] & 0x02)>>1;
	status->ERRNR    = (data[3] & 0x01);
}

/* Functions called by state machine ---------------------------------------------------*/

HAL_StatusTypeDef Si46xx_Send_GetSysState(void)
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_OK;

	data[0] = SI46XX_GET_SYS_STATE;
	data[1] = 0x00;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);

	return state;
}


/* SPI routines ---------------------------------------------------------------- */

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
	HAL_StatusTypeDef state = HAL_ERROR;

	/* Stop here if SPI isn't ready */
	if(hspi->State != HAL_SPI_STATE_READY)
	{
		return HAL_BUSY; // TODO: erstmal Busy. Wenn ein Fehler ist, sollte der aber auch gecatcht werden...
	}

	if(len < 4)
	{
		len = 4;
	}

	uint8_t zeroes[1+len]; // {0, ??? TODO dynamisch mit memalloc oder sowas
	memset(zeroes, 0, 1+len);

	SI46XX_CS_ON();
	state = HAL_SPI_TransmitReceive(hspi, zeroes, data, len+1, 100); // TODO irgendwann als IT Version? TODO status ist kein si46xxx.... muss gewandelt!!
	SI46XX_CS_OFF();

	return state;
}

/*
 * Analyze last status message
 */
Si46xx_statusType Si46xx_SPIgetAnalyzeStatus(uint8_t * data, uint16_t len)
{
	Si46xx_Status_Values_dt * deviceStatus = &Si46xxCfg.deviceStatus;

	switch(Si46xx_SPIgetStatus(Si46xxCfg.hspi, data, len))
	{
		case HAL_OK:
			progress_StatusBytes(deviceStatus, &spiBuffer[1]);

			if // Reset device error
			(
				deviceStatus->ARBERR == Si46xx_ERR_ERROR || /* An arbiter overflow has occurred. The only way to recover is for the user to reset the chip. */
				deviceStatus->ERRNR  == Si46xx_ERR_ERROR    /* Fatal error has occurred. The only way to recover is for the user to reset the chip.*/
			)
			{
				printf("\032[1;36mSi46xx: IC Error\032[0m\r\n");
				return Si46xx_DEVICE_ERROR;
			}

			if // Resend last command error
			(
					deviceStatus->CMDOFERR == Si46xx_ERR_ERROR || /* The command interface has overflowed, and data has been lost */
					deviceStatus->REPOFERR == Si46xx_ERR_ERROR    /* The reply interface has underflowed, and bad data has been returned to the user */
			)
			{
				printf("\032[1;36mSi46xx_Boot: Message Error\032[0m\r\n");
				return Si46xx_MESSAGE_ERROR;
			}

			if(deviceStatus->CTS == Si46xx_CTS_READY) // Ready for next command
			{
				return Si46xx_OK;

				/* TODO :NOT_READY 	0x0 	Chip is not ready for the user to send a command or read a reply. If the user sends a firmware-interpreted command, the contents of the transaction will be ignored and the CTS bit will remain 0. If the user reads a reply, they will receive the status byte (CTS bit with value 0 followed by 7 other status bits). All following bytes will read as zero.
				 *  -> muss der dann nicht für die anderen auhc ne 0 rausgeben?
				 */
			}
			return Si46xx_BUSY;

		case HAL_BUSY:
			return Si46xx_BUSY;

		default:
			printf("\032[1;36mSi46xx: Si46xx_SPIgetAnalyzeStatus SPI Error\032[0m\r\n");
			return Si46xx_SPI_ERROR;
	}
}


/* Timeout and waiting routines --------------------------------- */
void Si46xx_SetWaitTime(uint32_t ms)
{
	Si46xxCfg.waitStamp = HAL_GetTick();
	Si46xxCfg.waitTime  = ms;
}

uint32_t Si46xx_CurrentWaitTime(void)
{
	return HAL_GetTick() - Si46xxCfg.waitStamp;
}

enum Si46xx_Wait_en Si46xx_RemainingTimeLeft(void)
{
	if(Si46xx_CurrentWaitTime() < Si46xxCfg.waitTime)
	{
		return TIME_LEFT;
	}
	else
	{
		return TIME_OVER;
	}
}
