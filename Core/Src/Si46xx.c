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
HAL_StatusTypeDef Si46xx_SPIgetStatus(SPI_HandleTypeDef * hspi, uint8_t * data, uint16_t len);

HAL_StatusTypeDef Si46xx_Msg_RefreshSysState_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		data[0] = SI46XX_GET_SYS_STATE;
		data[1] = 0x00;

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);
	}

	return state;
}

HAL_StatusTypeDef Si46xx_Msg_RefreshSysState_receiveFunc()
{
	HAL_StatusTypeDef state = HAL_OK;

	state = Si46xx_SPIgetStatus(Si46xxCfg.hspi, spiBuffer, 5);

	if(state != HAL_OK)
	{
		return state;
	}

	progress_StatusBytes(&Si46xxCfg.deviceStatus, &spiBuffer[1]);

	// TODO: Fehlerbehandlung für IC-Errors

	Si46xxCfg.image = spiBuffer[5];

	printf("\033[1;36mSi46xx: Image %d\033[0m\r\n", Si46xxCfg.image);

	return state;
}

const Si46xx_msg_dt Si46xx_messages[SI46XX_MSG_SIZE] = {
		{0}, /* SI46XX_MSG_NONE */
		{.sendFunc = Si46xx_Msg_RefreshSysState_sendFunc, .receiveFunc = Si46xx_Msg_RefreshSysState_receiveFunc }	/* SI46XX_MSG_REFRESH_SYS_STATE */
};

extern Si46xx_state_en Si46xx_Boot_Tasks(void); // TODO temporär


void Si46xx_Tasks(void)
{

	if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1)
	{
		//Si46xx_msg_dt msgTest = {.cmd = SI46XX_GET_SYS_STATE };
		cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[SI46XX_MSG_REFRESH_SYS_STATE]);
	}

	GPIO_PinState pinDings = HAL_GPIO_ReadPin(Si46xx_INTB_GPIO_Port, Si46xx_INTB_Pin);

	// TODO: INTB-Pin auswerten, später mit Interrupt... geht nicht!! Pulldown an INTB?
	if(HAL_GPIO_ReadPin(Si46xx_INTB_GPIO_Port, Si46xx_INTB_Pin) == GPIO_PIN_RESET)
	{
		Si46xxCfg.isrState = ISR_SET;
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
				printf("\032[1;36mXXX: liegt auf dem Buffer, länge: %d\032[0m\r\n", Si46xxCfg.cb.count);

				if(currentWorkingMsg.sendFunc() == HAL_OK)
				{
					Si46xxCfg.state = Si46xx_STATE_BUSY;
					Si46xxCfg.isrState = ISR_UNSET;
				}
				else // otherwise put it back on stack
				{
					cb_push_back(&Si46xxCfg.cb, &currentWorkingMsg);
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
			if(Si46xxCfg.isrState == ISR_SET)
			{
				Si46xxCfg.isrState = ISR_UNSET;

				if(currentWorkingMsg.receiveFunc() == HAL_OK)
				{
					Si46xxCfg.state = Si46xx_STATE_IDLE;
				}
				else // otherwise put it back on stack
				{
					cb_push_back(&Si46xxCfg.cb, &currentWorkingMsg);
				}
			}
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

	return HAL_OK;
}

void progress_StatusBytes(Si46xx_Status_Values_dt * status, uint8_t * data)
{
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
