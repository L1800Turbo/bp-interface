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

Si46xx_msg_dt currentWorkingMsg;

/* Private functions ------------------------------------------- */
extern void Si46xx_Boot(void);
Si46xx_statusType Si46xx_SPIgetAnalyzeStatus(uint8_t * data, uint16_t len); // todo müsste eigentlich in header

/* Helper functions -------------------------------------------- */
void set_Si46xx_ISR(void)
{
	Si46xxCfg.isrState = ISR_SET;
}




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

void Si46xx_Tasks(void)
{
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
				printf("Si46xx: Befehl liegt auf dem Stack... (count: %d)\n", Si46xxCfg.cb.count);

				switch(currentWorkingMsg.sendFunc())
				{
					case HAL_OK:
						Si46xxCfg.state = Si46xx_STATE_BUSY;
						Si46xxCfg.isrState = ISR_UNSET;
						break;

					case HAL_ERROR:
						printf("Si46xx: SPI-Error in Sending function!\n");
						break;

					case HAL_BUSY:
					case HAL_TIMEOUT:
						cb_push_back(&Si46xxCfg.cb, &currentWorkingMsg); // TODO: eine aktuell beschäftigte Funktion sollte am Anfang bleiben!
						Si46xx_SetWaitTime(100);

						break;
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

			/* If interrupt was set and is low now  TODO: low-INT einstellen? */
			if(Si46xxCfg.isrState == ISR_SET &&
			   (HAL_GPIO_ReadPin(Si46xx_INTB_GPIO_Port, Si46xx_INTB_Pin) == GPIO_PIN_RESET))
			{
				Si46xxCfg.isrState = ISR_UNSET;

				switch(currentWorkingMsg.receiveFunc())
				{
					case Si46xx_OK:
						printf("Si46xx: Receive function OK, back to idle...\n");
						Si46xxCfg.state = Si46xx_STATE_IDLE;
						break;

					case Si46xx_BUSY: // Busy even if INTB pin set?
					case Si46xx_SPI_ERROR:
					case Si46xx_MESSAGE_ERROR:
						printf("Si46xx: Error from receive function, msg back on stack\n");
						cb_push_back(&Si46xxCfg.cb, &currentWorkingMsg); // otherwise put it back on stack
						Si46xx_SetWaitTime(100);
						break;

					case Si46xx_DEVICE_ERROR: // Serious error, reset device
						printf("Si46xx: Device error in receive function, resetting Si46xx...\n");
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

	// Configure default sources for firmware/patch
	Si46xx_Boot_SetSources(FW_SRC_UC, FW_SRC_UC);

	// Init first channel TODO später vom EEPROM?
	Si46xxCfg.freqIndex = 0;

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

HAL_StatusTypeDef Si46xx_Send_GetSysState(void) // TODO weg
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_OK;

	data[0] = SI46XX_GET_SYS_STATE;
	data[1] = 0x00;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);

	return state;
}




/*
 * Analyze last status message
 */
Si46xx_statusType Si46xx_SPIgetAnalyzeStatus(uint8_t * data, uint16_t len)
{
	Si46xx_statusType status = Si46xx_BUSY;
	Si46xx_Status_Values_dt * deviceStatus = &Si46xxCfg.deviceStatus;

	//printf("sst_%d\n",  Si46xxCfg.deviceStatus);

	switch(Si46xx_SPIgetStatus(Si46xxCfg.hspi, data, len))
	{
		case HAL_OK:
			progress_StatusBytes(deviceStatus, &data[1]); // TODO Passt das noch?
			//progress_StatusBytes(deviceStatus, &spiBuffer[1]); ortiginal...

			if // Reset device error
			(
				deviceStatus->ARBERR == Si46xx_ERR_ERROR || /* An arbiter overflow has occurred. The only way to recover is for the user to reset the chip. */
				deviceStatus->ERRNR  == Si46xx_ERR_ERROR    /* Fatal error has occurred. The only way to recover is for the user to reset the chip.*/
			)
			{
				printf("Si46xx: IC Error\n");
				status = Si46xx_DEVICE_ERROR;
			}

			if // Resend last command error
			(
					deviceStatus->CMDOFERR == Si46xx_ERR_ERROR || /* The command interface has overflowed, and data has been lost */
					deviceStatus->REPOFERR == Si46xx_ERR_ERROR    /* The reply interface has underflowed, and bad data has been returned to the user */
			)
			{
				printf("Si46xx_Boot: Message Error\n");
				status = Si46xx_MESSAGE_ERROR;
			}

			if(deviceStatus->CTS == Si46xx_CTS_READY) // Ready for next command
			{
				// TODO: noch besser einbauen:
				if(deviceStatus->ERR_CMD == Si46xx_ERR_ERROR)
				{
					printf("Si46xx_Boot: Command error, Bad command, see reply byte 4 for details\n");
				}
				status = Si46xx_OK;

				/* TODO :NOT_READY 	0x0 	Chip is not ready for the user to send a command or read a reply. If the user sends a firmware-interpreted command, the contents of the transaction will be ignored and the CTS bit will remain 0. If the user reads a reply, they will receive the status byte (CTS bit with value 0 followed by 7 other status bits). All following bytes will read as zero.
				 *  -> muss der dann nicht für die anderen auhc ne 0 rausgeben?
				 */
			}
			break;

		case HAL_BUSY:
			status = Si46xx_BUSY;
			break;

		default:
			printf("\032[1;36mSi46xx: Si46xx_SPIgetAnalyzeStatus SPI Error\032[0m\r\n");
			status = Si46xx_SPI_ERROR;
			break;
	}

	CDC_Transmit_FS((uint8_t *) "sst ", 4);
	CDC_Transmit_FS((uint8_t*) &Si46xxCfg.deviceStatus, sizeof(Si46xx_Status_Values_dt));
	//xCDC_Transmit_FS((uint8_t*) &Si46xxCfg.image, sizeof(enum Si46xx_Image));
	CDC_Transmit_FS((uint8_t*) "\n", 1);

	if(Si46xxCfg.deviceStatus.STCINT == Si46xx_STCINT_COMPLETE)
	{
		printf("sCurFreq_%d\n", sizeof(DAB_frequency_dt));
		CDC_Transmit_FS(&DAB_frequency_list[Si46xxCfg.freqIndex], sizeof(DAB_frequency_dt));
	}

	return status;
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
