/*
 * Si46xx_boot.c
 *
 *  Created on: 12.12.2021
 *      Author: kai
 */

#include "Si46xx_boot.h"

/* Private variables ---------------------------------------------------------*/
Si46xx_BootStates_en bootState; // State machine controlling the boot process
Si46xx_firmware_dt firmware;

/* Private define ------------------------------------------------------------*/
#define SI46XX_PWR_ON_TIME   4 // 3.2ms to wait after RST

extern uint8_t spiBuffer[4096];
extern struct Si46xx_Config Si46xxCfg;

/* Private functions --------------------------------------------------------*/
HAL_StatusTypeDef Si46xx_Send_PowerUp(void);
HAL_StatusTypeDef Si46xx_Send_LoadInit(void);
HAL_StatusTypeDef Si46xx_HostLoad(uint8_t * bufferPtr, uint32_t size);
HAL_StatusTypeDef Si46xx_Send_Boot(void);

// TODO: mehr eine temporäre Geschichte für USB-Befehl ohne Prüfung und so..
void Si46xx_Boot(void)
{
	HAL_GPIO_TogglePin(LED_GREEN_Port, LED_GREEN_Pin);
	//printf("Setze State ohne Frage auf BOOTING...\r\n"); -> in ISR geht das nicht
	bootState = Si46xx_INIT_STATE_OFF;
	Si46xxCfg.state = Si46xx_STATE_BOOTING;
}

/* Switch between the right boot state depending on the SPI status */
Si46xx_BootStates_en getNextBootState(uint8_t * statusData, Si46xx_BootStates_en nextState_Error, Si46xx_BootStates_en nextState_MsgError, Si46xx_BootStates_en nextState_okay)
{
	Si46xx_Status_Values_dt * deviceStatus = &Si46xxCfg.deviceStatus;

	progress_StatusBytes(deviceStatus, statusData); // TODO: Wenn alles FF ist ,auch noch warten?

	if // Reset device error
	(
		deviceStatus->ARBERR == Si46xx_ERR_ERROR || /* An arbiter overflow has occurred. The only way to recover is for the user to reset the chip. */
		deviceStatus->ERRNR  == Si46xx_ERR_ERROR    /* Fatal error has occurred. The only way to recover is for the user to reset the chip.*/
	)
	{
		printf("\032[1;36mSi46xx_Boot: IC Error\032[0m\r\n");
		return nextState_Error;
	}

	if // Resend last command error
	(
			deviceStatus->CMDOFERR == Si46xx_ERR_ERROR || /* The command interface has overflowed, and data has been lost */
			deviceStatus->REPOFERR == Si46xx_ERR_ERROR    /* The reply interface has underflowed, and bad data has been returned to the user */
	)
	{
		printf("\032[1;36mSi46xx_Boot: Message Error\032[0m\r\n");
		return nextState_MsgError;
	}

	if(deviceStatus->CTS == Si46xx_CTS_READY) // Ready for next command
	{
		return nextState_okay;
	}

	printf("\032[1;36mSi46xx_Boot: Kein sinnvoller nextState, statusData RAW: 0x%X\032[0m\r\n", *statusData);
	Si46xx_SetWaitTime(100);

	return bootState; // Aktuellen state behalten, loopen
}

/**
 * Si46xx_Boot():
 * Non-blocking function to loop through initialization process
 */
Si46xx_state_en Si46xx_Boot_Tasks(void)
{
	Si46xx_state_en retState = Si46xx_STATE_BUSY;
	Si46xx_BootStates_en tmpState; // Used in Si46xx_INIT_STATE_HOST_LOAD_WAIT

	/* If there is waiting time or SPI isn't ready, don't go into state machine */
	if(Si46xx_RemainingTimeLeft() == TIME_LEFT || Si46xxCfg.hspi->State != HAL_SPI_STATE_READY)
	{
		// TODO: HAL-State auswerten hier und dann unten vereinfachen?
		return retState;
	}

	switch(bootState)
	{
		case Si46xx_INIT_STATE_OFF:
			printf("\033[1;36mSi46xx_Boot: Si46xx_INIT_STATE_OFF\033[0m\r\n");
			SI46XX_RST_ON();

			Si46xx_SetWaitTime(10); // Wait 10ms in Reset

			bootState = Si46xx_INIT_STATE_RELEASE_RESET; // next state: Power up the device
			break;

		/* Release reset after a mininmum time */
		case Si46xx_INIT_STATE_RELEASE_RESET:
			printf("\033[1;36mSi46xx_Boot: Si46xx_INIT_STATE_RELEASE_RESET\033[0m\r\n");

			SI46XX_RST_OFF();

			Si46xx_SetWaitTime(4); // 3.2ms to wait after RST

			bootState = Si46xx_INIT_STATE_POWER_UP_SEND; // next state:
			break;

		/* Send initialization after reset wait time */
		case Si46xx_INIT_STATE_POWER_UP_SEND:
			printf("\033[1;36mSi46xx_Boot: Si46xx_INIT_STATE_POWER_UP_SEND\033[0m\r\n");

			if(Si46xx_Send_PowerUp() == HAL_OK)
			{
				Si46xx_SetWaitTime(100); // ms TODO erstmal testweise, der rebootet iwie 3x
				bootState = Si46xx_INIT_STATE_POWER_UP_WAIT;
			}
			else
			{
				Si46xxCfg.state = Si46xx_INIT_STATE_OFF;
			}
			break;

		case Si46xx_INIT_STATE_POWER_UP_WAIT:
			printf("\033[1;36mSi46xx_Boot: Si46xx_INIT_STATE_POWER_UP_WAIT\033[0m\r\n");

			switch(Si46xx_SPIgetStatus(Si46xxCfg.hspi, spiBuffer, 4))
			{
				case HAL_OK:
					// Get next state depending von Si46xx state
					bootState = getNextBootState(&spiBuffer[1],
								Si46xx_INIT_STATE_OFF, Si46xx_INIT_STATE_POWER_UP_SEND, Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND);

					firmware.step = FW_BOOTLOADER_PATCH;
					break;

				case HAL_BUSY:
					// Loop for next round
					break;

				default: // Errors during SPI transmission, resend for now, TODO: am ersten SPI-Befehl besser SOFF?
					Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms

					//bootState = Si46xx_INIT_STATE_POWER_UP_SEND;
					break;
			}
			break;

		/* Prepare to load firmware, choose the right buffer */
		case Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND:
			printf("\033[1;36mSi46xx_Boot: Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND\033[0m\r\n");

			switch(Si46xx_Send_LoadInit())
			{
				case HAL_OK:
					Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms

					if(firmware.step == FW_BOOTLOADER_PATCH)
					{
						firmware.fwBufPtr  = (uint8_t *) &Si46xx_Rom00Patch016;
						firmware.fwBufSize = sizeof(Si46xx_Rom00Patch016);

						printf("\033[1;36mSi46xx_Boot: Load Patch\033[0m\r\n");
					}
					else if(firmware.step == FW_FIRMWARE)
					{
						firmware.fwBufPtr  = (uint8_t *) &Si46xx_Firmware;
						firmware.fwBufSize = sizeof(Si46xx_Firmware);

						printf("\033[1;36mSi46xx_Boot: Load Firmware\033[0m\r\n");
					}

					bootState = Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_WAIT;
					break;

				case HAL_BUSY:
					// Loop for next round
					break;

				default: // Errors during SPI transmission, back to previous state
					Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms
					bootState = Si46xx_INIT_STATE_POWER_UP_SEND;
					break;
			}
			break;

		case Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_WAIT:
			printf("\033[1;36mSi46xx_Boot: Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_WAIT\033[0m\r\n");

			switch(Si46xx_SPIgetStatus(Si46xxCfg.hspi, spiBuffer, 4))
			{
				case HAL_OK:
					// Get next state depending von Si46xx state
					bootState = getNextBootState(&spiBuffer[1],
								Si46xx_INIT_STATE_OFF, Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND, Si46xx_INIT_STATE_HOST_LOAD_SEND);
					// als nächstes die Firmware, bzw. den Patch laden...
					break;

				case HAL_BUSY:
					// Loop for next round
					break;

				default: // Errors during SPI transmission, resend for now
					Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms

					//bootState = Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND;
					break;
			}
			break;

		/* Program patch / firmware */
		case Si46xx_INIT_STATE_HOST_LOAD_SEND:

			switch(Si46xx_HostLoad(firmware.fwBufPtr, (firmware.fwBufSize > 4092 ? 4092 : firmware.fwBufSize)))
			{
				case HAL_OK:
					Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms

					bootState = Si46xx_INIT_STATE_HOST_LOAD_WAIT;
					break;

				case HAL_BUSY:
					break;
				default: // Errors during SPI transmission, back to previous state
					Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms
					//firmware.step = FW_BOOTLOADER_PATCH;
					//bootState = Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND;
					break;
			}
			break;

		case Si46xx_INIT_STATE_HOST_LOAD_WAIT:
			switch(Si46xx_SPIgetStatus(Si46xxCfg.hspi, spiBuffer, 4))
			{
				case HAL_OK:

					/* Evaluate next state */
					if(firmware.fwBufSize > 4092) // Still packages to send
					{
						firmware.fwBufPtr = firmware.fwBufPtr + 4092;
						firmware.fwBufSize -= 4092;


						tmpState = Si46xx_INIT_STATE_HOST_LOAD_SEND;
					}
					else // Nothing more to send, go back to load with next FW package or finish
					{
						//printf("Firmware from flash written: %s\r\n", firmwareStepTexts[Si46xxCfg.firmwareBuf->fwStep]);

						firmware.step++;

						if(firmware.step > FW_FIRMWARE) // If there are no more FW packages, boot the device
						{
							Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms
							tmpState = Si46xx_INIT_STATE_BOOT_SEND;
						}
						else // if there are more packages, back to prepare to load the firmware
						{
							Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms
							tmpState = Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND; // Back to prepare command
						}
					}

					// Get next state depending von Si46xx state, go to previously assigned state if everything worked out
					bootState = getNextBootState(&spiBuffer[1],
								Si46xx_INIT_STATE_OFF, Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND, tmpState);

					break;

				case HAL_BUSY:
					// Loop for next round
					break;

				default: // Errors during SPI transmission
					Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms
					//firmware.step = FW_BOOTLOADER_PATCH;
					//bootState = Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND;
					break;
			}

			break;

		/* initialize boot progress */
		case Si46xx_INIT_STATE_BOOT_SEND:
			printf("\033[1;36mSi46xx_Boot: Si46xx_INIT_STATE_BOOT_SEND\033[0m\r\n");

			switch(Si46xx_Send_Boot())
			{
				case HAL_OK:
					Si46xx_SetWaitTime(500); // ms TODO

					bootState = Si46xx_INIT_STATE_BOOT_WAIT;
					break;

				case HAL_BUSY:
					// Loop for next round
					break;

				default: // Problems...
					Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms
					break;
			}
			break;

		case Si46xx_INIT_STATE_BOOT_WAIT:
			printf("\033[1;36mSi46xx_Boot: Si46xx_INIT_STATE_BOOT_WAIT\033[0m\r\n");

			switch(Si46xx_SPIgetStatus(Si46xxCfg.hspi, spiBuffer, 4))
			{
				case HAL_OK:
					// Get next state depending von Si46xx state
					tmpState = getNextBootState(&spiBuffer[1],
								Si46xx_INIT_STATE_OFF, Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND, Si46xx_INIT_STATE_IDLE);

					// 2nd check: is the application loaded? getNextBootState also updates the status values
					if(tmpState == Si46xx_INIT_STATE_IDLE)
					{
						if(Si46xxCfg.deviceStatus.PUP == Si46xx_PUP_APP)
						{
							printf("\033[1;36mSi46xx_Boot: Done booting...\033[0m\r\n");
							//bootState = tmpState;
							retState = Si46xx_STATE_IDLE; // Idle signalisieren
						}
						else
						{
							// loop... TODO: was tun?
						}
					}
					else // in case of errors...
					{
						bootState = tmpState;
					}
					break;

				case HAL_BUSY:
					// Loop for next round
					break;

				default: // Errors during SPI transmission, resend for now
					Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms

					//bootState = Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND;
					break;
			}
			break;

		/* Idle: Boot finished.. TODO: das hier weg und zurück in die Main? */
		case Si46xx_INIT_STATE_IDLE:


			switch(Si46xx_SPIgetStatus(Si46xxCfg.hspi, spiBuffer, 4))
			{
				case HAL_OK:

					printf("Aktueller State: \r\n");
					printf("CTS: %d, PUP: %d, ERR_CMD: %d\r\n", Si46xxCfg.deviceStatus.CTS, Si46xxCfg.deviceStatus.PUP, Si46xxCfg.deviceStatus.ERR_CMD);
					// Get next state depending von Si46xx state
					//bootState = getNextBootState(&spiBuffer[1],
					//			Si46xx_INIT_STATE_OFF, Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND, Si46xx_INIT_STATE_IDLE);

					break;

				case HAL_BUSY:
					// Loop for next round
					break;

				default: // Errors during SPI transmission, resend for now
					Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms

					//bootState = Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND;
					break;
			}
			/*
			switch(Si46xx_getStatus(5, state)) // TODO 5 geht gar nicht, wenn der zum ersten Mal hier aufgerufen wird!!
			{
				case Si46xx_READY:
					printf("Aktueller State: \r\n");
					printf("CTS: %d, PUP: %d, ERR_CMD: %d\r\n", Si46xxCfg.deviceStatus.CTS, Si46xxCfg.deviceStatus.PUP, Si46xxCfg.deviceStatus.ERR_CMD);
					printf("Image: %d\r\n", *state);

					switch(Si46xx_Send_GetSysState())
					{
						case HAL_OK:
							// IDLE State..Si46xx_Set_AnswerState(Si46xxCfg.state, Si46xx_STATE_IDLE, 4);
							break;

						case HAL_BUSY:
							Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms
							break;

						default: // Problems...
							Si46xxCfg.state = Si46xx_INIT_STATE_OFF;
							break;
					}
					break;

				case Si46xx_ERROR:
					Si46xxCfg.state = Si46xx_INIT_STATE_OFF;
					break;

				case Si46xx_MSG_ERROR:
					Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms
					Si46xxCfg.state = Si46xx_INIT_STATE_POWER_UP;
					break;

				default:
					break;

			}*/

			Si46xx_SetWaitTime(1500); // ms
			break;

	}

	return retState;
}


/* Functions called by state machine ---------------------------------------------------*/

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

HAL_StatusTypeDef Si46xx_HostLoad(uint8_t * bufferPtr, uint32_t size)
{
	HAL_StatusTypeDef state = HAL_OK;
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

HAL_StatusTypeDef Si46xx_Send_Boot(void)
{
	HAL_StatusTypeDef state = HAL_OK;

	//Si46xxCfg.timeoutValue = 300; TODO

	spiBuffer[0] = SI46XX_BOOT;
	spiBuffer[1] = 0x00;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, spiBuffer, 2);

	return state;
}

