/*
 * Si46xx_boot.c
 *
 *  Created on: 12.12.2021
 *      Author: kai
 */

#include "Si46xx_boot.h"

#include "usbd_cdc_if.h" // Functions to get data from USB CDC buffer

/* Private variables ---------------------------------------------------------*/
Si46xx_BootStates_en bootState; // State machine controlling the boot process
Si46xx_firmware_dt firmware;

// For debugging by USB
const char fw_stateStr[FW_size][20] =
{
		"FW_NONE",
		"FW_BOOTLOADER_PATCH",
		"FW_FIRMWARE"
};

/* Private define ------------------------------------------------------------*/
#define SI46XX_PWR_ON_TIME   4 // 3.2ms to wait after RST

extern uint8_t spiBuffer[4096];
extern struct Si46xx_Config Si46xxCfg;

/* Private functions --------------------------------------------------------*/
HAL_StatusTypeDef Si46xx_Send_PowerUp(void);
HAL_StatusTypeDef Si46xx_Send_LoadInit(void);
//HAL_StatusTypeDef Si46xx_HostLoad(fw_source_dt fwSource, uint8_t * bufferPtr, uint32_t size);
HAL_StatusTypeDef Si46xx_Send_Boot(void);

/* Configure sources for firmware */
// TODO anders oder weg
/*uint8_t Si46xx_Boot_SetSources(fw_source_dt srcBootloader_patch, fw_source_dt srcFirmware)
{
	uint8_t retVal = 0;

	if(srcBootloader_patch < FW_SRC_size)
	{
		//firmware.fw_source[FW_BOOTLOADER_PATCH] = srcBootloader_patch;
	}
	else
	{
		retVal = 1;
	}

	if(srcFirmware < FW_SRC_size)
	{
		//firmware.fw_source[FW_FIRMWARE] = srcFirmware;
	}
	else
	{
		retVal = 1;
	}

	return retVal;
}*/

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

	printf("Si46xx_Boot: Kein sinnvoller nextState, statusData RAW: 0x%X \n", *statusData);
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
			printf("Si46xx_Boot: Si46xx_INIT_STATE_OFF\n");
			SI46XX_RST_ON();

			Si46xx_SetWaitTime(10); // Wait 10ms in Reset

			bootState = Si46xx_INIT_STATE_RELEASE_RESET; // next state: Power up the device
			break;

		/* Release reset after a mininmum time */
		case Si46xx_INIT_STATE_RELEASE_RESET:
			printf("Si46xx_Boot: Si46xx_INIT_STATE_RELEASE_RESET\n");

			SI46XX_RST_OFF();

			Si46xx_SetWaitTime(4); // 3.2ms to wait after RST

			bootState = Si46xx_INIT_STATE_POWER_UP_SEND; // next state:
			break;

		/* Send initialization after reset wait time */
		case Si46xx_INIT_STATE_POWER_UP_SEND:
			printf("Si46xx_Boot: Si46xx_INIT_STATE_POWER_UP_SEND\n");

			if(Si46xx_Send_PowerUp() == HAL_OK)
			{
				Si46xx_SetWaitTime(200); // ms TODO erstmal testweise, der rebootet iwie 3x
				bootState = Si46xx_INIT_STATE_POWER_UP_WAIT;
			}
			else
			{
				Si46xxCfg.state = Si46xx_INIT_STATE_OFF;
			}
			break;

		case Si46xx_INIT_STATE_POWER_UP_WAIT:
			printf("Si46xx_Boot: Si46xx_INIT_STATE_POWER_UP_WAIT\n");

			switch(Si46xx_SPIgetStatus(Si46xxCfg.hspi, spiBuffer, 4))
			{
				case HAL_OK:
					// Get next state depending von Si46xx state
					bootState = getNextBootState(&spiBuffer[1],
								Si46xx_INIT_STATE_OFF, Si46xx_INIT_STATE_POWER_UP_SEND, Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND);

					Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms
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
			printf("Si46xx_Boot: Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND\n");

			switch(Si46xx_Send_LoadInit())
			{
				case HAL_OK:
					Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms

					if(firmware.current_fw_source == FW_SRC_USB) // If the current module should be transferred by USB
					{
						firmware.fwBufPtr = 0;
						firmware.fwBufSize = 0; // To be set by USB CDC function

						/* wait for USB firmware */
						firmware.usbFw_wanted = USB_FW_WAITING;

						/* initiate process to PC */
						// TODO: Fehlerbehandlung, wenn erkannt wird, dass gar kein PC angeschlossen ist
						printf("sfile_%s\n", fw_stateStr[firmware.step]);
					}
					else // If the transfer should happen from µC ROM
					{
						if(firmware.step == FW_BOOTLOADER_PATCH)
						{
							firmware.fwBufPtr  = (uint8_t *) &Si46xx_Rom00Patch016;
							firmware.fwBufSize = sizeof(Si46xx_Rom00Patch016);

							printf("Si46xx_Boot: Load Patch from uC\n");
						}
						else if(firmware.step == FW_FIRMWARE)
						{
							firmware.fwBufPtr  = (uint8_t *) &Si46xx_Firmware;
							firmware.fwBufSize = sizeof(Si46xx_Firmware);

							printf("Si46xx_Boot: Load Firmware from uC\n");
						}
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
			printf("Si46xx_Boot: Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_WAIT\n");

			switch(Si46xx_SPIgetStatus(Si46xxCfg.hspi, spiBuffer, 4))
			{
				case HAL_OK:
					Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms
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

			// If USB_FW is initiated and we ware waiting for the progress to be initiated and the block to be transferred
			if(firmware.usbFw_wanted == USB_FW_WAITING)
			{
				break;
			}
			else if(firmware.usbFw_wanted == USB_FW_TRANSFERRED)
			{
				// Set state for CDC module to let it wait, we transfer this block now
				firmware.usbFw_wanted = USB_FW_BUSY;
			}

			// HostLoad copies the data from uC flash or USB buffer onto SPI buffer and starts SPI transfer
			/*switch(Si46xx_HostLoad(firmware.fw_source[firmware.step], firmware.fwBufPtr, (firmware.fwBufSize > 4092 ? 4092 : firmware.fwBufSize)))
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
			}*/
			break;

		case Si46xx_INIT_STATE_HOST_LOAD_WAIT:
			switch(Si46xx_SPIgetStatus(Si46xxCfg.hspi, spiBuffer, 4))
			{
				case HAL_OK:
					Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms

					/* Evaluate next state */
					if(firmware.fwBufSize > SI46XX_BOOT_MAX_BUF_SIZE) // Still packages to send
					{
						firmware.fwBufSize -= SI46XX_BOOT_MAX_BUF_SIZE;
						printf("Si46xx_boot: remaining fwBufSize %ld \n", firmware.fwBufSize);

						HAL_GPIO_TogglePin(LED_GREEN_Port, LED_GREEN_Pin);

						// Tasks to do on USB transfer
						if(firmware.current_fw_source == FW_SRC_USB)
						{
							// Show CDC state machine that we're waiting for the next block
							firmware.usbFw_wanted = USB_FW_WAITING;
						}
						// Tasks to do while transferring from uC flash
						else if(firmware.current_fw_source == FW_SRC_UC)
						{
							firmware.fwBufPtr = firmware.fwBufPtr + SI46XX_BOOT_MAX_BUF_SIZE;
						}

						tmpState = Si46xx_INIT_STATE_HOST_LOAD_SEND;
					}
					else // Nothing more to send, go back to load with next FW package or finish
					{
						// Tasks to do on USB transfer mode when transfer of file is finished
						if(firmware.current_fw_source == FW_SRC_USB)
						{
							// Reset USB firmware transfer, if it was activated
							firmware.usbFw_wanted = USB_FW_NONE;
						}


						//printf("Firmware from flash written: %s\r\n", firmwareStepTexts[Si46xxCfg.firmwareBuf->fwStep]);

						firmware.step++;

						if(firmware.step > FW_FIRMWARE) // If there are no more FW packages, boot the device
						{
							Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms
							tmpState = Si46xx_INIT_STATE_BOOT_SEND;
						}
						else // if there are more packages, back to prepare to load the firmware
						{
							printf("Si46xx_Boot: More firmware to send\n");
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
					break;
			}

			break;

		/* initialize boot progress */
		case Si46xx_INIT_STATE_BOOT_SEND:
			printf("Si46xx_Boot: Si46xx_INIT_STATE_BOOT_SEND\n");

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
			printf("Si46xx_Boot: Si46xx_INIT_STATE_BOOT_WAIT\n");

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
							printf("Si46xx_Boot: Done booting...\n");
							//bootState = tmpState;
							retState = Si46xx_STATE_IDLE; // Idle signalisieren

							cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[SI46XX_MSG_REFRESH_SYS_STATE]);
						}
						else
						{
							Si46xx_SetWaitTime(100); // ms

							// loop... TODO: Timeout definieren, ab dem man auf Reset zurück geht, weil die Nummer wohl nicht funktioniert hat...
							// Wenn die FW kaputt kopiert wurde, landet er vermutlich auch hier

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

		/* Idle: Boot finished.. TODO: das hier weg und zurück in die Main? aktuell landet er nie hier... */
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
			Si46xx_SetWaitTime(1500); // ms
			break;

	}

	return retState;
}


/* Functions called by state machine ---------------------------------------------------*/

HAL_StatusTypeDef Si46xx_Send_PowerUp(void) // TODO: ist in Functions kopiert
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

HAL_StatusTypeDef Si46xx_Send_LoadInit(void) // TODO ist in functions...
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
	HAL_StatusTypeDef state = HAL_OK;

	//Si46xxCfg.timeoutValue = 300; TODO

	spiBuffer[0] = SI46XX_BOOT;
	spiBuffer[1] = 0x00;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, spiBuffer, 2);

	return state;
}

