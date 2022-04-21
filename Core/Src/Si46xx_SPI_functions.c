/*
 * Si46xx_SPI_functions.c
 *
 * Functions to be called by state machine
 *
 *  Created on: Dec 31, 2021
 *      Author: kai
 */
#include "Si46xx_SPI_functions.h"
#include <stdio.h>
#include <string.h>

#include "usbd_cdc_if.h" // To get USB data from the ring buffer

/* Private functions ------------------------------------------- */
Si46xx_statusType Si46xx_Msg_ReadReply_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_PowerUp_sendFunc();
HAL_StatusTypeDef Si46xx_Msg_HostLoad_sendFunc();
HAL_StatusTypeDef Si46xx_Msg_FlashLoad_LoadImage_sendFunc();
HAL_StatusTypeDef Si46xx_Msg_FlashLoad_WriteBlock_sendFunc();
HAL_StatusTypeDef Si46xx_Msg_FlashLoad_EraseSector_sendFunc();
HAL_StatusTypeDef Si46xx_Msg_FlashLoad_EraseChip_sendFunc();
HAL_StatusTypeDef Si46xx_Msg_LoadInit_sendFunc();
HAL_StatusTypeDef Si46xx_Msg_Boot_sendFunc();

HAL_StatusTypeDef Si46xx_Msg_GetSysState_sendFunc();
Si46xx_statusType Si46xx_Msg_GetSysState_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_GetDigitalServiceList_sendFunc();
Si46xx_statusType Si46xx_Msg_GetDigitalServiceList_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_DABtuneFreq_sendFunc();
Si46xx_statusType Si46xx_Msg_DABtuneFreq_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_StartDigitalService_sendFunc();
Si46xx_statusType Si46xx_Msg_StartDigitalService_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_StopDigitalService_sendFunc();
Si46xx_statusType Si46xx_Msg_StopDigitalService_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_GetDigitalServiceData_sendFunc();
Si46xx_statusType Si46xx_Msg_GetDigitalServiceData_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_GetEventStatus_sendFunc();
Si46xx_statusType Si46xx_Msg_GetEventStatus_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_GetEnsembleInfo_sendFunc();
Si46xx_statusType Si46xx_Msg_GetEnsembleInfo_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_SetFreqList_sendFunc();
Si46xx_statusType Si46xx_Msg_SetFreqList_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_GetFreqList_sendFunc();
Si46xx_statusType Si46xx_Msg_GetFreqList_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_GetServiceInfo_sendFunc();
Si46xx_statusType Si46xx_Msg_GetServiceInfo_receiveFunc();

void progress_StatusBytes(Si46xx_Status_Values_dt * status, uint8_t * data);
Si46xx_statusType Si46xx_SPIgetAnalyzeStatus(uint8_t * data, uint16_t len);

extern struct Si46xx_Config Si46xxCfg;
uint8_t spiBuffer[4096];

extern Si46xx_statusType Si46xx_SPIgetAnalyzeStatus(uint8_t * data, uint16_t len);

const Si46xx_msg_dt Si46xx_messages[SI46XX_MSG_SIZE] = {
		{0}, /* SI46XX_MSG_NONE */
		{
				.msgIndex    = SI46XX_MSG_POWER_UP,
				.msgName	 = "SI46XX_MSG_POWER_UP",
				.sendFunc    = Si46xx_Msg_PowerUp_sendFunc,
				.receiveFunc = Si46xx_Msg_ReadReply_receiveFunc
		},	/* SI46XX_MSG_POWER_UP */
		{
				.msgIndex    = SI46XX_MSG_HOST_LOAD,
				.msgName	 = "SI46XX_MSG_HOST_LOAD",
				.sendFunc    = Si46xx_Msg_HostLoad_sendFunc,
				.receiveFunc = Si46xx_Msg_ReadReply_receiveFunc
		},	/* SI46XX_MSG_HOST_LOAD */
		{
				.msgIndex    = SI46XX_MSG_FLASH_LOAD_IMG,
				.msgName	 = "SI46XX_MSG_FLASH_LOAD_IMG",
				.sendFunc    = Si46xx_Msg_FlashLoad_LoadImage_sendFunc,
				.receiveFunc = Si46xx_Msg_ReadReply_receiveFunc
		},	/* SI46XX_MSG_FLASH_LOAD_IMG */
		{
				.msgIndex    = SI46XX_MSG_FLASH_WRITE_BLOCK,
				.msgName	 = "SI46XX_MSG_FLASH_WRITE_BLOCK",
				.sendFunc    = Si46xx_Msg_FlashLoad_WriteBlock_sendFunc,
				.receiveFunc = Si46xx_Msg_ReadReply_receiveFunc
		},	/* SI46XX_MSG_FLASH_WRITE_BLOCK */
		{
				.msgIndex    = SI46XX_MSG_FLASH_ERASE_SECTOR,
				.msgName	 = "SI46XX_MSG_FLASH_ERASE_SECTOR",
				.sendFunc    = Si46xx_Msg_FlashLoad_EraseSector_sendFunc,
				.receiveFunc = Si46xx_Msg_ReadReply_receiveFunc
		},	/* SI46XX_MSG_FLASH_ERASE_SECTOR */
		{
				.msgIndex    = SI46XX_MSG_FLASH_ERASE_CHIP,
				.msgName	 = "SI46XX_MSG_FLASH_ERASE_CHIP",
				.sendFunc    = Si46xx_Msg_FlashLoad_EraseChip_sendFunc,
				.receiveFunc = Si46xx_Msg_ReadReply_receiveFunc
		},	/* SI46XX_MSG_FLASH_ERASE_CHIP */
		{
				.msgIndex    = SI46XX_MSG_LOAD_INIT,
				.msgName	 = "SI46XX_MSG_LOAD_INIT",
				.sendFunc    = Si46xx_Msg_LoadInit_sendFunc,
				.receiveFunc = Si46xx_Msg_ReadReply_receiveFunc
		},	/* SI46XX_MSG_LOAD_INIT */
		{
				.msgIndex    = SI46XX_MSG_BOOT,
				.msgName	 = "SI46XX_MSG_BOOT",
				.sendFunc    = Si46xx_Msg_Boot_sendFunc,
				.receiveFunc = Si46xx_Msg_ReadReply_receiveFunc
		},	/* SI46XX_MSG_BOOT */
		{
				.msgIndex    = SI46XX_MSG_REFRESH_SYS_STATE,
				.msgName	 = "SI46XX_MSG_REFRESH_SYS_STATE",
				.sendFunc    = Si46xx_Msg_GetSysState_sendFunc,
				.receiveFunc = Si46xx_Msg_GetSysState_receiveFunc
		},	/* SI46XX_MSG_REFRESH_SYS_STATE */
		{
				.msgIndex    = SI46XX_MSG_GET_DIGITAL_SERVICE_LIST,
				.msgName	 = "SI46XX_MSG_GET_DIGITAL_SERVICE_LIST",
				.sendFunc    = Si46xx_Msg_GetDigitalServiceList_sendFunc,
				.receiveFunc = Si46xx_Msg_GetDigitalServiceList_receiveFunc
		},  /* SI46XX_MSG_GET_DIGITAL_SERVICE_LIST */
		{
				.msgIndex    = SI46XX_MSG_DAB_TUNE_FREQ,
				.msgName	 = "SI46XX_MSG_DAB_TUNE_FREQ",
				.sendFunc    = Si46xx_Msg_DABtuneFreq_sendFunc,
				.receiveFunc = Si46xx_Msg_DABtuneFreq_receiveFunc
		},  /* SI46XX_MSG_DAB_TUNE_FREQ */
		{
				.msgIndex    = SI46XX_MSG_START_DIGITAL_SERVICE,
				.msgName	 = "SI46XX_MSG_START_DIGITAL_SERVICE",
				.sendFunc    = Si46xx_Msg_StartDigitalService_sendFunc,
				.receiveFunc = Si46xx_Msg_StartDigitalService_receiveFunc
		},  /* SI46XX_MSG_START_DIGITAL_SERVICE */
		{
				.msgIndex    = SI46XX_MSG_STOP_DIGITAL_SERVICE,
				.msgName	 = "SI46XX_MSG_STOP_DIGITAL_SERVICE",
				.sendFunc    = Si46xx_Msg_StopDigitalService_sendFunc,
				.receiveFunc = Si46xx_Msg_StopDigitalService_receiveFunc
		},  /* SI46XX_MSG_STOP_DIGITAL_SERVICE */
		{
				.msgIndex    = SI46XX_MSG_GET_DIGITAL_SERVICE_DATA,
				.msgName	 = "SI46XX_MSG_GET_DIGITAL_SERVICE_DATA",
				.sendFunc    = Si46xx_Msg_GetDigitalServiceData_sendFunc,
				.receiveFunc = Si46xx_Msg_GetDigitalServiceData_receiveFunc
		},  /* SI46XX_MSG_GET_DIGITAL_SERVICE_DATA */
		{
				.msgIndex	 = SI46XX_MSG_GET_EVENT_STATUS,
				.msgName	 = "SI46XX_MSG_GET_EVENT_STATUS",
				.sendFunc	 = Si46xx_Msg_GetEventStatus_sendFunc,
				.receiveFunc = Si46xx_Msg_GetEventStatus_receiveFunc
		},	/* SI46XX_MSG_GET_EVENT_STATUS */
		{
				.msgIndex	 = SI46XX_MSG_GET_ENSEMBLE_INFO,
				.msgName	 = "SI46XX_MSG_GET_ENSEMBLE_INFO",
				.sendFunc	 = Si46xx_Msg_GetEnsembleInfo_sendFunc,
				.receiveFunc = Si46xx_Msg_GetEnsembleInfo_receiveFunc
		},	/* SI46XX_MSG_GET_ENSEMBLE_INFO */
		{
				.msgIndex	 = SI46XX_MSG_SET_FREQ_LIST,
				.msgName	 = "SI46xx_MSG_SET_FREQ_LIST",
				.sendFunc	 = Si46xx_Msg_SetFreqList_sendFunc,
				.receiveFunc = Si46xx_Msg_SetFreqList_receiveFunc
		},	/* SI46xx_MSG_SET_FREQ_LIST */
		{
				.msgIndex    = SI46XX_MSG_GET_FREQ_LIST,
				.msgName	 = "SI46XX_MSG_GET_FREQ_LIST",
				.sendFunc    = Si46xx_Msg_GetFreqList_sendFunc,
				.receiveFunc = Si46xx_Msg_GetFreqList_receiveFunc
		}	,/* SI46XX_MSG_GET_FREQ_LIST     */
		{
				.msgIndex    = SI46XX_MSG_GET_SERVICE_INFO,
				.msgName	 = "SI46XX_MSG_GET_SERVICE_INFO",
				.sendFunc    = Si46xx_Msg_GetServiceInfo_sendFunc,
				.receiveFunc = Si46xx_Msg_GetServiceInfo_receiveFunc
		}	/* SI46XX_MSG_GET_SERVICE_INFO     */
};


/* Basic SPI routines ---------------------------------------------------------------- */

void Si46xx_Push(Si46xx_msg_en msgIndex)
{
	cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[msgIndex]);
}


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

	if(Si46xxCfg.isrState == ISR_INACTIVE) // Wait per default after each message
	{
		Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT);
	}

	// TODO: hier dann eigwentlich das Wait erst hin ohne ISR, v.a. beim Firmqare übertragen

	// Workaround 2 SPI-Master: Auf Input stellen
	if(SPI_Set_Input == 1)
	{
		Set_SPI_GPIO_Listen();
	}
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
	Si46xx_statusType status = Si46xx_BUSY;
	Si46xx_Status_Values_dt * deviceStatus = &Si46xxCfg.deviceStatus;

	switch(Si46xx_SPIgetStatus(Si46xxCfg.hspi, data, len))
	{
		case HAL_OK:
			progress_StatusBytes(deviceStatus, &data[1]); // TODO Passt das noch?
			//progress_StatusBytes(deviceStatus, &spiBuffer[1]); ortiginal...

			if(deviceStatus->ERR_CMD == Si46xx_ERR_ERROR)
			{
				printf("Si46xx.c Command error, Bad command, see reply byte 4 for details\n");
				status = Si46xx_MESSAGE_ERROR;
			}

			if // Reset device error
			(
				deviceStatus->ARBERR == Si46xx_ERR_ERROR || /* An arbiter overflow has occurred. The only way to recover is for the user to reset the chip. */
				deviceStatus->ERRNR  == Si46xx_ERR_ERROR    /* Fatal error has occurred. The only way to recover is for the user to reset the chip.*/
			)
			{
				printf("Si46xx: IC Error\n");
				status = Si46xx_DEVICE_ERROR;
			}

			else if // Resend last command error
			(
					deviceStatus->CMDOFERR == Si46xx_ERR_ERROR || /* The command interface has overflowed, and data has been lost */
					deviceStatus->REPOFERR == Si46xx_ERR_ERROR    /* The reply interface has underflowed, and bad data has been returned to the user */
			)
			{
				printf("Si46xx: Message Error\n");
				status = Si46xx_MESSAGE_ERROR;
			}

			else if(deviceStatus->CTS == Si46xx_CTS_READY) // Ready for next command
			{

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

	if(Si46xxCfg.deviceStatus.STCINT == Si46xx_STCINT_COMPLETE) // TODO: muss nach der Funktion zum Refreshen...
	{
		printf("sCurFreq_%d\n", sizeof(DAB_frequency_dt));
		CDC_Transmit_FS(&DAB_frequency_list[Si46xxCfg.freqIndex], sizeof(DAB_frequency_dt));
	}

	return status;
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



/* Message functions to be called from stack ------------------- */

// Generic read reply
Si46xx_statusType Si46xx_Msg_ReadReply_receiveFunc()
{
	uint8_t data[5];

	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 4);

	if(state != Si46xx_OK)
	{
		//printf("Si46xx_Msg_ReadReply_receiveFunc: NOT okay (evtl busy)!\n"); // TODO: Hier auch einen Timeout einbauen?
		return state;
	}

	//printf("Si46xx_Msg_ReadReply_receiveFunc: okay\n");

	return state;
}

HAL_StatusTypeDef Si46xx_Msg_HostLoad_sendFunc()
{
	HAL_StatusTypeDef state = HAL_OK;
	uint32_t i = 0;

	Si46xx_firmware_dt * firmware = &Si46xxCfg.firmware;

	uint32_t size = (firmware->fwBufSize > 4092 ? 4092 : firmware->fwBufSize);

	spiBuffer[0] = SI46XX_HOST_LOAD;
	spiBuffer[1] = 0x00;
	spiBuffer[2] = 0x00;
	spiBuffer[3] = 0x00;


	// If the firmware comes from the uC Flash
	if(firmware->current_fw_source == FW_SRC_UC)
	{
		for(i=0; i<size; i++)
		{
			spiBuffer[i+4] = *firmware->fwBufPtr;

			firmware->fwBufPtr++;
		}
	}
	// If the firmware should be transferred from the USB buffer
	else if(firmware->current_fw_source == FW_SRC_USB)
	{
		cdc_ringbufRx_get((uint8_t *) (spiBuffer+4), (size_t *) &size);
	}
	else // Firmware source not implemented
	{
		state = HAL_ERROR;
		return state;
	}

	SI46XX_CS_ON();
	state = HAL_SPI_Transmit_IT(Si46xxCfg.hspi, spiBuffer, size + 4);
	// CS_OFF by INT

	Si46xx_SetWaitTime(50); // TODO: wird von der Fertig-Funktion überschrieben...

	return state;
}

// Sub Function FLASH_LOAD_IMG of FLASH_LOAD
/* Load a firmware image or patch from flash.
 * This is the same as the FLASH_LOAD command but is represented here following the flash subcommand format.
 */
HAL_StatusTypeDef Si46xx_Msg_FlashLoad_LoadImage_sendFunc()
{
	uint8_t data[12];
	HAL_StatusTypeDef state = HAL_OK;

	data[0] = SI46XX_FLASH_LOAD;
	data[1] = SI46XX_FLASH_LOAD_IMG;
	data[2] = 0x00;
	data[3] = 0x00;

	data[4] = (Si46xxCfg.firmware_flash_address >>  0) & 0xFF;
	data[5] = (Si46xxCfg.firmware_flash_address >>  8) & 0xFF;
	data[6] = (Si46xxCfg.firmware_flash_address >> 16) & 0xFF;
	data[7] = (Si46xxCfg.firmware_flash_address >> 24) & 0xFF;

	data[8] = 0x00;
	data[9] = 0x00;
	data[10] = 0x00;
	data[11] = 0x00;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 12);

	// Workaround, wenn sich beide die Leitung teilen: Wenn CS losgelassen wird, auf Eingang Schalten
	SPI_Set_Input = 1;

	return state;
}

// Sub Function FLASH_WRITE_BLOCK of FLASH_LOAD
/* Write a block of bytes to the flash.
 * All the bytes on flash that are written must have been previously erased to 0xFF with the FLASH_ERASE_CHIP or FLASH_ERASE_SECTOR subcommands.
 */
HAL_StatusTypeDef Si46xx_Msg_FlashLoad_WriteBlock_sendFunc()
{
	HAL_StatusTypeDef state = HAL_OK;
	uint32_t i = 0;

	Si46xx_firmware_dt * firmware = &Si46xxCfg.firmware;

	uint32_t size = (firmware->fwBufSize > 256 ? 256 : firmware->fwBufSize);

	spiBuffer[0] = SI46XX_FLASH_LOAD;
	spiBuffer[1] = SI46XX_FLASH_WRITE_BLOCK;
	spiBuffer[2] = 0x0C;
	spiBuffer[3] = 0xED;

	// PAD0...3
	spiBuffer[4] = 0x00;
	spiBuffer[5] = 0x00;
	spiBuffer[6] = 0x00;
	spiBuffer[7] = 0x00;

	// FLASH_ADDR
	spiBuffer[ 8] = (firmware->current_flash_address >>  0) & 0xFF;
	spiBuffer[ 9] = (firmware->current_flash_address >>  8) & 0xFF;
	spiBuffer[10] = (firmware->current_flash_address >> 16) & 0xFF;
	spiBuffer[11] = (firmware->current_flash_address >> 24) & 0xFF;

	// SIZE
	spiBuffer[12] = (size >>  0) & 0xFF;
	spiBuffer[13] = (size >>  8) & 0xFF;
	spiBuffer[14] = (size >> 16) & 0xFF;
	spiBuffer[15] = (size >> 24) & 0xFF;

	// If the firmware comes from the uC Flash
	if(firmware->current_fw_source == FW_SRC_UC)
	{
		for(i=0; i<size; i++)
		{
			spiBuffer[i+16] = *firmware->fwBufPtr;

			firmware->fwBufPtr++;
		}
	}
	// If the firmware should be transferred from the USB buffer
	else if(firmware->current_fw_source == FW_SRC_USB)
	{
		cdc_ringbufRx_get((uint8_t *) (spiBuffer+16), (size_t *) &size);
	}
	else // Firmware source not implemented
	{
		state = HAL_ERROR;
		return state;
	}

	SI46XX_CS_ON();
	state = HAL_SPI_Transmit_IT(Si46xxCfg.hspi, spiBuffer, size + 16);
	// CS_OFF by INT

	Si46xx_SetWaitTime(50); // TODO: wird von der Fertig-Funktion überschrieben...

	firmware->current_flash_address += size;

	return state;
}

HAL_StatusTypeDef Si46xx_Msg_FlashLoad_EraseSector_sendFunc() // TODO: Adresse noch nicht definiert
{
	uint8_t data[8];
	HAL_StatusTypeDef state = HAL_OK;

	data[0] = SI46XX_FLASH_LOAD;
	data[1] = SI46XX_FLASH_ERASE_SECTOR;
	data[2] = 0xC0;
	data[3] = 0xDE;

	/* Starting address on flash of the sector to erase, byte offset from the start of flash.
	 * Note: sector_addr[23..0] are used, sector_addr[31..24] are ignored.
	 * The least significant bits that would be masked by the sector boundary are ignored.*/
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 8);

	return state;
}

HAL_StatusTypeDef Si46xx_Msg_FlashLoad_EraseChip_sendFunc()
{
	uint8_t data[4];
	HAL_StatusTypeDef state = HAL_OK;

	data[0] = SI46XX_FLASH_LOAD;
	data[1] = SI46XX_FLASH_ERASE_CHIP;
	data[2] = 0xDE;
	data[3] = 0xC0;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 4);

	return state;
}

HAL_StatusTypeDef Si46xx_Msg_LoadInit_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_OK;

	data[0] = SI46XX_LOAD_INIT;
	data[1] = 0x00;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);

	return state;
}

HAL_StatusTypeDef Si46xx_Msg_Boot_sendFunc()
{
	HAL_StatusTypeDef state = HAL_OK;

	//Si46xxCfg.timeoutValue = 300; TODO

	spiBuffer[0] = SI46XX_BOOT;
	spiBuffer[1] = 0x00;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, spiBuffer, 2);

	return state;
}

// SI46XX_MSG_POWER_UP
HAL_StatusTypeDef Si46xx_Msg_PowerUp_sendFunc()
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

	Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT); // ms TODO erstmal testweise, der rebootet iwie 3x
	Si46xxCfg.isrState = ISR_INACTIVE; // This function doesn't offer interrupts, as it's before the bootloader patch

	return state;
}



// SI46XX_MSG_REFRESH_SYS_STATE
HAL_StatusTypeDef Si46xx_Msg_GetSysState_sendFunc()
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

Si46xx_statusType Si46xx_Msg_GetSysState_receiveFunc()
{
	uint8_t data[6];

	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 5);

	if(state != Si46xx_OK)
	{
		return state;
	}

	Si46xxCfg.image = data[5];

	//printf("Si46xx: Image %d\n", Si46xxCfg.image);
	printf("ssys_%d\n", Si46xxCfg.image);

	return state;
}

/* SI46XX_MSG_GET_DIGITAL_SERVICE_LIST */
HAL_StatusTypeDef Si46xx_Msg_GetDigitalServiceList_sendFunc()
{
	uint8_t data[6];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		// If tune is not marked as complete, check once if it is now
		if(Si46xxCfg.deviceStatus.STCINT == Si46xx_STCINT_INCOMPLETE)
		{
			/*state = */Si46xx_SPIgetAnalyzeStatus(data, 5); // State nicht aktualisieren, soll busy bleiben
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
	//uint8_t data[7]; // TODO 7, temporär festgelegt, was ist der maximale Wert hier?
	uint8_t * data = spiBuffer;
	dab_channel_dt * chan = &Si46xxCfg.channelData;

	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 6);
	//uint16_t length = 0;

	if(state != Si46xx_OK)
	{
		return state;
	}

	// clear old channel data
	memset(chan, 0x00, sizeof(dab_channel_dt));

	chan->channel = Si46xxCfg.freqIndex;

	// TODO: Version und so...

	chan->listSize = (uint16_t) data[5] + (data[6] << 8);

	//length = (uint16_t) data[5] + (data[6] << 8);
	printf("Si46xx: Size of service list: %d bytes \n", chan->listSize);

	//Si46xxCfg.image = spiBuffer[5];

	state = Si46xx_SPIgetAnalyzeStatus(data, 6 + chan->listSize);

	if(state != Si46xx_OK || chan->listSize == 0)
	{
		return state;
	}

	// Send message for debug
	printf("sSrvList_%d\n", 6+chan->listSize);
	CDC_Transmit_FS(data, 6+chan->listSize);

	/* TODO: Digitale Serviceliste auswerten:
	 * 8.5
 Finding a Digital Service
Before starting a data service, the host must collect information about the services that exist in the ensemble. This is
done using the GET_DIGITAL_SERVICE_LIST host command. The service list definitions for DAB is described in
“8.5.1. DAB/DMB Radio Service List”, the following table shows the format of the GET_DIGITAL_SERVICE_LIST
host command.
	 *
	 */

	// TODO: In richtige Liste einbauen, damit man das später auch abrufen kann... Zunächst mit fixer Svc/Cmp-Listenbreite?

	uint8_t * bufPtr = data + 5; // Point to List Size
	chan->listSize = bufPtr[0] + (bufPtr[1] << 8);

	chan->version = bufPtr[2] + (bufPtr[3] << 8);

	//uint8_t numberServices = bufPtr[4];
	chan->numServices = bufPtr[4];
	printf("Si46xx: # of services: %d \n", chan->numServices);

	uint8_t position = 8; // initial...
	bufPtr += position;

	// Loop through services
	for(uint8_t i=0; i<chan->numServices; i++)
	{
		chan->services[i].serviceID = bufPtr[0] + (bufPtr[1] << 8) + (bufPtr[2] << 16) + (bufPtr[3] << 24);
		//uint32_t serviceID = bufPtr[0] + (bufPtr[1] << 8) + (bufPtr[2] << 16) + (bufPtr[3] << 24);

		chan->services[i].pdFlag = bufPtr[4] & 0x01;
		//uint8_t pdFlag = bufPtr[4] & 0x01;

		struct serviceID_P // P/D=0
		{
			uint16_t SRV_REF:12;
			uint8_t CountryID:4;
			uint16_t RFU;
		};

		struct serviceID_D // P/D=1
		{
			uint32_t SRV_REF:20;
			uint8_t CountyID:4;
			uint8_t ECC;
		};

		uint8_t numberComponents = bufPtr[5] & 0x0F;
		char serviceLabel[16+1];
		memcpy(serviceLabel, &bufPtr[8], 16);
		serviceLabel[16] = '\0';

		printf("Si46xx: ServiceID: %lX, P/D: %d, Label: %s \n", chan->services[i].serviceID, chan->services[i].pdFlag, serviceLabel);

		struct serviceID_P * srvID_P;
		struct serviceID_D * srvID_D;

		switch(chan->services[i].pdFlag)
		{
			case 0:
				srvID_P = (struct serviceID_P*) chan->services[i].serviceID;
				printf("Si46xx: SRV_REF: %X CountryID: %d\n", srvID_P->SRV_REF, srvID_P->CountryID);
				break;

			case 1:
				srvID_D = (struct serviceID_D*) chan->services[i].serviceID;
				printf("Si46xx: SRV_REF: %X CountryID: %d, ECC: %X\n", srvID_D->SRV_REF, srvID_D->CountyID, srvID_D->ECC);
				break;
		}

		printf("Si46xx: # of components: %d \n", numberComponents);

		// Jump to components
		//position += 24;
		bufPtr += 24;

		// Loop through components in the service
		for(uint8_t j=0; j<numberComponents; j++)
		{
			dab_component_t * comp = &chan->services[i].components[j];

			// Adjust pointer to current component
			//bufPtr += position;

			comp->tmID = bufPtr[1] >> 6;
			comp->componentID = 0;

			//uint8_t tmID = bufPtr[1] >> 6;
			//uint16_t componentID = 0;

			switch(comp->tmID) // Component ID depends on the TM ID
			{
				case 0: // TMId=00 (MSC stream audio)
				case 1: // TMId=01 (MSC stream data)
				case 2: // TMId=10 (Reserved)
					comp->componentID = bufPtr[0] & 0x3F;
					break;

				case 3: // TMId=11 (MSC packet data)
					comp->componentID = bufPtr[0] + ((bufPtr[1] & 0x0F) << 8);
					// DGFlag is on Bit 13
					break;
			}

			comp->ascTy_dscTy = bufPtr[2] >> 2;
			//uint8_t ascTy_dscTy = bufPtr[2] >> 2;


			printf("Si46xx:      TMId: %X, ComponentID: %X, ASCTy/DSCTy: %d\n", comp->tmID, comp->componentID, comp->ascTy_dscTy);

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

	// freqIndex needs to be defined to a channel first
	//Si46xxCfg.freqIndex = DAB_Chan_5C;

	// Check configuration
	if(Si46xxCfg.freqIndex > 47)
	{
		return HAL_ERROR;
	}

	Si46xxCfg.deviceStatus.STCINT = Si46xx_STCINT_COMPLETE; // für den ersten... TODO ist das noch nötig?

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
	uint8_t data[5];
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 4);

	if(state != Si46xx_OK)
	{
		return state;
	}

	// Reset the indexes, as they are different on this ensemble
	Si46xxCfg.wantedService.serviceID = 0;
	Si46xxCfg.wantedService.componentID = 0;

	if(Si46xxCfg.deviceStatus.STCINT == Si46xx_STCINT_INCOMPLETE) // TODO: Hier zu fragen ist nutzlos, kurz nach aufrufen ist tuning ja nicht durch
	{
		printf("Si46xx: Tune not complete... \n");
	}
	else
	{
		printf("Si46xx: Tune complete! \n");
	}


	return state;
}

// SI46XX_MSG_START_DIGITAL_SERVICE
/* START_DIGITAL_SERVICE starts an audio or data service.
 * This command is used for DAB audio and data services. To determine what services exist in an ensemble please use the GET_DIGITAL_SERVICE_LIST command.
 * In the case of starting an audio service, it is not required to stop a currently running audio service/program before starting a new one.
 * The currently running audio service will be stopped automatically when the new service is requested.
 */
HAL_StatusTypeDef Si46xx_Msg_StartDigitalService_sendFunc()
{
	uint8_t data[0xC];
	HAL_StatusTypeDef state = HAL_BUSY;

	/*if(Si46xxCfg.wantedService.serviceID == 0 || Si46xxCfg.wantedService.componentID == 0) TODO: hier stattdessen prüfen, ob Index vorhanden ist/sind
	{
		state = HAL_ERROR;
		return state;
	}*/

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		// Get current service and component ID from wanted indexes
		uint32_t serviceID   = Si46xxCfg.channelData.services[Si46xxCfg.wantedService.serviceID].serviceID;
		uint32_t componentID = Si46xxCfg.channelData.services[Si46xxCfg.wantedService.serviceID].components[Si46xxCfg.wantedService.componentID].componentID;

		data[0x0] = SI46XX_START_DIGITAL_SERVICE;
		data[0x1] = 0x00;
		data[0x2] = 0x00;
		data[0x3] = 0x00;


		data[0x4] = (serviceID >>  0) & 0xFF;
		data[0x5] = (serviceID >>  8) & 0xFF;
		data[0x6] = (serviceID >> 16) & 0xFF;
		data[0x7] = (serviceID >> 24) & 0xFF;

		data[0x8] = (componentID >>  0) & 0xFF;
		data[0x9] = (componentID >>  8) & 0xFF;
		data[0xA] = (componentID >> 16) & 0xFF;
		data[0xB] = (componentID >> 24) & 0xFF;


		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 0xC);

		printf("Si46xx: Starting digital service...\n");
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_StartDigitalService_receiveFunc()
{
	// TODO: das ist eigentlich eine generische Funktion, wie auch DABtuneFreq
	uint8_t data[5];
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 4);

	return state;
}

// SI46XX_MSG_STOP_DIGITAL_SERVICE
HAL_StatusTypeDef Si46xx_Msg_StopDigitalService_sendFunc()
{
	uint8_t data[0xC];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		// Get current service and component ID from wanted indexes
		uint32_t serviceID   = Si46xxCfg.channelData.services[Si46xxCfg.wantedService.serviceID].serviceID;
		uint32_t componentID = Si46xxCfg.channelData.services[Si46xxCfg.wantedService.serviceID].components[Si46xxCfg.wantedService.componentID].componentID;

		data[0x0] = SI46XX_STOP_DIGITAL_SERVICE;
		data[0x1] = 0x00;
		data[0x2] = 0x00;
		data[0x3] = 0x00;

		data[0x4] = (serviceID >>  0) & 0xFF;
		data[0x5] = (serviceID >>  8) & 0xFF;
		data[0x6] = (serviceID >> 16) & 0xFF;
		data[0x7] = (serviceID >> 24) & 0xFF;

		data[0x8] = (componentID >>  0) & 0xFF;
		data[0x9] = (componentID >>  8) & 0xFF;
		data[0xA] = (componentID >> 16) & 0xFF;
		data[0xB] = (componentID >> 24) & 0xFF;


		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 0xC);
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_StopDigitalService_receiveFunc()
{
	// TODO: das ist eigentlich eine generische Funktion, wie auch DABtuneFreq
	uint8_t data[5];
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 4);

	return state;
}

//SI46XX_MSG_GET_DIGITAL_SERVICE_DATA
HAL_StatusTypeDef Si46xx_Msg_GetDigitalServiceData_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_BUSY;

	enum statusOnly
	{
		STATUS_NORMAL = 0x0, /* Return everything */
		STATUS_POLL   = 0x1  /* Only return interrupt source and available buffers information */
	}GetDigitalServiceData_StatusOnly;

	enum ACK
	{
		DONT_ACK	= 0x0,	/* Don't acknowledge the interrupt	*/
		ACK			= 0x1	/* Acknowledging the interrupt will clear the DSRVINT bit and the interrupt source bits. */
	}GetDigitalServiceData_ACK;

	GetDigitalServiceData_StatusOnly = STATUS_NORMAL;
	GetDigitalServiceData_ACK		 = ACK;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		data[0] = SI46XX_GET_DIGITAL_SERVICE_DATA;
		data[1] = (GetDigitalServiceData_StatusOnly << 4) | GetDigitalServiceData_ACK;

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);

		// As maximum wait time:
		//Si46xx_SetWaitTime(100); // TODO Timeout und so... muss eine andere Funktion gebaut werden
	}

	return state;
}

// TODO: hier ist noch gar nichts fertig!!
Si46xx_statusType Si46xx_Msg_GetDigitalServiceData_receiveFunc()
{
	return Si46xx_OK;
}



// SI46XX_MSG_GET_EVENT_STATUS
HAL_StatusTypeDef Si46xx_Msg_GetEventStatus_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
			data[0] = SI46XX_DAB_GET_EVENT_STATUS;
			data[1] = 0x00 | 1; // TODO: hier ACK einbauen

			state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_GetEventStatus_receiveFunc()
{
	uint8_t data[10];
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 9);

	// Attach interrupt infos to signals
	Si46xxCfg.events.service_list_int	= (data[5]>>0) & 0x01;
	Si46xxCfg.events.freq_info_int		= (data[5]>>1) & 0x01;

	return state;
}

// SI46XX_MSG_GET_ENSEMBLE_INFO
HAL_StatusTypeDef Si46xx_Msg_GetEnsembleInfo_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
			data[0] = SI46XX_GET_ENSEMBLE_INFO;
			data[1] = 0x00;

			state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_GetEnsembleInfo_receiveFunc()
{
	uint8_t data[0x1A];
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 0x19);

	Si46xxCfg.channelData.ensembleID = data[5] + (data[6]<<8);
	memcpy(Si46xxCfg.channelData.ensembleLabel, &data[7], 16);

	printf("sEnsemble_%u_%s\n", Si46xxCfg.channelData.ensembleID, Si46xxCfg.channelData.ensembleLabel);
	// TODO Informationen auswerten

	return state;
}


//SI46xx_MSG_SET_FREQ_LIST
HAL_StatusTypeDef Si46xx_Msg_SetFreqList_sendFunc() // TODO: Muss getestet werden!
{
	uint8_t data[196] = {0,};
	HAL_StatusTypeDef state = HAL_BUSY;


	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		data[0] = SI46XX_SET_FREQ_LIST;
		data[1] = (DAB_Chan_SIZE & 0xFF);
		data[2] = 0; // Normal guaranteed tuning range 168 MHz to 240 MHz
		data[3] = 0;

		for(uint8_t i=0, j=0; i<DAB_Chan_SIZE; i++, j+=4)
		{
			data[4+j+0] = (DAB_frequency_list[i].freq & 0x000000FF) >>  0;
			data[4+j+1] = (DAB_frequency_list[i].freq & 0x0000FF00) >>  8;
			data[4+j+2] = (DAB_frequency_list[i].freq & 0x00FF0000) >> 16;
			data[4+j+3] = (DAB_frequency_list[i].freq & 0xFF000000) >> 24;

			if(i>47) // maximum for list
			{
				break;
			}
		}

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, sizeof(data));

		// Reset list status as we need to check it afterwards
		Si46xxCfg.freqencyListStatus = FREQ_LIST_INVALID;
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_SetFreqList_receiveFunc()
{
	// TODO: das ist eigentlich eine generische Funktion, wie auch DABtuneFreq
	uint8_t data[5];
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 4);

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
	uint8_t data[201];
	uint8_t * dataPtr;
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 200);

	if(state != Si46xx_OK)
	{
		return state;
	}

	printf("Si46xx: Frequencies: %d\n", data[5]);

	dataPtr = data + 9;

	// Set frequency list status to valid, reset it to invalid if it doesn't match
	Si46xxCfg.freqencyListStatus = FREQ_LIST_VALID;

	for(uint8_t i=0, j=0; i<DAB_Chan_SIZE; i++, j+=4)
	{
		// saved frequency at current point
		uint32_t currentSpiFreq = (uint32_t) dataPtr[j]+(dataPtr[j+1]<<8)+(dataPtr[j+2]<<16)+(dataPtr[j+3]<<24);

		if(DAB_frequency_list[i].freq != currentSpiFreq)
		{
			printf("Si46xx_Msg_GetFreqList_receiveFuncFrequencies index %d doesn't match!\n", i);

			Si46xxCfg.freqencyListStatus = FREQ_LIST_INVALID;
			break;
		}
	}

	// Transmit frequency list to terminal if valid
	if(Si46xxCfg.freqencyListStatus == FREQ_LIST_VALID)
	{
		// Send frequency list
		printf("sFreqList_%d\n", sizeof(DAB_frequency_list));
		CDC_Transmit_FS((uint8_t*) DAB_frequency_list, sizeof(DAB_frequency_list));
	}

	/*for(uint8_t i=0x08; i<0xC7; i+=4)
	{
		printf("Si46xx: Freq at %d: %lu \n", i,
				(uint32_t) data[i+1]+(data[i+2]<<8)+(data[i+3]<<16)+(data[i+4]<<24));
	}*/

	return state;
}

// SI46XX_MSG_GET_SERVICE_INFO
HAL_StatusTypeDef Si46xx_Msg_GetServiceInfo_sendFunc()
{
	uint8_t data[7];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		// Get current service from wanted indexes
		uint32_t serviceID   = Si46xxCfg.channelData.services[Si46xxCfg.wantedService.serviceID].serviceID;

		data[0] = SI46XX_GET_SERVICE_INFO;
		data[1] = 0x00;
		data[2] = 0x00;
		data[3] = 0x00;

		data[0x4] = (serviceID >>  0) & 0xFF;
		data[0x5] = (serviceID >>  8) & 0xFF;
		data[0x6] = (serviceID >> 16) & 0xFF;
		data[0x7] = (serviceID >> 24) & 0xFF;

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 7);
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_GetServiceInfo_receiveFunc()
{
	uint8_t data[0x1A];
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 0x19);

	// TODO auswerten und in Variablen packen

	return state;
}

