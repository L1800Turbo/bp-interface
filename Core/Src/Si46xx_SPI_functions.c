/*
 * Si46xx_SPI_functions.c
 *
 * Functions to be called by state machine
 *
 *  Created on: Dec 31, 2021
 *      Author: kai
 */
#include "Si46xx_SPI_functions.h"
#include "Si46xx_DAB_digital_service_data.h"
#include <stdio.h>
#include <string.h>

#include "usbd_cdc_if.h" // To get USB data from the ring buffer

enum ACK
{
	DONT_ACK = 0,
	ACK	     = 1
};

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
HAL_StatusTypeDef Si46xx_Msg_GetPartInfo_sendFunc();
Si46xx_statusType Si46xx_Msg_GetPartInfo_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_GetSysState_sendFunc();
Si46xx_statusType Si46xx_Msg_GetSysState_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_GetFuncInfo_sendFunc();
Si46xx_statusType Si46xx_Msg_GetFuncInfo_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_SetProperty_sendFunc();
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
HAL_StatusTypeDef Si46xx_Msg_DigradStatus_sendFunc();
Si46xx_statusType Si46xx_Msg_DigradStatus_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_GetEventStatus_sendFunc();
Si46xx_statusType Si46xx_Msg_GetEventStatus_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_GetEnsembleInfo_sendFunc();
Si46xx_statusType Si46xx_Msg_GetEnsembleInfo_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_SetFreqList_sendFunc();
Si46xx_statusType Si46xx_Msg_SetFreqList_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_GetFreqList_sendFunc();
Si46xx_statusType Si46xx_Msg_GetFreqList_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_DabGetComponentInfo_sendFunc();
Si46xx_statusType Si46xx_Msg_DabGetComponentInfo_receiveFunc();
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
				.msgIndex    = SI46XX_MSG_GET_PART_INFO,
				.msgName	 = "SI46XX_MSG_GET_PART_INFO",
				.sendFunc    = Si46xx_Msg_GetPartInfo_sendFunc,
				.receiveFunc = Si46xx_Msg_GetPartInfo_receiveFunc
		},	/* SI46XX_MSG_GET_PART_INFO */
		{
				.msgIndex    = SI46XX_MSG_REFRESH_SYS_STATE,
				.msgName	 = "SI46XX_MSG_REFRESH_SYS_STATE",
				.sendFunc    = Si46xx_Msg_GetSysState_sendFunc,
				.receiveFunc = Si46xx_Msg_GetSysState_receiveFunc
		},	/* SI46XX_MSG_REFRESH_SYS_STATE */
		{
				.msgIndex    = SI46XX_MSG_GET_FUNC_INFO,
				.msgName	 = "SI46XX_MSG_GET_FUNC_INFO",
				.sendFunc    = Si46xx_Msg_GetFuncInfo_sendFunc,
				.receiveFunc = Si46xx_Msg_GetFuncInfo_receiveFunc
		},	/* SI46XX_MSG_GET_FUNC_INFO */
		{
				.msgIndex    = SI46XX_MSG_SET_PROPERTY,
				.msgName	 = "SI46XX_MSG_SET_PROPERTY",
				.sendFunc    = Si46xx_Msg_SetProperty_sendFunc,
				.receiveFunc = Si46xx_Msg_ReadReply_receiveFunc
		},	/* SI46XX_MSG_SET_PROPERTY */
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
				.msgIndex	 = SI46XX_MSG_DIGRAD_STATUS,
				.msgName	 = "SI46XX_MSG_DIGRAD_STATUS",
				.sendFunc	 = Si46xx_Msg_DigradStatus_sendFunc,
				.receiveFunc = Si46xx_Msg_DigradStatus_receiveFunc
		},	/* SI46XX_MSG_DIGRAD_STATUS */
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
		},  /* SI46XX_MSG_GET_FREQ_LIST     */
		{
				.msgIndex    = SI46XX_MSG_DAB_GET_COMPONENT_INFO,
				.msgName	 = "SI46XX_MSG_DAB_GET_COMPONENT_INFO",
				.sendFunc    = Si46xx_Msg_DabGetComponentInfo_sendFunc,
				.receiveFunc = Si46xx_Msg_DabGetComponentInfo_receiveFunc
		},	/* SI46XX_MSG_DAB_GET_COMPONENT_INFO     */
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

	if(Si46xxCfg.deviceStatus.STC == Si46xx_STCINT_COMPLETE) // TODO: muss nach der Funktion zum Refreshen...
	{
		printf("sCurFreq_%d\n", sizeof(DAB_frequency_dt));
		CDC_Transmit_FS(&DAB_frequency_list[Si46xxCfg.freqIndex], sizeof(DAB_frequency_dt));
	}

	return status;
}

// TODO: auf Dauer mit dem Status anders machen
void progress_StatusBytes(Si46xx_Status_Values_dt * status, uint8_t * data)
{
	/* Progress into status variables */
	status->CTS      = (data[0] >> 7) & 0x01;
	status->ERR_CMD  = (data[0] >> 6) & 0x01;

	Si46xxCfg.events.digital_radio_link_change = (data[0] >> 5) & 0x01; // DACQINT
	//status->DSRVINT  = (data[0] >> 4) & 0x01;

	// DSRVINT: Indicates that an enabled data component of one of the digital services requires attention.
	//          Service by sending the GET_DIGITAL_SERVICE_DATA command.
	Si46xxCfg.events.digital_service_data = (data[0] >> 4) & 0x01;

	//status->STCINT   = (data[0] & 0x01);
	// STCINT: Seek/Tune complete interrupt: Set STC flag and ACK
	if(data[0] & 0x01)
	{
		status->STC = Si46xx_STCINT_COMPLETE;

		// Set event
		Si46xxCfg.events.seek_tune_complete = Si46xx_INTERRUPT;
	}

	// DEVNTINT: Digital radio event change interrupt indicator. Indicates that a new event related to the digital
	//           radio has occurred. Service by sending the DAB_DIGRAD_STATUS command. Quellen muessen manuell aktiviert werden.
	Si46xxCfg.events.digital_radio_event_change = (data[1] >> 5) & 0x01;

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

	spiBuffer[0] = SI46XX_SPI_CMD_HOST_LOAD;
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

	data[0] = SI46XX_SPI_CMD_FLASH_LOAD;
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

	spiBuffer[0] = SI46XX_SPI_CMD_FLASH_LOAD;
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

	data[0] = SI46XX_SPI_CMD_FLASH_LOAD;
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

	data[0] = SI46XX_SPI_CMD_FLASH_LOAD;
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

	data[0] = SI46XX_SPI_CMD_LOAD_INIT;
	data[1] = 0x00;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);

	return state;
}

HAL_StatusTypeDef Si46xx_Msg_Boot_sendFunc()
{
	HAL_StatusTypeDef state = HAL_OK;

	//Si46xxCfg.timeoutValue = 300; TODO

	spiBuffer[0] = SI46XX_SPI_CMD_BOOT;
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
	data[0]  = SI46XX_SPI_CMD_POWER_UP;
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

// SI46XX_MSG_GET_PART_INFO
HAL_StatusTypeDef Si46xx_Msg_GetPartInfo_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		data[0] = SI46XX_SPI_CMD_GET_PART_INFO;
		data[1] = 0x00;

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_GetPartInfo_receiveFunc()
{
	uint8_t data[10+1];

	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, sizeof(data));

	if(state != Si46xx_OK)
	{
		return state;
	}

	Si46xxCfg.deviceInformation.chipRevision = data[5];
	Si46xxCfg.deviceInformation.romID		 = data[6];
	Si46xxCfg.deviceInformation.partNumber   = data[9] | (data[10]<<8);


	//printf("Si46xx: Image %d\n", Si46xxCfg.image);
	printf("sHwRev_%d_%d_%d\n", Si46xxCfg.deviceInformation.chipRevision,
			Si46xxCfg.deviceInformation.romID, Si46xxCfg.deviceInformation.partNumber);

	return state;
}

// SI46XX_MSG_REFRESH_SYS_STATE
HAL_StatusTypeDef Si46xx_Msg_GetSysState_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		data[0] = SI46XX_SPI_CMD_GET_SYS_STATE;
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

/* SI46XX_MSG_GET_FUNC_INFO */
HAL_StatusTypeDef Si46xx_Msg_GetFuncInfo_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		data[0] = SI46XX_SPI_CMD_GET_FUNC_INFO;
		data[1] = 0x00;

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_GetFuncInfo_receiveFunc()
{
	uint8_t data[12+1];

	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, sizeof(data));

	if(state != Si46xx_OK)
	{
		return state;
	}

	Si46xxCfg.deviceInformation.softwareRevision[0] = data[5];
	Si46xxCfg.deviceInformation.softwareRevision[1] = data[6];
	Si46xxCfg.deviceInformation.softwareRevision[2] = data[7];


	//printf("Si46xx: Image %d\n", Si46xxCfg.image);
	printf("sSwRev_%d_%d_%d\n", Si46xxCfg.deviceInformation.softwareRevision[0],
			Si46xxCfg.deviceInformation.softwareRevision[1], Si46xxCfg.deviceInformation.softwareRevision[2]);

	return state;
}

/* SI46XX_MSG_SET_PROPERTY */
HAL_StatusTypeDef Si46xx_Msg_SetProperty_sendFunc()
{
	uint8_t data[6];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		data[0] = SI46XX_SPI_CMD_SET_PROPERTY;
		data[1] = 0x00;

		data[2] =  Si46xxCfg.currentProperty.index       & 0xFF;
		data[3] = (Si46xxCfg.currentProperty.index >> 8) & 0xFF;

		data[4] =  Si46xxCfg.currentProperty.data       & 0xFF;
		data[5] = (Si46xxCfg.currentProperty.data >> 8) & 0xFF;

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, sizeof(data));
	}

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
		if(Si46xxCfg.deviceStatus.STC == Si46xx_STCINT_INCOMPLETE)
		{
			printf("Si46xx_Msg_GetDigitalServiceList_sendFunc: Si46xx_STCINT_INCOMPLETE, not fetching service list\n");
			/*state = */Si46xx_SPIgetAnalyzeStatus(data, 5); // State nicht aktualisieren, soll busy bleiben
		}

		if(Si46xxCfg.deviceStatus.STC == Si46xx_STCINT_COMPLETE)
		{
			data[0] = SI46XX_SPI_CMD_GET_DIGITAL_SERVICE_LIST;
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

		//printf("Si46xx: ServiceID: %lX, P/D: %d, Label: %s \n", chan->services[i].serviceID, chan->services[i].pdFlag, serviceLabel);

		struct serviceID_P * srvID_P;
		struct serviceID_D * srvID_D;

		switch(chan->services[i].pdFlag)
		{
			case AUDIO_SERVICE:
				srvID_P = (struct serviceID_P*) chan->services[i].serviceID;
				//printf("Si46xx: SRV_REF: %X CountryID: %d\n", srvID_P->SRV_REF, srvID_P->CountryID);
				break;

			case DATA_SERVICE:
				srvID_D = (struct serviceID_D*) chan->services[i].serviceID;
				//printf("Si46xx: SRV_REF: %X CountryID: %d, ECC: %X\n", srvID_D->SRV_REF, srvID_D->CountyID, srvID_D->ECC);
				break;
		}

		//printf("Si46xx: # of components: %d \n", numberComponents);

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
					comp->componentID = bufPtr[0] & 0x3F; // TODO: da muss TMiD eigentlich noc han den Anfang, fällt nur nicht auf, weil es 0 ist...
					break;

				case 3: // TMId=11 (MSC packet data)
					//comp->componentID = bufPtr[0] + ((bufPtr[1] & 0x0F) << 8);
					comp->componentID = bufPtr[0] | (bufPtr[1] << 8);
					// DGFlag is on Bit 13
					break;
			}

			comp->ascTy_dscTy = bufPtr[2] >> 2;
			//uint8_t ascTy_dscTy = bufPtr[2] >> 2;


			//printf("Si46xx:      TMId: %X, ComponentID: %X, ASCTy/DSCTy: %d\n", comp->tmID, comp->componentID, comp->ascTy_dscTy);

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

	//Si46xxCfg.deviceStatus.STCINT = Si46xx_STCINT_COMPLETE; // für den ersten... TODO ist das noch nötig?

	// No other tuning should be running
	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY  && Si46xxCfg.deviceStatus.STC == Si46xx_STCINT_COMPLETE)
	{
		data[0] = SI46XX_SPI_CMD_DAB_TUNE_FREQ;
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

		Si46xxCfg.deviceStatus.STC = Si46xx_STCINT_INCOMPLETE; // Reset STCINT

		Si46xxCfg.deviceStatus.VALID = NOT_VALID;
		Si46xxCfg.deviceStatus.ACQ = NO_ACQ;
		Si46xxCfg.deviceStatus.FICERR = NO_FICERR;

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

		data[0x0] = SI46XX_SPI_CMD_START_DIGITAL_SERVICE;
		data[0x1] = 0x00 | 1;
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

		data[0x0] = SI46XX_SPI_CMD_STOP_DIGITAL_SERVICE;
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

	enum ACK GetDigitalServiceData_ACK;

	GetDigitalServiceData_StatusOnly = STATUS_NORMAL;
	GetDigitalServiceData_ACK		 = ACK;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		data[0] = SI46XX_SPI_CMD_GET_DIGITAL_SERVICE_DATA;
		data[1] = (GetDigitalServiceData_StatusOnly << 4) | GetDigitalServiceData_ACK;

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);

		// As maximum wait time:
		//Si46xx_SetWaitTime(100); // TODO Timeout und so... muss eine andere Funktion gebaut werden
	}

	return state;
}

// TODO: Temporär zum testen
struct mot_status
{
	uint8_t startHeader:1; // Wenn der header fertig ist
	uint8_t startBlock:1;  // Wenn der nächste Block startet
	uint16_t currentTransportId; // Setzen, wenn der Header fertig ist

	char * currentName; // Aktueller Dateiname oder so
}mot_status;

uint8_t test[1000] = {0, };
Si46xx_statusType Si46xx_Msg_GetDigitalServiceData_receiveFunc()
{
	uint8_t data[4096] = {0,}; // TODO: Erstmal nur so festgelegt
	uint16_t byte_count = 0;

	enum data_source_en
	{
		DATA_SERVICE = 0x0, /* Indicates that the payload is from a standard data service and DATA_TYPE is DSCTy. */
		PAD_DATA     = 0x1, /* Indicates that the payload is non-DLS PAD and DATA_TYPE is DSCTy. */
		PAD_DLS      = 0x2  /* Indicates that the payload is DLS PAD and DATA_TYPE is 0. */
	};

	enum dsc_types //  Data Service Component Types: ETSI TS 101 756 Table 2b
	{
		DSC_TDC   =  5, // Transparent Data Channel (TDC), see ETSI TS 101 759
		DSC_MPEG2 = 24, // MPEG-2 Transport Stream, see ETSI TS 102 427
		DSC_MOT   = 60, // Multimedia Object Transfer (MOT), see ETSI EN 301 234
		DSC_PROP  = 61  // Proprietary service: no DSCTy signalled
	};

	// ETSI TS 102 980 Ch. 6.0
	struct dl_plus_tag
	{
		uint8_t content_type;
		uint8_t start_marker;
		uint8_t length_marker;
	};

	// according to ETSI TS 102 980 Section 7.4.5.2: XPAD DLS prefix with Si46xx API
	struct dls_str
	{
		uint8_t toggle_bit:1;
		uint8_t C_flag:1;

		enum command_field
		{
			CLEAR_DISPLAY_CMD = 0b0001,
			DL_PLUS_CMD       = 0b0010
			// ETSI: all other codes are reserved for future use
		}command;

		// Field 2
		enum charset_field
		{
			EBU_Latin_based     = 0b0000,
			UCS2_transformation = 0b0110,
			UTF8_transformation = 0b1111
		}charset;

		uint8_t link:1;

		char msg[128+1];

		// Body
		uint8_t item_toggle_bit:1;
		uint8_t item_running_bit:1;
		uint8_t number_tags:2;

	}dls_payload;

	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 0x17);

	byte_count = data[0x13] | (data[0x14] << 8);

	printf("byte_count: %u\n", byte_count);

	// temporär:
	if(byte_count > sizeof(data)-0x18) byte_count = sizeof(data)-0x18;

	state = Si46xx_SPIgetAnalyzeStatus(data, 0x17 + byte_count); // TODO state auswerten..

	enum data_source_en data_src = (data[8] >> 6) & 0x2;
	enum dsc_types dscty = data[8] & 0x3F;

	uint16_t uatype = data[0x11] | (data[0x12] << 8); //Si46xx_Msg_DabGetComponentInfo_receiveFunc hat ein Enum dafür, aber das deckt sich nicht mit Si46?
	uint16_t seg_num  = data[0x15] | (data[0x16] << 8);
	uint16_t num_segs = data[0x17] | (data[0x18] << 8);

	printf("uatype: %d, data_src: %d (dscty: %d) seg_num: %u num_segs: %u\n", uatype, data_src, dscty, seg_num, num_segs);

	// Dynamic label
	switch(data_src)
	{
		case DATA_SERVICE:
			if(dscty == DSC_MOT)
			{
				// TODO Externe Funktion mit Parametern: DSCTy, UA-Type, Payload-Len, Pointer zu den Daten
				//

				// EN 300 401 Figure 12
				struct msc_data_group
				{
					struct msc_data_group_header
					{
						// Byte 0
						uint8_t extension_flag:1;   // Extension flag: this 1-bit flag shall indicate whether the extension field is present
						uint8_t crc_flag:1;			// CRC flag: this 1-bit flag shall indicate whether there is a CRC at the end of the MSC data group
						uint8_t segment_flag:1;     // Segment flag: this 1-bit flag shall indicate whether the segment field is present, or not
						uint8_t user_access_flag:1; // User access flag: this 1-bit flag shall indicate whether the user access field is present, or not
						uint8_t data_group_type:4;  /* Data group type: this 4-bit field shall define the type of data carried in the data group data field.
													   The following types are defined for use by all data service components:
														b3 - b0
														0 0 0 0 : General data;
														0 0 0 1 : CA messages (see ETSI TS 102 367 [4]).
													   The remaining types are dependent upon the value of the DSCTy and defined by the relevant
													   document (see clause 6.3.1). */

						// Byte 1
						/* Continuity index: the binary value of this 4-bit field shall be incremented each time a MSC data group of a particular type,
						 *  with a content different from that of the immediately preceding data group of the same type, is transmitted.  */
						uint8_t continuity_index:4;
						/* Repetition index: the binary value of this 4-bit field shall signal the remaining number of repetitions of a MSC data group
						 * with the same data content, occurring in successive MSC data groups of the same type. Exceptionally, the code "1111"
						 * shall be used to signal that the repetition continues for an undefined period. */
						uint8_t repetition_index:4;

						// Byte 2+3
						/* Extension field: this 16-bit field shall be used to carry information for CA on data group level (see ETSI TS 102 367).
						 * For other Data group types, the Extension field is reserved for future additions to the Data group header. */
						uint16_t extension_field;
					}group_header;

					struct msc_segment_field
					{
						// Byte 1+2
						/*  Segment number: this 15-bit field, coded as an unsigned binary number (in the range 0 to 32 767),
						 * shall indicate the segment number. */
						uint16_t segment_number:15;
						/* Last: this 1-bit flag shall indicate whether the segment number field is the last or
						 * whether there are more to be transmitted, as follows:
						 *   0: more segments to follow;
						 *   1: last segment. */
						uint8_t last:1;
					}segment_field;

					struct msc_user_access_field
					{
						// Byte 0
						uint8_t transport_id_flag:1; // Transport Id flag: this 1-bit flag shall indicate whether the Transport Id field is present, or not

						/*  Length indicator: this 4-bit field, coded as an unsigned binary number (in the range 0 to 15), shall
						 * indicate the length n in bytes of the Transport Id and End user address fields. */
						uint8_t length_indicator:4;

						// Byte 1+2
						uint16_t transport_id;

						// Byte 3-... length: (length_indicator-2)
						uint8_t * end_user_address_fied; // TODO: erst nur nen Pointer...
					}user_access_field; // TODO: End user address field??

					uint16_t crc;

				};

				uint8_t * payloadPtr = data + 0x19;
				uint8_t * currentPtr = payloadPtr; //data + 0x19; // TODO: payloadPtr macht einen Error...

				struct msc_data_group data_group =
				{
						.group_header =
						{
								.extension_flag   = (currentPtr[0] >> 7) & 0x01,
								.crc_flag         = (currentPtr[0] >> 6) & 0x01,
								.segment_flag     = (currentPtr[0] >> 5) & 0x01,
								.user_access_flag = (currentPtr[0] >> 4) & 0x01,
								.data_group_type  = currentPtr[0] & 0x0F,

								.continuity_index = (currentPtr[1] >> 4) & 0x0F,
								.repetition_index = (currentPtr[1] >> 0) & 0x0F
						}
				};
				currentPtr += 2;

				// Extension field?
				if(data_group.group_header.extension_flag == 1)
				{
					data_group.group_header.extension_field = currentPtr[0] + (currentPtr[1] << 8);

					// include this area
					currentPtr += 2;
				}

				// Segment field if activated
				if(data_group.group_header.segment_flag == 1)
				{
					data_group.segment_field.segment_number = currentPtr[0] & 0x7F;
					data_group.segment_field.last           = (currentPtr[0] >> 7) & 0x01;

					printf("Segment: number: %u, last: %u\n", data_group.segment_field.segment_number, data_group.segment_field.last);

					// include this area
					currentPtr += 2;
				}

				// User access field, if activated
				if(data_group.group_header.user_access_flag == 1)
				{
					uint8_t length_counter = 0;

					data_group.user_access_field.transport_id_flag = (currentPtr[0] >> 4) & 0x01;
					data_group.user_access_field.length_indicator  = (currentPtr[0] >> 0) & 0x0F;

					length_counter = data_group.user_access_field.length_indicator;

					printf("transport_id_flag %u, length_indicator: %u\n",
							data_group.user_access_field.transport_id_flag, data_group.user_access_field.length_indicator);

					currentPtr++;

					//Transport ID if activated
					if(data_group.user_access_field.transport_id_flag == 1)
					{
						data_group.user_access_field.transport_id = currentPtr[0] + (currentPtr[1] << 8);
						printf("transport_id: %u\n", data_group.user_access_field.transport_id);

						currentPtr += 2;
						//length_counter -= 2; TODO: Wird der jetzt mitgezählt oder nicht?
					}

					// End User Address field -> TODO: erst nur ein Ptr. Wo kommt das vor?

					// end User address field length, calculation acc. ETSI EN 300 401 Fig. 12
					currentPtr += length_counter;
				}

				printf("extension_flag %u, crc_flag %u, segment_flag %u, user_access_flag %u, data_group_type %u\n",
						data_group.group_header.extension_flag, data_group.group_header.crc_flag, data_group.group_header.segment_flag,
						data_group.group_header.user_access_flag, data_group.group_header.data_group_type);

				printf("continuity_index %u, repetition_index %u\n", data_group.group_header.continuity_index, data_group.group_header.repetition_index);

				// Test TODO für MOT
				struct mot_header
				{
					/* BodySize: This 28-bit field, coded as an unsigned binary number, indicates the total size of the body in bytes.
					 * If the body size signalled by this parameter does not correspond to the size of the reassembled MOT body, then the
					 * MOT body shall be discarded. */
					uint32_t bodySize:28;

					/* HeaderSize: This 13-bit field, coded as an unsigned binary number, indicates the total size of the header information in
					 * bytes including the header core size of 7 bytes. -> die 7 bytes sind dieser struct */
					uint16_t headerSize:13;

					/* ContentType: This 6-bit field indicates the main category of the body's content.
					 * The interpretation of this field shall be defined in TS 101 756, table 17. */
					uint8_t contentType:6;

					/* ContentSubType: This 9-bit field indicates the exact type of the body's content depending on the value of
					 * the field ContentType. The interpretation of this field shall be defined in TS 101 756, table 17. */
					uint16_t contentSubType:9;
				};

				struct mot_header_extension
				{
					/* PLI (Parameter Length Indicator): This 2-bit field describes the total length of the associated parameter.
					 * The following definitions apply:
					 * - 0 0 total parameter length = 1 byte, no DataField available;
					 * - 0 1 total parameter length = 2 bytes, length of DataField is 1 byte;
					 * - 1 0 total parameter length = 5 bytes; length of DataField is 4 bytes;
					 * - 1 1 total parameter length depends on the DataFieldLength indicator (the maximum parameter length is 32 770 bytes). */
					uint8_t PLI:2;

					uint8_t ParamId:6; // ParamId (Parameter Identifier): This 6-bit field identifies the parameter.
					// TODO: Hier nen Enum draus machen

					uint32_t dataField;

					/* Ext (ExtensionFlag): This 1-bit field specifies the length of the DataFieldLength Indicator and is coded as follows:
					 *  0: the total parameter length is derived from the next 7 bits;
					 *  1: the total parameter length is derived from the next 15 bits.
					 *
					 *  The ExtensionFlag is only present if the PLI field is set to "11". */
					enum mot_header_ext_length
					{
						EXT_LENGTH_7BIT  = 0,
						EXT_LENGTH_15BIT = 1
					}ext;

					/* DataFieldLength Indicator: This field specifies as an unsigned binary number the length of the parameter's DataField in bytes.
					 * The length of this field is either 7 bits or 15 bits, depending on the setting of the ExtensionFlag.
					 * The DataFieldLength Indicator is only present if the PLI field is set to "11". */
					uint16_t dataFieldLength:15;
				};

				// TEST Header einlesen
				if(data_group.group_header.data_group_type == 3)
				{
					struct mot_header header = {
							.bodySize       = (currentPtr[3] | (currentPtr[2]<<8) | (currentPtr[1]<<16) | (currentPtr[0]<<24)) >> 4,
							.headerSize     = ((currentPtr[3] & 0x0F) << 8) | (currentPtr[4] << 1) | (currentPtr[5] >> 7),
							.contentType    = (currentPtr[5] >> 1) & 0x3F,
							.contentSubType = ((currentPtr[5] & 0x01) << 8) | currentPtr[6]
					};

					printf("bodySize %u, headerSize %u, contentType %u, contentSubType %u\n",
							header.bodySize, header.headerSize, header.contentType, header.contentSubType);

					// TODO: Bodysize unten abfragen, ob die Größe erreicht ist

					// Jump the header length
					currentPtr += 7;
					header.headerSize -= 7;

					while(header.headerSize > 0) // TODO: später eine While?
					{
						struct mot_header_extension ext = {
								.PLI     = (currentPtr[0] >> 6) & 0x03,
								.ParamId = currentPtr[0] & 0x3F
						};

						currentPtr++;
						header.headerSize--;

						printf("Header Extension PLI %u, ParamId %u\n", ext.PLI, ext.ParamId);

						// TODO: den weiteren KRam
						//uint8_t * dataFieldDyn;
						switch(ext.PLI)
						{
							case 0b00:

								break;

							case 0b01:
								// .... TODO

								currentPtr++;
								header.headerSize--;
								break;

							case 0b10:
								ext.dataField = currentPtr[3] | (currentPtr[2]<<8) | (currentPtr[1]<<16) | (currentPtr[0]<<24);
								printf(" -> Data field: %lX\n", ext.dataField);

								currentPtr += 4;
								header.headerSize -=4;
								break;

							case 0b11:
								ext.ext = (currentPtr[0] >> 7) & 0x01;

								// Chose between 7 or 15 bit DataFieldLength
								if(ext.ext == EXT_LENGTH_7BIT)
								{
									ext.dataFieldLength = currentPtr[0] & 0x7F;
									currentPtr++;
									header.headerSize--;
								}
								else
								{
									ext.dataFieldLength = ((currentPtr[0] & 0x7F) << 8) | currentPtr[1];
									currentPtr += 2;
									header.headerSize -= 2;
								}

								// TODO: Datafield.. aber nicht 32bit zwansäuftig
								//dataFieldDyn = (uint8_t *) malloc(ext.dataFieldLength * sizeof(uint8_t)+1);
								//memcpy(dataFieldDyn, currentPtr, ext.dataFieldLength);
								//dataFieldDyn[ext.dataFieldLength]=0; // testweise String abschließen TODO

								//free(dataFieldDyn);

								// Test für ContentName
								// Character set indicator 4bit, rfu 4bit
								currentPtr++;
								header.headerSize--;

								mot_status.currentName = malloc(ext.dataFieldLength+1-1);
								mot_status.currentName[ext.dataFieldLength-1] = 0;
								memcpy(mot_status.currentName, currentPtr, ext.dataFieldLength-1);

								printf("Datafield: %s\n", currentPtr); // TODO: Gefährlicher Test, 0 am Ende nicht sicher...

								currentPtr += (ext.dataFieldLength-1);
								header.headerSize -= (ext.dataFieldLength-1);

								break;
						}

						// TODO: hier dann die Funktion evaluieren, z.B.  ContentName -> aber unten erst CRC!!


					}

					// Activate new file transfer on last block
					if(data_group.segment_field.last == 1)
					{
						// TODO: dateiname und so muss dann auch im Status liegen
						if(mot_status.currentName != 0)
						{
							printf("sSetLogFilename_%s\n", mot_status.currentName); // noch falsch...
							mot_status.startHeader = 1; // Restart on the next header
							//mot_status.startBlock  = 1; // Start new file on the next header TODO hier erst fragen, ob der andere fertig ist
							mot_status.currentTransportId = data_group.user_access_field.transport_id;

							free(mot_status.currentName);
						}


					}

				}
				else if(data_group.group_header.data_group_type == 4)
				{

					// TODO: Fürs erste dumpen
					if(mot_status.currentTransportId == data_group.user_access_field.transport_id)
						//TODO: Was sonst??
					{
						uint16_t logSize = byte_count-((uint32_t)(currentPtr-payloadPtr))-2;
						//uint16_t logSize = byte_count-((uint32_t)(currentPtr-data-0x17));

						// TODO auch blödsin... der schneidet irgendwo was ab irgendwie... 500 byte zu viel? ka..
						//logSize -= 2;

						printf("-> logSize %u byte\n", logSize);

						memcpy(&test, currentPtr, logSize);

						/*char writeDings[] = "writeBinLog_501\n";
						extern USBD_HandleTypeDef hUsbDeviceFS;

						USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*) &writeDings, sizeof(writeDings));
						USBD_CDC_TransmitPacket(&hUsbDeviceFS);

						USBD_CDC_SetTxBuffer(&hUsbDeviceFS, currentPtr, logSize);
						USBD_CDC_TransmitPacket(&hUsbDeviceFS);*/

						printf("writeBinLog_%u\n", logSize);
						CDC_Transmit_FS(currentPtr, logSize);
						//if(CDC_Transmit_FS(currentPtr, logSize) == USBD_BUSY) // TODO: CRC am Ende noch weg: -2
						//{
						//	printf("CDC war busy!\n");
						//}


						printf("<b>Laber!</b>\n");

						//noch eine Idee.... mit memcpy erstmal in einen anderen Buffer. Dann senden
					}
				}



				// TODO: Das letzte Feld ist CRC. Oben berechnen besser..

			}
			break;

		case PAD_DATA:
			printf("PAD_DATA ist noch nicht implementiert...\n");
			break;
		case PAD_DLS: // TODO: in eigene Funktion?
			dls_payload.toggle_bit = (data[25] >> 7) & 0x1;
			dls_payload.C_flag     = (data[25] >> 4) & 0x1;

			if(dls_payload.C_flag)
			{
				dls_payload.command = (data[25]) & 0xF;
				dls_payload.link    = (data[25] >> 4) & 0x1;

				// CId: immer 0 0 0 0: (data[27] >> 4) & 0xF;
				dls_payload.item_toggle_bit  = (data[27] >> 3) & 0x1;
				dls_payload.item_running_bit = (data[27] >> 2) & 0x1;
				dls_payload.number_tags      = (data[27] >> 0) & 0x3;

				printf("item_toggle_bit %u, item_running_bit %u, number_tags %u\n", dls_payload.item_toggle_bit, dls_payload.item_running_bit, dls_payload.number_tags);
				switch(dls_payload.command)
				{
					case CLEAR_DISPLAY_CMD:
						printf("CLEAR_DISPLAY_CMD\n");
						break;

					case DL_PLUS_CMD:
						printf("DL_PLUS_CMD\n");
						break;
				}

				uint8_t * dl_plus_index = &data[28];
				for(uint8_t i=0; i<dls_payload.number_tags; i++)
				{
					char currentMsg[128] = {0,};
					struct dl_plus_tag tag =
					{
							.content_type = *dl_plus_index & 0x7F,
							.start_marker = *(dl_plus_index+1) & 0x7F,
							.length_marker = *(dl_plus_index+2) & 0x7F,
					};

					// TODO: Abfrage, ob in der MSG überhaupt schon was empfangen wurde... vllt kam die Nachricht vorher

					memcpy(&currentMsg, &dls_payload.msg[tag.start_marker], tag.length_marker+1);
					//*(currentMsg + tag.length_marker + 1) = 0;
					printf("content type %u: %s\n", tag.content_type, currentMsg);

					dl_plus_index += 3;
				}

				// Every DL Plus tag is 24bit long

				// TODO: hier For-Schleife über Kapitel 6
			}
			else
			{
				dls_payload.charset = (data[25] >> 4) & 0xF;

				memset(&dls_payload.msg, 0, sizeof(dls_payload.msg));
				memcpy(&dls_payload.msg, &data[27], byte_count-2);

				switch(dls_payload.charset)
				{
					case EBU_Latin_based:
						printf("Codepage: EBU_Latin_based\n");
						break;

					case UCS2_transformation:
						printf("Codepage: UCS2_transformation\n");
						break;

					case UTF8_transformation:
						printf("Codepage: UTF8_transformation\n");
						break;
				}

				printf("Current service data text: %s\n", dls_payload.msg);
			}
			break;

	}



	return state;
}

// SI46XX_MSG_DIGRAD_STATUS
HAL_StatusTypeDef Si46xx_Msg_DigradStatus_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_BUSY;

	enum ACK DIGRAD_ACK; // Clears all pending digital radio interrupts.
	enum ACK STCACK;     // Clears the STC interrupt status indicator if set.

	// TODO: gibt noch mehr zu ACKen !

	DIGRAD_ACK = ACK;
	STCACK     = ACK;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
			data[0] = SI46XX_SPI_CMD_DAB_DIGRAD_STATUS;
			data[1] = 0x00 | (DIGRAD_ACK << 3) |(STCACK);

			state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_DigradStatus_receiveFunc()
{
	uint8_t data[0x27+1];
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, sizeof(data));

	uint32_t tuned_frequency;

	Si46xxCfg.events.acquisition_state_change = (data[5]>>2) & 0x01;
	// TODO FICERRINT könnte noch interessant sein

	Si46xxCfg.deviceStatus.VALID  = (data[6]>>0) & 0x01;
	Si46xxCfg.deviceStatus.ACQ    = (data[6]>>2) & 0x01;
	Si46xxCfg.deviceStatus.FICERR = (data[6]>>3) & 0x01;

	Si46xxCfg.deviceStatus.RSSI        = data[7];
	Si46xxCfg.deviceStatus.SNR         = data[8];
	Si46xxCfg.deviceStatus.FIC_QUALITY = data[9];
	// CNR data[10]
	// FIB_ERROR_COUNT data[11 und 12]

	tuned_frequency = data[13] | (data[14]<<8) | (data[15]<<16) | (data[16]<<24);

	printf("sDigrad_%u%u%u%u_%lu\n", data[6], data[7], data[8], data[9],
			tuned_frequency);

	return state;
}

// SI46XX_MSG_GET_EVENT_STATUS
HAL_StatusTypeDef Si46xx_Msg_GetEventStatus_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_BUSY;

	enum ACK EVENT_ACK = ACK;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
			data[0] = SI46XX_SPI_CMD_DAB_GET_EVENT_STATUS;
			data[1] = 0x00 | EVENT_ACK; // TODO: hier ACK einbauen

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

	// TODO: Other Ensemble (OE) Services interrupt könnte auch noch interessant sein -> AF-Funktionialität, Details in der API

	return state;
}

// SI46XX_MSG_GET_ENSEMBLE_INFO
HAL_StatusTypeDef Si46xx_Msg_GetEnsembleInfo_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
			data[0] = SI46XX_SPI_CMD_GET_ENSEMBLE_INFO;
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
		data[0] = SI46XX_SPI_CMD_SET_FREQ_LIST;
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
		data[0] = SI46XX_SPI_CMD_GET_FREQ_LIST;
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

// SI46XX_MSG_DAB_GET_COMPONENT_INFO
HAL_StatusTypeDef Si46xx_Msg_DabGetComponentInfo_sendFunc()
{
	uint8_t data[12];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		// Get current service from wanted indexes
		uint32_t serviceID   = Si46xxCfg.channelData.services[Si46xxCfg.wantedService.serviceID].serviceID;
		uint32_t componentID = Si46xxCfg.channelData.services[Si46xxCfg.wantedService.serviceID].components[Si46xxCfg.wantedService.componentID].componentID;

		data[0] = SI46XX_SPI_CMD_DAB_GET_COMPONENT_INFO;
		data[1] = 0x00;
		data[2] = 0x00;
		data[3] = 0x00;

		data[4] = (serviceID >>  0) & 0xFF;
		data[5] = (serviceID >>  8) & 0xFF;
		data[6] = (serviceID >> 16) & 0xFF;
		data[7] = (serviceID >> 24) & 0xFF;

		data[8]  = (componentID >>  0) & 0xFF;
		data[9]  = (componentID >>  8) & 0xFF;
		data[10] = (componentID >> 16) & 0xFF;
		data[11] = (componentID >> 24) & 0xFF;

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, sizeof(data));
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_DabGetComponentInfo_receiveFunc()
{
	uint8_t data[0x36+1];
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 0x35);

	// TODO: erstmal hier definieren, falls irgendwann gebraucht

	// Source: ETSI TS 101 756 V2.1.1 Table 16: User Application Types
	enum user_application_type_en
	{
		UATYPE_SlideShow   = 0x002, /* ETSI TS 101 499 */
		UATYPE_TPEG        = 0x004,
		UATYPE_SPI         = 0x007, /* ETSI TS 102 818 */
		UATYPE_DMB         = 0x009, /* ETSI TS 102 428 */
		UATYPE_Filecasting = 0x00D, /* ETSI TS 103 177 */
		UATYPE_Journaline  = 0x44A, /* ETSI TS 102 979 */

		// all others are not used or reserved
	};

	uint8_t ua_num       = data[0x1B]; /* The number of user application types. */
	uint8_t ua_total_len = data[0x1C]; /* The total length (in byte) of the UATYPE, UADATALEN and UADATA fields, including the padding bytes which is described in UADATAN field. */
	enum user_application_type_en ua_type = data[0x1D] | (data[0x1E]<<8);
	uint8_t ua_data_len = data[0x1F]; /* The UADATA field length, excluding the padding byte which is described in UADATAN field. */


	// TODO auswerten und in Variablen packen

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

		data[0] = SI46XX_SPI_CMD_GET_SERVICE_INFO;
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
	char serviceLabel[16+1];
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 0x19);

	memcpy(serviceLabel, &data[9], 16);

	printf("Si46xx GetServiceInfo Label: %s \n", serviceLabel);

	// TODO auswerten und in Variablen packen

	return state;
}

