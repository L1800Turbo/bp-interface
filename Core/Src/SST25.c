/*
 * SST25.c
 *
 * Write SST25VF016B Flash
 *
 *  Created on: Apr 14, 2022
 *      Author: kai
 */

#include "SST25.h"
#include "Si46xx.h"

// If the Si46xx firmware should be on the uC flash here
//#define SI46XX_FLASH_ON_UC

HAL_StatusTypeDef SST25_SPI_TransmitReceive(SPI_HandleTypeDef * hspi, uint8_t * dataOut, uint8_t * dataIn, uint16_t len);
HAL_StatusTypeDef SST25_SPI_Transmit(SPI_HandleTypeDef * hspi, uint8_t * dataOut, uint16_t len);

void SST25_Set_Address(uint32_t address);

HAL_StatusTypeDef SST25_Write_Disable();

HAL_StatusTypeDef SST25_Chip_Erase(uint32_t address);
HAL_StatusTypeDef SST25_Write_StatusRegister();
HAL_StatusTypeDef SST25_AAI_Program(uint8_t first, uint8_t * buffer);

uint8_t SPI_Flash_Active = 0;

struct sst25Cfg
{
	/* SPI handler */
	SPI_HandleTypeDef * hspi;

	/* Status */
	struct sst25_status
	{
		uint8_t busy:1;
		uint8_t writeEnabled:1;
		uint8_t blockProtection:4;
		uint8_t autoAddressIncrement:1;
		uint8_t blockProtectionLockdown:1;
	} status;

	/* Start address */
	uint32_t address;

}sst25Cfg;

void SST25_Init(SPI_HandleTypeDef * hspi)
{
	sst25Cfg.hspi = hspi;

	SST25_Set_Address(0x00); // Als Default erst an 0x00

	SST25_Write_StatusRegister(); // Als Default Schreiben auf Flash erlauben
}

void SST25_Set_Address(uint32_t address)
{
	sst25Cfg.address = address;
}

// Einfache workaround-Funktion, um den direkt angeschlossenen Flash zu programmieren
void SST25_Write_Flash_File()
{
#ifndef SI46XX_FLASH_ON_UC
	printf("FW Flash not loaded into uC flash!\n");

#else
	uint32_t fileSize = 0;
	uint8_t firstRun = 1;

	//extern uint8_t Si46xx_Firmware; //[499356]
	uint8_t * fwPtr = (uint8_t*) &Si46xx_Firmware;

	SST25_read_Status();
	while(sst25Cfg.status.busy != 0)
	{
		if(SST25_read_Status() != HAL_OK)
		{
			printf("SST25_Write_Flash_File: Error \n");
			return;
		}
	}

	SST25_Chip_Erase(sst25Cfg.address);

	SST25_read_Status();
	while(sst25Cfg.status.busy != 0)
	{
		if(SST25_read_Status() != HAL_OK)
		{
			printf("SST25_Write_Flash_File: Error \n");
			return;
		}
	}

	fileSize = sizeof(Si46xx_Firmware);

	printf("Writing fw to flash, address: %lX, size: %lu\n", sst25Cfg.address, fileSize);

	// TODO
	/*
	 - WREN setzen
	 - ADH / 24bit adresse / byte 0 und 1
	 - status-while
	 while:
	   - ADH, byte 2 und 3 und dann n und n+1
	 - WRDI: Exit AAI modus -> Write-disable
	 - status-Register lesen
	 */

	SST25_Write_Enable(); // Enable WREN

	while(fileSize > 0)
	{
		SST25_AAI_Program(firstRun, fwPtr);

		// Forward file pointer and decrement size
		fwPtr += 2;
		fileSize -= 2;

		SST25_read_Status();
		while(sst25Cfg.status.busy != 0)
		{
			if(SST25_read_Status() != HAL_OK)
			{
				printf("SST25_Write_Flash_File: Error \n");
				return;
			}
		}

		firstRun = 0;
	}

	SST25_Write_Disable();
	SST25_read_Status();

	printf("Writing should be done...\n");
#endif

	// TODO: SI46xx resetten


}

//TODO erstmal als Testfunktion
HAL_StatusTypeDef SST25_read_ID(uint8_t * manufacturer, uint8_t * device)
{
	HAL_StatusTypeDef state = HAL_ERROR;
	uint8_t dataOut[6] = {0,};
	uint8_t dataIn[6]  = {0,};

	dataOut[0] = SST25_MSG_READ_ID;
	dataOut[1] = 0x00;
	dataOut[2] = 0x00;
	dataOut[3] = 0x00;

	state = SST25_SPI_TransmitReceive(sst25Cfg.hspi, dataOut, dataIn, sizeof(dataOut));

	*manufacturer = dataIn[4];
	*device = dataIn[5];

	return state;
}

HAL_StatusTypeDef SST25_read_Status()
{
	HAL_StatusTypeDef state = HAL_ERROR;
	uint8_t dataOut[2] = {0,};
	uint8_t dataIn[2]  = {0,};

	dataOut[0] = SST25_MSG_READ_STATUS_REGISTER;

	state = SST25_SPI_TransmitReceive(sst25Cfg.hspi, dataOut, dataIn, sizeof(dataOut));

	if(state == HAL_OK)
	{
		memcpy((uint8_t*) &sst25Cfg.status, &dataIn[1], 1);
	}

	return state;
}

HAL_StatusTypeDef SST25_Write_StatusRegister()
{
	uint8_t dataOut[2];

	SST25_Write_Enable(); // Enable WREN

	dataOut[0] = SST25_MSG_WRITE_STATUS_REGISTER;
	dataOut[1] = 0;

	return SST25_SPI_Transmit(sst25Cfg.hspi, dataOut, 2);

}


HAL_StatusTypeDef SST25_Write_Enable()
{
	uint8_t dataOut[1];

	dataOut[0] = SST25_MSG_WRITE_ENABLE;

	return SST25_SPI_Transmit(sst25Cfg.hspi, dataOut, 1);

}

HAL_StatusTypeDef SST25_Write_Disable()
{
	uint8_t dataOut[1];

	dataOut[0] = SST25_MSG_WRITE_DISABLE;

	return SST25_SPI_Transmit(sst25Cfg.hspi, dataOut, 1);

}

HAL_StatusTypeDef SST25_Erase_4kb(uint32_t address)
{
	uint8_t data[4];

	SST25_Write_Enable(); // Enable WREN

	data[0] = SST25_MSG_4KB_SECTOR_ERASE;
	data[1] = (address >>  0) & 0xFF;
	data[2] = (address >>  8) & 0xFF;
	data[3] = (address >> 16) & 0xFF;

	return SST25_SPI_Transmit(sst25Cfg.hspi, data, sizeof(data));
}


HAL_StatusTypeDef SST25_Chip_Erase(uint32_t address)
{
	uint8_t data[1];

	SST25_Write_Enable(); // Enable WREN

	data[0] = SST25_MSG_CHIP_ERASE;

	return SST25_SPI_Transmit(sst25Cfg.hspi, data, sizeof(data));
}

HAL_StatusTypeDef SST25_AAI_Program(uint8_t first, uint8_t * buffer)
{
	uint8_t data[6];
	uint8_t size = 0;

	if(first == 1)
	{
		data[0] = SST25_MSG_AAI_PROGRAM;

		data[1] = (sst25Cfg.address >>  0) & 0xFF;
		data[2] = (sst25Cfg.address >>  8) & 0xFF;
		data[3] = (sst25Cfg.address >> 16) & 0xFF;

		data[4] = buffer[0];
		data[5] = buffer[1];

		size = 6;
	}
	else
	{
		data[0] = SST25_MSG_AAI_PROGRAM;

		data[1] = buffer[0];
		data[2] = buffer[1];

		size = 3;
	}

	return SST25_SPI_Transmit(sst25Cfg.hspi, data, size);
}



void Set_FLASH_CS_Listen()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = CS_SPI_FLASH_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(CS_SPI_FLASH_GPIO_Port, &GPIO_InitStruct);
}

void Set_FLASH_CS_Send()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = CS_SPI_FLASH_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_SPI_FLASH_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(CS_SPI_FLASH_GPIO_Port, CS_SPI_FLASH_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef SST25_SPI_TransmitReceive(SPI_HandleTypeDef * hspi, uint8_t * dataOut, uint8_t * dataIn, uint16_t len)
{
	HAL_StatusTypeDef state = HAL_ERROR;

	// TODO Abfrage, ob sendebereit: CS vom Si46xx einlesen: Aktiv?
	// CS vom Flash einlesen: Aktiv?

	// Is communication already active? -> Si46xx is sending data
	if(HAL_GPIO_ReadPin(CS_SPI_FLASH_GPIO_Port, CS_SPI_FLASH_Pin) == GPIO_PIN_RESET)
	{
		return HAL_BUSY;
	}

	// Signalize that we send with FLASH_CS
	SPI_Flash_Active = 1;
	Set_FLASH_CS_Send();

	// Flash CS als Ausgang setzen
	FLASH_CS_ON();
	state = HAL_SPI_TransmitReceive(hspi, dataOut, dataIn, len, 100);
	FLASH_CS_OFF();

	Set_FLASH_CS_Listen();
	SPI_Flash_Active = 0;


	return state;
}

HAL_StatusTypeDef SST25_SPI_Transmit(SPI_HandleTypeDef * hspi, uint8_t * dataOut, uint16_t len)
{
	HAL_StatusTypeDef state = HAL_ERROR;

	// TODO Abfrage, ob sendebereit: CS vom Si46xx einlesen: Aktiv?
	// CS vom Flash einlesen: Aktiv?

	// Is communication already active? -> Si46xx is sending data
	if(HAL_GPIO_ReadPin(CS_SPI_FLASH_GPIO_Port, CS_SPI_FLASH_Pin) == GPIO_PIN_RESET)
	{
		return HAL_BUSY;
	}

	// Signalize that we send with FLASH_CS
	SPI_Flash_Active = 1;
	Set_FLASH_CS_Send();

	// Flash CS als Ausgang setzen
	FLASH_CS_ON();
	state = HAL_SPI_Transmit(hspi, dataOut, len, 100);
	FLASH_CS_OFF();

	Set_FLASH_CS_Listen();
	SPI_Flash_Active = 0;


	return state;
}

