/*
 * SST25.h
 *
 *  Created on: Apr 14, 2022
 *      Author: kai
 */

#ifndef INC_SST25_H_
#define INC_SST25_H_

#include "main.h"
#include "stm32f4xx_hal.h"

#define FLASH_CS_ON()  HAL_GPIO_WritePin(CS_SPI_FLASH_GPIO_Port, CS_SPI_FLASH_Pin, GPIO_PIN_RESET)
#define FLASH_CS_OFF() HAL_GPIO_WritePin(CS_SPI_FLASH_GPIO_Port, CS_SPI_FLASH_Pin, GPIO_PIN_SET)

enum SST25_commands
{
	SST25_MSG_WRITE_STATUS_REGISTER = 0x01,
	SST25_MSG_READ_STATUS_REGISTER  = 0x05,
	SST25_MSG_WRITE_DISABLE         = 0x04,
	SST25_MSG_WRITE_ENABLE          = 0x06,
	SST25_MSG_4KB_SECTOR_ERASE      = 0x20,
	SST25_MSG_CHIP_ERASE            = 0x60,
	SST25_MSG_READ_ID               = 0x90,
	SST25_MSG_AAI_PROGRAM			= 0xAD
};

void SST25_Init(SPI_HandleTypeDef * hspi);
void SST25_Set_Address(uint32_t address);
void SST25_Write_File(uint32_t fileSize);

HAL_StatusTypeDef SST25_read_ID(uint8_t * manufacturer, uint8_t * device);
HAL_StatusTypeDef SST25_Write_Enable();
HAL_StatusTypeDef SST25_read_Status();

void SST25_Write_Flash_File();

#endif /* INC_SST25_H_ */
