/*
 * Si46xx_SPI_functions.h
 *
 *  Created on: Dec 31, 2021
 *      Author: kai
 */

#ifndef INC_SI46XX_SPI_FUNCTIONS_H_
#define INC_SI46XX_SPI_FUNCTIONS_H_

//#include "Si46xx.h" // TODO: gegenseitig included...
#include "Si46xx_data_types.h"
#include "stm32f4xx_hal.h"

#include "main.h"


#define SI46XX_DEFAULT_SPI_WAIT 10 // ms, when looping and polling for an SPI ready state from uC or Si46xx

#define SI46XX_RST_ON()  HAL_GPIO_WritePin(Si46xx_RSTB_GPIO_Port, Si46xx_RSTB_Pin, GPIO_PIN_RESET); // Reset LOW
#define SI46XX_RST_OFF() HAL_GPIO_WritePin(Si46xx_RSTB_GPIO_Port, Si46xx_RSTB_Pin, GPIO_PIN_SET);

#define SI46XX_CS_ON()  HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_RESET)
#define SI46XX_CS_OFF() HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_SET)

// Enum for possible SPI commands
enum Si46xx_SPI_commands {
	/* Boot commands */
	SI46XX_RD_REPLY      			= 0x00,	/* Returns the status byte and data for the last command sent to the device. 			*/
	SI46XX_POWER_UP      			= 0x01,	/* Power-up the device and set system settings. 										*/
	SI46XX_HOST_LOAD     			= 0x04,	/* Loads an image from the HOST over the command interface 								*/
	SI46XX_FLASH_LOAD    			= 0x05, /* Loads an image from external FLASH over secondary SPI bus 							*/
	SI46XX_LOAD_INIT     			= 0x06,	/* Prepares the bootloader to receive a new image. 										*/
	SI46XX_BOOT         			= 0x07,	/* Boots the image currently loaded in RAM. 											*/
	SI46XX_GET_PART_INFO			= 0x08,	/* Reports basic information about the device. 											*/
	SI46XX_GET_SYS_STATE			= 0x09,	/* Reports system state information. 													*/
	SI46XX_GET_POWER_UP_ARGS		= 0x0A,	/* Reports basic information about the device such as arguments used during POWER_UP. 	*/

	SI46XX_SET_PROPERTY  			= 0x13,	/* Sets the value of a property.														*/

	SI46XX_GET_DIGITAL_SERVICE_LIST	= 0x80,	/* Gets a service list of the ensemble.	    											*/
	SI46XX_START_DIGITAL_SERVICE	= 0x81, /* Starts an audio or data service. 													*/
	SI46XX_STOP_DIGITAL_SERVICE		= 0x82, /* Stops an audio or data service.														*/
	SI46XX_GET_DIGITAL_SERVICE_DATA = 0x84,	/* Gets a block of data associated with one of the enabled data components of a digital services. */

	SI46XX_DAB_TUNE_FREQ			= 0xB0, /*  Tunes the DAB Receiver to a frequency between 168 MHz and 240 MHz					*/
	SI46XX_SET_FREQ_LIST			= 0xB8, /*  Sets the DAB frequency table. The frequencies are in units of kHz. */
	SI46XX_GET_FREQ_LIST 			= 0xB9	/* Gets the DAB frequency table  														*/
};

typedef enum
{
	SI46XX_MSG_NONE = 0,
	SI46XX_MSG_REFRESH_SYS_STATE,
	SI46XX_MSG_GET_DIGITAL_SERVICE_LIST,
	SI46XX_MSG_DAB_TUNE_FREQ,
	SI46XX_MSG_START_DIGITAL_SERVICE,
	SI46XX_MSG_STOP_DIGITAL_SERVICE,
	SI46XX_MSG_GET_DIGITAL_SERVICE_DATA,
	SI46xx_MSG_SET_FREQ_LIST,
	SI46XX_MSG_GET_FREQ_LIST,

	SI46XX_MSG_SIZE
}Si46xx_msg_en;

// Enum for SPI status of current function
typedef enum
{
	Si46xx_OK = 0,
	Si46xx_BUSY,
	Si46xx_SPI_ERROR,
	Si46xx_MESSAGE_ERROR,
	Si46xx_DEVICE_ERROR
}Si46xx_statusType;

typedef struct
{
	Si46xx_msg_en msgIndex; // Mostly for Debug
	char msgName[40];	// Name of the function for debug and external use
	HAL_StatusTypeDef (*sendFunc)();
	Si46xx_statusType (*receiveFunc)();

}Si46xx_msg_dt;

const Si46xx_msg_dt Si46xx_messages[SI46XX_MSG_SIZE];

#endif /* INC_SI46XX_SPI_FUNCTIONS_H_ */
