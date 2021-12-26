/*
 * Si46xx_boot.h
 *
 *  Created on: 12.12.2021
 *      Author: kai
 */

#ifndef INC_SI46XX_BOOT_H_
#define INC_SI46XX_BOOT_H_

#include "Si46xx.h"

typedef enum
{
	Si46xx_INIT_STATE_OFF = 0,
	Si46xx_INIT_STATE_RELEASE_RESET,
	Si46xx_INIT_STATE_POWER_UP_SEND,
	Si46xx_INIT_STATE_POWER_UP_WAIT,
	Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_SEND,
	Si46xx_INIT_STATE_PREPARE_LOAD_FIRMWARE_WAIT,
	Si46xx_INIT_STATE_HOST_LOAD_SEND,
	Si46xx_INIT_STATE_HOST_LOAD_WAIT,

	//Si46xx_INIT_STATE_NVM_LOAD_FIRMWARE,
	Si46xx_INIT_STATE_BOOT_SEND,
	Si46xx_INIT_STATE_BOOT_WAIT,
	Si46xx_INIT_STATE_IDLE
}Si46xx_BootStates_en;

typedef struct
{
	enum step_en {
		FW_NONE = 0,
		FW_BOOTLOADER_PATCH,
		FW_FIRMWARE
	}step;

	uint8_t * fwBufPtr;
	uint32_t fwBufSize;
}Si46xx_firmware_dt;

void Si46xx_Boot_Tasks(void);

#endif /* INC_SI46XX_BOOT_H_ */
