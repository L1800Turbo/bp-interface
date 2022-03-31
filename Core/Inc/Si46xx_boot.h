/*
 * Si46xx_boot.h
 *
 *  Created on: 12.12.2021
 *      Author: kai
 */

#ifndef INC_SI46XX_BOOT_H_
#define INC_SI46XX_BOOT_H_

#include "Si46xx.h"

//#define SI46XX_BOOT_MAX_BUF_SIZE 4092

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





Si46xx_state_en Si46xx_Boot_Tasks(void);

//uint8_t Si46xx_Boot_SetSources(fw_source_dt bootloader_patch, fw_source_dt firmware);

//uint8_t Si46xx_boot_setUSB_size(uint32_t size);


#endif /* INC_SI46XX_BOOT_H_ */
