/*
 * Si46xx_boot.h
 *
 *  Created on: 12.12.2021
 *      Author: kai
 */

#ifndef INC_SI46XX_BOOT_H_
#define INC_SI46XX_BOOT_H_

#include "Si46xx.h"

#define SI46XX_BOOT_MAX_BUF_SIZE 4092

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

typedef enum
{
	USB_FW_NONE = 0,
	USB_FW_WAITING,		// If we wait for the first CDC response
	USB_FW_TRANSFERRED, // Show if CDC has a block transferred
	USB_FW_BUSY // in case the buffer is being used and shouldn't be overwritten by CDC
}usb_fw_en;

typedef struct
{
	enum step_en {
		FW_NONE = 0,
		FW_BOOTLOADER_PATCH,
		FW_FIRMWARE,

		FW_size
	}step;

	// Configuration where the files are located
	fw_source_dt fw_source[FW_size];

	usb_fw_en usbFw_wanted;

	uint8_t * fwBufPtr;
	uint32_t fwBufSize;
	//size_t fwUsbBufSize; // the USB package size to check if a whole block is received by USB CDC
}Si46xx_firmware_dt;

Si46xx_state_en Si46xx_Boot_Tasks(void);

uint8_t Si46xx_Boot_SetSources(fw_source_dt bootloader_patch, fw_source_dt firmware);

usb_fw_en Si46xx_boot_getUSB_fw_state(void);
uint8_t Si46xx_boot_setUSB_fw_state(usb_fw_en usbFw);
//uint8_t * Si46xx_boot_getUSB_ptr(void);
size_t Si46xx_boot_getFwBufSize(void);
uint8_t Si46xx_boot_setFwBufSize(uint32_t size);
//uint8_t Si46xx_boot_setUSB_size(uint32_t size);

#endif /* INC_SI46XX_BOOT_H_ */
