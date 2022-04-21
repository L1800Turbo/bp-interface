/*
 * Si46xx_firmware_transfer.h
 *
 *  Created on: 10.03.2022
 *      Author: kai
 */

#ifndef INC_SI46XX_FIRMWARE_TRANSFER_H_
#define INC_SI46XX_FIRMWARE_TRANSFER_H_

uint8_t Si46xx_firmware_isBusy(void);
Si46xx_statusType Si46xx_send_firmware(enum fw_source fw_source, uint8_t * fwBufPtr, uint32_t fwBufSize);

void Si46xx_firmware_tasks(void);

usb_fw_en Si46xx_boot_getUSB_fw_state(void);
uint8_t Si46xx_boot_setUSB_fw_state(usb_fw_en usbFw);
//uint8_t * Si46xx_boot_getUSB_ptr(void);
size_t Si46xx_boot_getFwBufSize(void);
uint8_t Si46xx_boot_setFwBufSize(uint32_t size);

#endif /* INC_SI46XX_FIRMWARE_TRANSFER_H_ */
