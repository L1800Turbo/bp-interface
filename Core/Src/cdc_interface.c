/*
 * usb_interface.c
 *
 *  Created on: Dec 31, 2021
 *      Author: kai
 */
#include "cdc_interface.h"
#include "usbd_cdc_if.h"

// Access to functions for firmware transfer
#include "Si46xx_boot.h"

/* Private variable definitions --------------------------------------- */
char cdcBuf[15];
char msgBuf[100];
size_t msgBufIndex;

cdc_interface_dt cdcInterface;

/* Private function definitions --------------------------------------- */
void cdc_Interface_AnalyzeFunction(char * messageStr);



void cdc_Interface_Init(void)
{
	msgBufIndex = 0;
	memset(msgBuf, 0, sizeof(msgBuf));
	memset(cdcBuf, 0, sizeof(cdcBuf));
}

void cdc_Interface_Tasks(void)
{
	size_t len = 10; // TODO: Vergrößern
	// Ringbuffer tasks for RX and TX ringbuffer
	cdc_Ringbuf_Tasks();

	// If we transfer a file, forward all data stream
	if(cdcInterface.fileTransfer_state == CDC_FILE_TRANSFER_ACTIVE)
	{
		size_t cdcBufferSize;
		uint32_t fwBufferSize;

		switch(Si46xx_boot_getUSB_fw_state())
		{
			case USB_FW_WAITING:
				cdcBufferSize = cdc_ringbuf_Rx_getLength(); // Size of the CDC buffer where we get data from usb
				fwBufferSize = Si46xx_boot_getFwBufSize(); // SizePtr of the buffer to be sent by SPI (typically 4kB blocks or the rest)

				// Limit buffer to block size
				if(fwBufferSize > SI46XX_BOOT_MAX_BUF_SIZE)
				{
					fwBufferSize = SI46XX_BOOT_MAX_BUF_SIZE;
				}

				// If the CDC buffer contains enough data for a block transfer
				if(cdcBufferSize >= fwBufferSize)
				{
					// Show Si46xx Boot state machine that a block is complete
					Si46xx_boot_setUSB_fw_state(USB_FW_TRANSFERRED);

					//printf("cdc-interface: Block size %d >= %ld, Block transferred\n", cdcBufferSize, fwBufferSize);
				}
				// TODO: Timeout einbauen, der sich immer zurücksetzt, wenn ein neuer Block rein kommt

				break;

			case USB_FW_TRANSFERRED:
			case USB_FW_BUSY:
				// Busy, wait until Si46xx module is done copying this part by SPI
				break;

			default: // If there is no file transfer or transfer is finished
				printf("CDC file transfer finished, back to CDC_FILE_TRANSFER_INACTIVE\n");
				cdcInterface.fileTransfer_state = CDC_FILE_TRANSFER_INACTIVE;

				break;
		}
	}
	else
	{
		cdc_ringbufRx_get((uint8_t *) cdcBuf, &len);

		for(uint8_t i=0; i<len; i++)
		{
			if(cdcBuf[i] == '\r')
			{
				msgBuf[msgBufIndex+1] = '\0'; // Terminate string
				cdc_Interface_AnalyzeFunction(msgBuf);

				msgBufIndex = 0;
			}
			else
			{
				msgBuf[msgBufIndex++] = cdcBuf[i];

			//	CDC_Transmit_FS((uint8_t*)&cdcBuf[i], 1); echo

				if(msgBufIndex >= sizeof(msgBuf))
				{
					// TODO: wenn zu viel für die Nachricht gesendet wurde...
				}
			}
		}
	}
}

void cdc_Interface_AnalyzeFunction(char * messageStr)
{
	//printf("\033[1;36mSsjsjsjace...\033[0m\r\n");

	if(strncmp(messageStr, "ver", 3) == 0)
	{
		printf("ver 0.01\n");

		// Send source configuration
		extern Si46xx_firmware_dt firmware;
		//extern const char fw_stateStr;

		// Send string with current firmware configuration
		printf("sfw_src_1_%d_2_%d\n", (uint8_t) firmware.fw_source[1], (uint8_t) firmware.fw_source[2]);
	}
	else if(messageStr[0] == 's')
	{
		extern struct Si46xx_Config Si46xxCfg;

		// Get state of Si46xx
		if(strncmp(messageStr, "sst", 3) == 0)
		{
			cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[SI46XX_MSG_REFRESH_SYS_STATE]);
			//CDC_Transmit_FS((uint8_t *) "sst ", 4);
			//CDC_Transmit_FS((uint8_t*) &Si46xxCfg.deviceStatus, sizeof(Si46xx_Status_Values_dt));
			//CDC_Transmit_FS((uint8_t*) &Si46xxCfg.image, sizeof(enum Si46xx_Image));
			//CDC_Transmit_FS((uint8_t*) "\n", 1);
		}

		// get a list of possible commands to run directly
		else if(strncmp(messageStr, "scmd_list", 9) == 0)
		{
			printf("scmd_list_");

			for(Si46xx_msg_en i=0; i < SI46XX_MSG_SIZE; i++)
			{
				printf("%d;%s#", i, Si46xx_messages[i].msgName);
			}

			printf("\n");
		}

		// Run a command directly
		else if(strncmp(messageStr, "scmd", 4) == 0)
		{
			uint8_t cmdIndex = atoi(&messageStr[4]);

			//printf("cmdIndex %d", cmdIndex);
			if(cmdIndex > 0 && cmdIndex < SI46XX_MSG_SIZE)
			{
				printf("Got command index %d\n", cmdIndex);
				cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[cmdIndex]); //SI46XX_MSG_REFRESH_SYS_STATE z.B. 1
			}
		}

		// If the boot source configuration should be changed
		else if(strncmp(messageStr, "sfw_src", 7) == 0)
		{
			if(messageStr[7] - 0x30 < FW_SRC_size && messageStr[8] - 0x30 < FW_SRC_size)
			{
				Si46xx_Boot_SetSources(messageStr[7] - 0x30, messageStr[8] - 0x30);
			}
			else
			{
				printf("Error setting boot sources!\n");
			}

			// TODO temporär, eine firmware protected get function bauen...
			extern Si46xx_firmware_dt firmware;
			printf("sfw_src_1_%d_2_%d\n", (uint8_t) firmware.fw_source[1], (uint8_t) firmware.fw_source[2]);
		}

		// Tune to a frequency, refresh the contents of it: stune_n
		else if(strncmp(messageStr, "stune", 5) == 0)
		{
			enum DAB_frequencies channelIndex = atoi(&messageStr[6]);

			// Check if channel param was valid
			if(channelIndex >= 0 && channelIndex < DAB_Chan_SIZE)
			{
				Si46xxCfg.freqIndex = channelIndex;

				printf("Si46xx: Tuning to channel index %d: %s\n", channelIndex, DAB_frequency_list[channelIndex].name);

				cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[SI46XX_MSG_DAB_TUNE_FREQ]);
				cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[SI46XX_MSG_GET_DIGITAL_SERVICE_LIST]);
			}

			// TODO: Frequenz einstellen -> Tunen auf Frequenz -> Infos holen
			// Channel zurückgeben, wenn getunt
			// Wenn tunen nicht geht, abbrechen (msg-Stack leeren?)
			// Irgendwo ein Flag zur Frequenz setzen, dass die nicht tunebar ist...
			// Bei Tunen auf Frequenz würde dann ein Timeout benötigt, der sich darum kümmert

		}

		else if(strncmp(messageStr, "sboot", 5) == 0)
		{
			// TODO: Temporäte Lösung
			extern void Si46xx_Boot(void);
			Si46xx_Boot();
		}

		// Transfer a (firmware/patch) file onto the device
		else if(strncmp(messageStr, "sfile", 5) == 0)
		{
			// file size until end of string
			uint32_t fileSize = atoi(messageStr+5);

			// check if waiting and file size could be transferred
			if(fileSize > 0 && (Si46xx_boot_getUSB_fw_state() == USB_FW_WAITING ) )
			{
				if(Si46xx_boot_setFwBufSize(fileSize) == 0)
				{
					// Expect file transfers for CDC module
					cdcInterface.fileTransfer_state = CDC_FILE_TRANSFER_ACTIVE;

					// Delete any other possible received data, next part should be the file contents
					cdc_ringbufRx_clear();

					printf("sfile_ok\n");
				}
				else
				{
					printf("cdc_interface: fileSize for USB transfer couldn't be set!\n");
				}
			}
			else // no valid file size provided or Si46xx module isn't waiting for USB data
			{
				printf("sfile_err\n");
			}
		}
	}

}



