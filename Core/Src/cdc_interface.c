/*
 * usb_interface.c
 *
 *  Created on: Dec 31, 2021
 *      Author: kai
 */
#include "cdc_interface.h"
#include "usbd_cdc_if.h"

// Access to functions for firmware transfer
#include "Si46xx_firmware_transfer.h"

#include "SST25.h"

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
	//cdc_Ringbuf_Tasks(); // TODO: war mal in SysTick_Handler(void), jetzt in main()

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
				// don't add new data as long the current data is transferred and not being copied
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
				msgBuf[msgBufIndex/*+1*/] = '\0'; // Terminate string
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
		extern struct Si46xx_Config Si46xxCfg;
		printf("ver 0.01\n");

		// Send string with current firmware configuration
		printf("sfw_src_%d_%lu\n", (uint8_t) Si46xxCfg.firmware_source, Si46xxCfg.firmware_flash_address);
	}
	else if(strncmp(messageStr, "flash", 5) == 0)
	{
		if(strncmp(messageStr, "flash_ident", 11) == 0)
		{
			uint8_t manufacturer;
			uint8_t device;
			SST25_read_ID(&manufacturer, &device);

			SST25_read_Status();

			printf("flash_ident_%X_%X\n", manufacturer, device);
		}
		else if(strncmp(messageStr, "flash_add", 9) == 0)
		{
			uint32_t address = atoi(messageStr+10);

			SST25_Set_Address(address);
		}
		else if(strncmp(messageStr, "flash_write", 11) == 0) // TODO: einfach aus µC-Flash schreiben
		{
			SST25_Write_Flash_File();
		}

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
			uint8_t cmdIndex = atoi(&messageStr[4]); // TODO: Das hier passt noch cniht mit dem Index!

			//printf("cmdIndex %d", cmdIndex);
			if(cmdIndex > 0 && cmdIndex < SI46XX_MSG_SIZE)
			{
				//printf("Got command index %d\n", cmdIndex);
				cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[cmdIndex]); //SI46XX_MSG_REFRESH_SYS_STATE z.B. 1
			}
		}

		// If the boot source configuration should be changed
		else if(strncmp(messageStr, "sfw_src", 7) == 0)
		{
			extern struct Si46xx_Config Si46xxCfg;

			uint8_t srcIndex = atoi(messageStr+7);
			if(srcIndex < FW_SRC_size)
			{
				Si46xxCfg.firmware_source = srcIndex;
			}
			else
			{
				printf("Error setting boot sources!\n");
			}

			printf("sfw_src_%d_%lu\n", (uint8_t) Si46xxCfg.firmware_source, Si46xxCfg.firmware_flash_address);
		}

		// Change the default flash address for the firmware to boot the device from
		else if(strncmp(messageStr, "sfw_flash_add", 13) == 0)
		{
			extern struct Si46xx_Config Si46xxCfg;
			uint32_t address = atoi(messageStr+13);

			Si46xxCfg.firmware_flash_address = address;
		}

		// Set the flash address to write data to
		else if(strncmp(messageStr, "sfw_flash_write_add", 19) == 0)
		{
			extern struct Si46xx_Config Si46xxCfg;
			uint32_t address = atoi(messageStr+19);

			Si46xxCfg.firmware.current_fw_destination = FW_DST_FLASH;
			Si46xxCfg.firmware.current_flash_address  = address;
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
				//cb_push_back(&Si46xxCfg.cb, &Si46xx_messages[SI46XX_MSG_GET_DIGITAL_SERVICE_LIST]); TODO: Macht hier keinen Sinn, ist ja noch unvollst.
			}

			// TODO: Frequenz einstellen -> Tunen auf Frequenz -> Infos holen
			// Channel zurückgeben, wenn getunt
			// Wenn tunen nicht geht, abbrechen (msg-Stack leeren?)
			// Irgendwo ein Flag zur Frequenz setzen, dass die nicht tunebar ist...
			// Bei Tunen auf Frequenz würde dann ein Timeout benötigt, der sich darum kümmert

		}

		// Selecting a service and component
		else if(strncmp(messageStr, "sStartSrv", 9) == 0)
		{
			char * strPtr;
			char delimiter[] = "_";
			uint8_t serviceID_index;
			uint8_t componentID_index;


			printf("Si46xx: sStartSrv called...\n");

			// get indexes from string
			strPtr = strtok(messageStr, delimiter); // first part with command
			strPtr = strtok(NULL, delimiter);		// get the first number

			// Get service ID index
			if(strPtr != NULL)
			{
				serviceID_index = atoi(strPtr); // TODO: wenn es nicht passt, gibt er 0 aus, aber das könnte ja auch ein richtiger Wert sein...

				// Get component ID index
				strPtr = strtok(NULL, delimiter);

				if(strPtr != NULL)
				{
					componentID_index = atoi(strPtr);

					Si46xxCfg.wantedService.serviceID   = serviceID_index;
					Si46xxCfg.wantedService.componentID = componentID_index;

					printf("Si46xx: Selected serviceID %lu, componentID: %u \n",
							Si46xxCfg.channelData.services[serviceID_index].serviceID,
							Si46xxCfg.channelData.services[serviceID_index].components[componentID_index].componentID);
				}
			}
		}

		else if(strncmp(messageStr, "sreset", 6) == 0)
		{
			Si46xx_Reset();
		}

		else if(strncmp(messageStr, "sboot", 5) == 0)
		{
			// TODO: Temporäte Lösung
			//extern void Si46xx_Boot(void);
			//Si46xx_Boot();

			/*if(Si46xx_send_firmware(FW_SRC_UC, (uint8_t *) &Si46xx_Rom00Patch016, sizeof(Si46xx_Rom00Patch016)) == Si46xx_OK)
			{
				printf("send_firmware anstoßen ging\n");
			}
			else
			{
				printf("send_firmware anstoßen ging nicht\n");
			}*/

			//extern uint8_t test[1000];// = {0, };

			//for(uint16_t i=0; i<510; i++)
			//{
			//	test[i] = i % 0x100;
			//}
			//test[300] = '\n';
			//uint16_t laenge = 200;

			//printf("writeBinLog_%d\n", laenge);
			//CDC_Transmit_FS(test, laenge);

			printf("Laber: öäüß é æ\n");

			char payload[] = {0x40, 0x93, 0x91, 0};
			uint8_t byte_count = sizeof(payload);

			char bloedSabbel[128];

			char * msgPtr = bloedSabbel;

			struct EBU_Latin_table
			{
				char ebu;
				char utf8[4];
			};

#define CHAR_TABLE_SIZE 2
			struct EBU_Latin_table charTable[CHAR_TABLE_SIZE] =
			{
				{.ebu = 0x91, .utf8 = "ä"},
				{.ebu = 0x97, .utf8 = "ö"}
			};

			int8_t found = -1;
			for(uint8_t i=0; i<byte_count-2+2; i++)
			//while(*msgPtr != 0)
			{
				for(uint8_t j=0; j<sizeof(charTable); j++)
				{
					if(payload[i] == charTable[j].ebu)
					{
						memcpy(msgPtr, charTable[j].utf8, strlen(charTable[j].utf8));
						msgPtr += strlen(charTable[j].utf8);

						found = 1;
						break;
					}
				}

				if(found == 1)
				{
					continue;
				}
				else
				{
					*msgPtr = payload[i];
					msgPtr++;
				}

			}

			printf("Sabbel: %s\n", bloedSabbel);

		}

		else if(strncmp(messageStr, "sload_fw", 8) == 0) // TODO: noch umbenennen, wenn das auch für Flash-FW-Transfer genutzt wird
		{
			// TODO: Temporäte Lösung
			//extern void Si46xx_Boot(void);
			//Si46xx_Boot();

			// TODO: Temporäte Lösung II
			//if(Si46xx_send_firmware(FW_SRC_UC, (uint8_t *) &Si46xx_Firmware, sizeof(Si46xx_Firmware)) == Si46xx_OK)
			/*if(Si46xx_send_firmware(FW_SRC_USB, 0, 0) == Si46xx_OK)
			{
				printf("send_firmware sboot_fw anstoßen ging\n");
			}
			else
			{
				printf("send_firmware sboot_fw anstoßen ging nicht\n");
			}*/

			// TODO: Temporäte Lösung III
			Si46xxCfg.radio_states = Si46xx_Radio_Start;
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

					printf("sfile_ok\n"); // Triggers in ScriptCommunicator to start sending
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



