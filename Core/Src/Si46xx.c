/*
 * Si46xx.c
 *
 * Generic functions and state machine to work with Si46xx
 *
 *  Created on: 21.03.2021
 *      Author: kai

 */
#include "Si46xx.h"
#include "Si46xx_firmware_transfer.h"


struct Si46xx_Config Si46xxCfg;

Si46xx_msg_dt currentWorkingMsg;

/* Private functions ------------------------------------------- */
extern void Si46xx_Boot(void);


/* Private variables ------------------------------------------- */
enum Si46xx_reset
{
	Si46xx_RST_SET = 0,
	Si46xx_RST_RELEASED,
};
enum Si46xx_reset resetState;

/* Helper functions -------------------------------------------- */
void set_Si46xx_ISR(void)
{
	if(Si46xxCfg.isrState == ISR_INACTIVE) // First rising edge after bootloader patch, TODO Sagt aber noch nichts richtig aus
	{
		//Si46xxCfg.isrState = ISR_FIRST_FLAG;
	}
	else
	{
		Si46xxCfg.isrState = ISR_SET;

		// If there's nothing to do, check the reason of the int:
		if(!Si46xx_isBusy() && Si46xxCfg.radio_states == Si46xx_Radio_Idle)
		{
			Si46xxCfg.isrState = ISR_UNSET; // TODO: ist das an dieser Stelle sonderlich schlau? Vll ein eigenes Event machen?
			Si46xx_Push(SI46XX_MSG_REFRESH_SYS_STATE);
		}
	}

	// TODO: hier auf Interrupts reagieren, wenn nicht in Busy-State, dann ein DAB_GET_EVENT_STATUS
	// TODO: Dafür muss aber erst noch die Konfiguration eingebaut werden, wo er einen INT erzeugt

}


// TODO: Properties wo anders noch einbauen?
#define BASE_PROPERTY_NO 4
struct property baseProperties[BASE_PROPERTY_NO] =
{
		{.index = DIGITAL_SERVICE_INT_SOURCE, .data = 0x0001},
		{.index = INT_CTL_ENABLE,             .data = (INT_CTL_DEVNTIEN | INT_CTL_CTSIEN | INT_CTL_DACQIEN | INT_CTL_DSRVIEN | INT_CTL_STCIEN)},
		{.index = DAB_EVENT_INTERRUPT_SOURCE, .data = SRVLIST_INTEN },
		{.index = DAB_XPAD_ENABLE,            .data = (/*MOT_SLS_ENABLE |*/ DLS_ENABLE)}
};
uint8_t currentProperty = 0;

//extern Si46xx_state_en Si46xx_Boot_Tasks(void); // TODO temporär

void Si46xx_Reset_Status(void)
{
	Si46xxCfg.analyzedStatus = Si46xx_OK;
}

void Si46xx_Reset(void)
{
	// Clear message buffer
	cb_free(&Si46xxCfg.cb);

	// Reset analyzed status
	Si46xx_Reset_Status();

	resetState = Si46xx_RST_SET;
	Si46xxCfg.function_state = Si46xx_STATE_RESET;
}

uint8_t Si46xx_isBusy(void) // TODO: Als Busy-Abfrage einbauen !!!! DAS GEHT SO NICHT, der hat 0 und ist in IDLE, wenn er sendet
{
	return Si46xxCfg.cb.count || Si46xxCfg.function_state != Si46xx_STATE_IDLE;
}

/* State machine to control the radio functions, boot, ... */
// TODO: Timeouts einbauen
void Si46xx_radio_tasks(void)
{
	switch(Si46xxCfg.radio_states)
	{
		case Si46xx_Radio_Off:
			break;

		case Si46xx_Radio_Start:
			// Reset device
			Si46xx_Reset();

			// Send power up
			Si46xx_Push(SI46XX_MSG_POWER_UP);

			Si46xxCfg.radio_states++;
			break;

		case Si46xx_Radio_Start_Wait:
			if(!Si46xx_isBusy()) // TODO: genau beobachten, die Funktion passt noch nicht ganz
			{
				if(Si46xxCfg.analyzedStatus == Si46xx_OK)
				{
					printf("Si46xx_Radio_Start_Wait: Si46xx_OK\n");
					Si46xxCfg.radio_states++;
				}
			}
			break;

		case Si46xx_Radio_Patch:
			Si46xx_Push(SI46XX_MSG_LOAD_INIT); // TODO: würde er dann mehrmals aufrufen, wenn das unten nicht OK ist
			if(Si46xx_send_firmware(FW_SRC_UC, (uint8_t *) &Si46xx_Rom00Patch016, sizeof(Si46xx_Rom00Patch016)) == Si46xx_OK)
			{
				Si46xxCfg.radio_states++;
			}
			else
			{
				// TODO: Warten?
			}
			break;

		case Si46xx_Radio_Patch_Wait:
			if(!Si46xx_isBusy() && !Si46xx_firmware_isBusy()) // TODO: genau beobachten, die Funktion passt noch nicht ganz
			{
				if(Si46xxCfg.analyzedStatus == Si46xx_OK)
				{
					printf("Si46xx_Radio_Patch_Wait: Si46xx_OK\n");
					Si46xxCfg.radio_states++;
				}
			}
			break;

		case Si46xx_Radio_FlashFirmware:
			Si46xx_Push(SI46XX_MSG_LOAD_INIT);

			Si46xxCfg.firmware_flash_address = 0x0000;
			Si46xx_Push(SI46XX_MSG_FLASH_LOAD_IMG);

			Si46xxCfg.radio_states++;
			break;

		case Si46xx_Radio_FlashFirmware_Wait:
			if(!Si46xx_isBusy()) // TODO: genau beobachten, die Funktion passt noch nicht ganz
			{
				if(Si46xxCfg.analyzedStatus == Si46xx_OK)
				{
					printf("Si46xx_Radio_FlashFirmware_Wait: Si46xx_OK\n");
					Si46xxCfg.radio_states++;
				}
			}
			break;

		case Si46xx_Radio_Boot:
			Si46xx_Push(SI46XX_MSG_BOOT);
			Si46xx_Push(SI46XX_MSG_REFRESH_SYS_STATE);
			Si46xxCfg.radio_states++;
			break;

		case Si46xx_Radio_Boot_Wait:
			if(!Si46xx_isBusy()) // TODO: genau beobachten, die Funktion passt noch nicht ganz
			{
				if(Si46xxCfg.analyzedStatus == Si46xx_OK && Si46xxCfg.image == Si46xx_DAB)
				{
					printf("Si46xx_Radio_Boot_Wait: Si46xx_OK, DAB loaded");

					// Activate interrupt reactions to answers
					Si46xxCfg.isrState = ISR_UNSET;

					Si46xxCfg.radio_states++;
				}
				else
				{
					printf("Boot not successful!\n");
					Si46xxCfg.radio_states = Si46xx_Radio_Idle;
				}
			}
			break;

		case Si46xx_Radio_Properties:
			if(BASE_PROPERTY_NO > 0)
			{
				Si46xxCfg.currentProperty.index = baseProperties[currentProperty].index;
				Si46xxCfg.currentProperty.data  = baseProperties[currentProperty].data;
				Si46xx_Push(SI46XX_MSG_SET_PROPERTY);

				Si46xxCfg.radio_states++;
			}
			else
			{
				Si46xxCfg.radio_states = Si46xx_Radio_Config;
			}
			break;

		case Si46xx_Radio_Properties_Wait:
			if(!Si46xx_isBusy()) // TODO: genau beobachten, die Funktion passt noch nicht ganz
			{
				if(Si46xxCfg.analyzedStatus == Si46xx_OK)
				{
					printf("Si46xx_Radio_Properties_Wait: Si46xx_OK");

					if(++currentProperty >= BASE_PROPERTY_NO)
					{
						Si46xxCfg.radio_states++;
					}
					else
					{
						Si46xxCfg.radio_states = Si46xx_Radio_Properties;
					}
				}
			}
			break;

		case Si46xx_Radio_Config:

			// TODO: Hier würde dann das Adjust Properties aus dem Programming Guide kommen

			// Set frequency list from DAB_frequency_dt DAB_frequency_list[DAB_Chan_SIZE];
			Si46xx_Push(SI46XX_MSG_SET_FREQ_LIST);
			Si46xx_Push(SI46XX_MSG_GET_FREQ_LIST); // For debugging -> will update client interface

			// Get information about HW and SW version
			Si46xx_Push(SI46XX_MSG_GET_PART_INFO);
			Si46xx_Push(SI46XX_MSG_GET_FUNC_INFO);

			Si46xxCfg.radio_states++;
			break;

		case Si46xx_Radio_Config_Wait:
			if(!Si46xx_isBusy()) // TODO: genau beobachten, die Funktion passt noch nicht ganz
			{
				if(Si46xxCfg.analyzedStatus == Si46xx_OK)
				{
					printf("Si46xx_Radio_Config_Wait: Si46xx_OK\n");

					Si46xxCfg.deviceStatus.STC = Si46xx_STCINT_COMPLETE; // As first one

					Si46xxCfg.radio_states = Si46xx_Radio_Idle;
				}
			}
			break;

		case Si46xx_Radio_Idle:

			// Fetch events:
			if(
					(Si46xxCfg.events.digital_radio_link_change == Si46xx_INTERRUPT) ||
					(Si46xxCfg.events.seek_tune_complete == Si46xx_INTERRUPT)
			  )
			{
				printf("Event: digital_radio_link_change or seek_tune_complete or digital_radio_event_change\n");
				Si46xxCfg.events.digital_radio_link_change = Si46xx_NORMAL;
				Si46xxCfg.events.seek_tune_complete = Si46xx_NORMAL;
				Si46xx_Push(SI46XX_MSG_DIGRAD_STATUS);
			}

			if(
					(Si46xxCfg.events.acquisition_state_change == Si46xx_INTERRUPT) ||
					(Si46xxCfg.events.digital_radio_event_change == Si46xx_INTERRUPT)
			  )
			{
				Si46xxCfg.events.acquisition_state_change   = Si46xx_NORMAL;
				Si46xxCfg.events.digital_radio_event_change = Si46xx_NORMAL;

				printf("Event: acquisition_state_change \n");

				if(Si46xxCfg.deviceStatus.VALID == IS_VALID) // TODO: auhc FIC Quality und SIG Level prüfen
				{
					//if(Si46xxCfg.deviceStatus.FIC_QUALITY >= xy) TODO: laut Manual soll man FIC Quality und Sig Level prüfen
					Si46xx_Push(SI46XX_MSG_GET_EVENT_STATUS);
				}
				else // TOOO Hier kommt er nicht hin, wenn er den Sender gar nicht erst rein bekommt
				{
					// TODO: Hätte er hier dann den Sender verloren?
					printf("Tuned frequency not valid!\n");
				}
			}

			if(Si46xxCfg.events.service_list_int == Si46xx_INTERRUPT)
			{
				printf("Event: service_list_int\n");
				Si46xxCfg.events.service_list_int = Si46xx_NORMAL;
				Si46xx_Push(SI46XX_MSG_GET_DIGITAL_SERVICE_LIST);
				Si46xx_Push(SI46XX_MSG_GET_ENSEMBLE_INFO);
			}

			if(Si46xxCfg.events.digital_service_data == Si46xx_INTERRUPT)
			{
				printf("Event: digital_service_data\n");
				Si46xxCfg.events.digital_service_data = Si46xx_NORMAL;

				Si46xx_Push(SI46XX_MSG_GET_DIGITAL_SERVICE_DATA);
			}

			// If there is no acquisition and we want to react to it
			if(Si46xxCfg.events.no_acquisition == Si46xx_INTERRUPT)
			{
				printf("Event: no_acquisition\n");
				Si46xxCfg.events.no_acquisition = Si46xx_NORMAL;

				printf("sEnsemble_invalid\n");

				// Go to next ensemble if this one's not available
				if(Si46xxCfg.cfg.forwardIfInvalid)
				{
					printf("Moving to the next channel\n");
					Si46xxCfg.wantedService.freqIndex = (Si46xxCfg.wantedService.freqIndex+1) % DAB_Chan_SIZE;

					// Reset these parameters as they won't be valid on the next one
					Si46xxCfg.wantedService.serviceID_index   = 0;
					Si46xxCfg.wantedService.componentID_index = 0;

					Si46xx_Push(SI46XX_MSG_DAB_TUNE_FREQ);
				}
			}

			// Fetch radio commands
			if(Si46xxCfg.cmd.tuneEnsemble)
			{
				Si46xx_Push(SI46XX_MSG_DAB_TUNE_FREQ);
				Si46xxCfg.cmd.tuneEnsemble = 0;

				Si46xxCfg.deviceStatus.channelList_valid = NOT_VALID;
			}

			// If we got a valid channel list and want to start a service
			if(Si46xxCfg.deviceStatus.channelList_valid == IS_VALID && Si46xxCfg.cmd.startService)
			{
				printf("Command: directly starting service\n");
				// TODO: Man sollte besser die richtigen IDs statt nur der Positionen im Array speichern, oder?
				Si46xx_Push(SI46XX_MSG_START_DIGITAL_SERVICE);

				// Don't start looking if we lose this ensemble TODO: Konfigurierbar?
				Si46xxCfg.cfg.forwardIfInvalid = 0;
				Si46xxCfg.cmd.startService = 0;
			}


			break;
	}

	// TODO: Bootup-Flag? Also das andere Befehle nicht genommen werden, wenn das DAB gar nicht gebootet ist??
}

void Si46xx_DAB_play(enum DAB_play_functions func)
{
	printf("Si46xx_DAB_play: got command #enum_DAB_play_functions#%u\n", func);

	if(Si46xxCfg.radio_states == Si46xx_Radio_Off)
	{
		Si46xxCfg.radio_states = Si46xx_Radio_Start;
	}

	Si46xxCfg.cfg.audio_only = 1; // TODO erstmal lose hier rein

	// StartUp and Change Ensemble
	if(func == DAB_Play_StartUp || func == DAB_Play_Up || func == DAB_Play_Down)
	{
		Si46xxCfg.cmd.tuneEnsemble = 1;
		Si46xxCfg.cfg.forwardIfInvalid = 1;
	}

	// Channel change matrix
	if(func == DAB_Play_Up)
	{
		Si46xxCfg.wantedService.freqIndex = (Si46xxCfg.wantedService.freqIndex+1) % (DAB_Chan_SIZE-1);
	}
	else if(func == DAB_Play_Down)
	{
		if(Si46xxCfg.wantedService.freqIndex == 0)
		{
			Si46xxCfg.wantedService.freqIndex = DAB_Chan_SIZE - 1;
		}
		else
		{
			Si46xxCfg.wantedService.freqIndex--;
		}
	}
	else if(func == DAB_Play_Right)
	{
		// If last component in service
		if(Si46xxCfg.wantedService.componentID_index == Si46xxCfg.channelData.services[Si46xxCfg.wantedService.serviceID_index].numberComponents-1)
		{
			Si46xxCfg.wantedService.serviceID_index = (Si46xxCfg.wantedService.serviceID_index+1) % (Si46xxCfg.channelData.numServices-1);
			Si46xxCfg.wantedService.componentID_index = 0;
		}
		else
		{
			Si46xxCfg.wantedService.componentID_index++;
		}
	}
	else if(func == DAB_Play_Left)
	{
		// First component in service
		if(Si46xxCfg.wantedService.componentID_index == 0)
		{
			if(Si46xxCfg.wantedService.serviceID_index == 0)
			{
				Si46xxCfg.wantedService.serviceID_index = Si46xxCfg.channelData.numServices -1;
			}
			else
			{
				Si46xxCfg.wantedService.serviceID_index--;
			}
			Si46xxCfg.wantedService.componentID_index = Si46xxCfg.channelData.services[Si46xxCfg.wantedService.serviceID_index].numberComponents-1;
		}
		else
		{
			Si46xxCfg.wantedService.componentID_index--;
		}
	}

	Si46xxCfg.cmd.startService = 1; //TODO stop service hier noch irgendwo wegen Datenservice?

	/*
	 * TODO:
	 * - ChannelID festlegen auf 1 (oder letzter gespeicherter), falls noch nciht geschehen
	 * - serviceID festlegen auf 1. (oder letzter gespeicherter), falls noch nicht geschehen
	 * - componendID festlegen auf 1. (oder letzter gespeicherter), falls noch nicht geschehen
	 *  wenn er die Channelliste noch nicht hat, kann er ja trotdem starten und die Liste kommt nach. Sollte am Display sowieso kont. akt. werden
	 * - START_DIGITAL_SERVICE
	 *
	 * - gespeicherte Sender:
	 *   - separat abspeichern
	 *   - dann passend abrufen und direkt Start machen, der Rest mit Channelliste kann ja danach kommen
	 *
	 * wie soll diese Funktion durchlaufen? nur Flags setzen?
	 */

	// Assuming frequency list has been synched during bootup

}

/* State machine taking care of the SPI communication funcitons */
void Si46xx_function_tasks(void)
{
	if(Si46xxCfg.radio_states == Si46xx_Radio_Off)
	{
		return;
	}

	// Run sub-state machine for firmware jobs
	Si46xx_firmware_tasks();

	if(Si46xx_RemainingTimeLeft() == TIME_LEFT) // TODO: Das ist doppelt mit unten
	{
		return;
	}

	switch(Si46xxCfg.function_state)
	{
		case Si46xx_STATE_IDLE:
			// Leave idle if messages are waiting
			if(Si46xxCfg.cb.count > 0)
			{
				Si46xxCfg.function_state = Si46xx_STATE_SENDING;
			}
			break;

		case Si46xx_STATE_SENDING:

			// hier dann mit ISRs arbeiten...

			// if flag & getSysState: befehl ausführen, größe der Rückgabewerte einsammeln, in wait-state gehen
			// ISR bringt ihn zurück in Antwort-Auswerten-State, aber dann ist auch eine Auswertefunktion nötig...
			// vielleicht einfach ein Struct, wo dann die Senden und Antworten-Funktion gepointert werden?
			// dann eine Aufrufe-Funktion für das jeweilige Feature Si46xx_GetSysState -> Si46xx_Send_GetSysState und Si46xx_Rcv_GetSysState
			// allgemeinen Reply-Pointer bauen? Also auf die Antwortbytes, dass er die Ablegt? Aber dann müsste er in der allgemeinen Auswertefkt ja wissen, was er damit soll...


			//if(cb_pop_front(&Si46xxCfg.cb, &currentWorkingMsg) == CB_OK)
			if(cb_get_front(&Si46xxCfg.cb, &currentWorkingMsg) == CB_OK)
			{
				printf("Si46xx: Got command '%s'...\n", currentWorkingMsg.msgName);

				//Si46xxCfg.function_state = Si46xx_STATE_IDLE; // Back to idle as default, to be changed if message could be sent properly
				// TODO: Hier nicht IDLE hin! das darf erst später gesetzt werden, sonst glauben andere funktionien, diese mach hat nichts zu tun

				// Unset ISR if set before, not if not activated
				if(Si46xxCfg.isrState == ISR_SET)
				{
					Si46xxCfg.isrState = ISR_UNSET;
				}

				switch(currentWorkingMsg.sendFunc())
				{
					case HAL_OK:
						// If no interrupts used (before boot patch) and no timeout time set in function
						if(Si46xxCfg.isrState == ISR_INACTIVE && Si46xx_RemainingTimeLeft() == TIME_OVER)
						{
							Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT);
						}

						Si46xxCfg.function_state = Si46xx_STATE_BUSY;
						break;

					case HAL_ERROR:
						printf("Si46xx: SPI-Error in Sending function!\n");
						Si46xxCfg.function_state = Si46xx_STATE_IDLE;
						break;

					case HAL_BUSY:
					case HAL_TIMEOUT:
						printf("Si46xx: SPI-Busy / Timeout, nochmal...\n");
						//cb_push_back(&Si46xxCfg.cb, &currentWorkingMsg); // TODO: eine aktuell beschäftigte Funktion sollte am Anfang bleiben!
						Si46xx_SetWaitTime(100);

						//TODO: Si46xxCfg.function_state so lassen, oder?

						break;
				}
			}
			else // Reset if getting a message wasn't possible
			{
				printf("Si46xx: Couldn't fetch expected message from stack!\n");
				Si46xxCfg.function_state = Si46xx_STATE_IDLE;
			}

			break;

		case Si46xx_STATE_RESET:
			// Small state machine to keep RST line long enough low
			switch(resetState)
			{
				case Si46xx_RST_SET:
					SI46XX_RST_ON();
					Si46xx_SetWaitTime(50); // Wait 50ms in Reset TODO erstmal hoch gegriffen

					resetState = Si46xx_RST_RELEASED; // next state: Power up the device
					break;

				case Si46xx_RST_RELEASED:
					SI46XX_RST_OFF();
					Si46xx_SetWaitTime(10); // TODO Datenblatt ist 3.2ms to wait after RST

					Si46xxCfg.function_state = Si46xx_STATE_IDLE; // Leave reset state
					break;
			}
			break;

		case Si46xx_STATE_BUSY:

			/* If interrupt was set and is low now  TODO: low-INT einstellen? */
			if(
				(Si46xxCfg.isrState == ISR_SET && (HAL_GPIO_ReadPin(Si46xx_INTB_GPIO_Port, Si46xx_INTB_Pin) == GPIO_PIN_RESET))
				  ||
				((Si46xxCfg.isrState == ISR_FIRST_FLAG || Si46xxCfg.isrState == ISR_INACTIVE) && Si46xx_RemainingTimeLeft() == TIME_OVER) /* If no ISR is being used, just timings */
			  )
			{
				if(Si46xxCfg.isrState == ISR_SET)
				{
					Si46xxCfg.isrState = ISR_UNSET;
				}

				// First flag TODO Test -> in Boot-Befehl oder danach?
				/*if(Si46xxCfg.isrState == ISR_FIRST_FLAG && (HAL_GPIO_ReadPin(Si46xx_INTB_GPIO_Port, Si46xx_INTB_Pin) == GPIO_PIN_RESET))
				{
					Si46xxCfg.isrState = ISR_UNSET;
					printf("Si46xx: Jetzt ISRs ernst nehmen...\n");
				}*/
				Si46xxCfg.analyzedStatus = currentWorkingMsg.receiveFunc();

				switch(Si46xxCfg.analyzedStatus)
				{
					case Si46xx_OK:
						printf("Si46xx: Receive function OK, back to idle...\n\n");

						cb_clear_front(&Si46xxCfg.cb); // receive ok, we can take this message from the ring buffer

						Si46xxCfg.function_state = Si46xx_STATE_IDLE;
						break;

					case Si46xx_SPI_ERROR:
						printf("Si46xx: SPI Error from receive function, keep msg on stack\n");
						//cb_push_back(&Si46xxCfg.cb, &currentWorkingMsg); // otherwise put it back on stack
						Si46xx_SetWaitTime(100);

						break;

					case Si46xx_BUSY: // Busy even if INTB pin set?

						if(Si46xxCfg.isrState == ISR_INACTIVE) // TODO: Dieses Busy nach oben schieben, muss ja nicht jedes Mal die SPI-Funktion aufr.
						{
							Si46xx_SetWaitTime(SI46XX_DEFAULT_SPI_WAIT);
							break; // Without ISR being busy can be normal
						}
						// otherwise no break, continue with error
					case Si46xx_MESSAGE_ERROR:
						printf("Si46xx: Error from receive function!\n");
						Si46xx_SetWaitTime(100);

						// Clear message buffer
						cb_free(&Si46xxCfg.cb);

						// TODO: auf Idle richtig hier?
						Si46xxCfg.function_state = Si46xx_STATE_IDLE;
						break;

					case Si46xx_DEVICE_ERROR: // Serious error, reset device
						printf("Si46xx: Device error in receive function, resetting Si46xx...\n");

						// Clear message buffer
						cb_free(&Si46xxCfg.cb);

						//Si46xx_Boot(); // TODO: Temporärer Befehl
						Si46xx_Reset();// TODO: auch Temporärer Befehl so alleine.

						// TODO: hauptroutine sollte dann auhc wissen, dass sie neu booten muss

						break;
				}
			}

			// TODO: hier Timeoutfunktion
			break;
	}
}

HAL_StatusTypeDef Si46xx_InitConfiguration(SPI_HandleTypeDef * hspi)
{
	SI46XX_CS_OFF();
	SI46XX_RST_ON();

	// Initial config values
	struct Si46xx_Init_Values init =
	{
			.CTS_InterruptEnable = Si46xx_ENABLE,
			.CLK_MODE = Si46xx_XOSC,
			.XOSC_TR_SIZE = 0x07,
			.IBIAS = 0x48,
			.IBIAS_RUN = 0,
			.XTAL_Freq = 19200000, // Hz (0x0124F900 ??)
			.CTUN = 0x1F
	};

	Si46xxCfg.initConfig = init;
	Si46xxCfg.hspi = hspi;

	Si46xxCfg.radio_states = Si46xx_Radio_Off;
	Si46xxCfg.function_state = Si46xx_STATE_IDLE;
	resetState = Si46xx_RST_SET;

	// Initialize ring buffer for messages
	cb_init(&Si46xxCfg.cb, 10, sizeof(Si46xx_msg_dt));

	//Si46xxCfg.timeoutValue = SI46XX_DEFAULT_SPI_WAIT;

	// Start without interrupts
	Si46xxCfg.isrState = ISR_INACTIVE;

	Si46xxCfg.events.service_list_int = 0;

	// Firmware configuration
	Si46xxCfg.firmware_source        = FW_SRC_FLASH; // Set default source
	Si46xxCfg.firmware_flash_address = 0x000; // 0x200

	// Init first channel TODO später vom EEPROM?
	Si46xxCfg.wantedService.freqIndex   = 0;
	Si46xxCfg.wantedService.serviceID_index   = 0;
	Si46xxCfg.wantedService.componentID_index = 0;

	Si46xxCfg.deviceStatus.channelList_valid = NOT_VALID;

	return HAL_OK;
}




/* Timeout and waiting routines --------------------------------- */
void Si46xx_SetWaitTime(uint32_t ms)
{
	Si46xxCfg.waitStamp = HAL_GetTick();
	Si46xxCfg.waitTime  = ms;
}

uint32_t Si46xx_CurrentWaitTime(void)
{
	return HAL_GetTick() - Si46xxCfg.waitStamp;
}

enum Si46xx_Wait_en Si46xx_RemainingTimeLeft(void)
{
	if(Si46xx_CurrentWaitTime() < Si46xxCfg.waitTime)
	{
		return TIME_LEFT;
	}
	else
	{
		return TIME_OVER;
	}
}
