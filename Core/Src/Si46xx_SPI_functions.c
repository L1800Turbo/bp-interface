/*
 * Si46xx_SPI_functions.c
 *
 *  Created on: Dec 31, 2021
 *      Author: kai
 */
#include "Si46xx_SPI_functions.h"
#include <stdio.h>
#include <string.h>

/* Private functions ------------------------------------------- */
HAL_StatusTypeDef Si46xx_Msg_GetSysState_sendFunc();
Si46xx_statusType Si46xx_Msg_GetSysState_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_GetDigitalServiceList_sendFunc();
Si46xx_statusType Si46xx_Msg_GetDigitalServiceList_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_DABtuneFreq_sendFunc();
Si46xx_statusType Si46xx_Msg_DABtuneFreq_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_SetFreqList_sendFunc();
Si46xx_statusType Si46xx_Msg_SetFreqList_receiveFunc();
HAL_StatusTypeDef Si46xx_Msg_GetFreqList_sendFunc();
Si46xx_statusType Si46xx_Msg_GetFreqList_receiveFunc();


extern struct Si46xx_Config Si46xxCfg;
uint8_t spiBuffer[4096];

extern Si46xx_statusType Si46xx_SPIgetAnalyzeStatus(uint8_t * data, uint16_t len);

const Si46xx_msg_dt Si46xx_messages[SI46XX_MSG_SIZE] = {
		{0}, /* SI46XX_MSG_NONE */
		{
				.msgIndex    = SI46XX_MSG_REFRESH_SYS_STATE,
				.msgName	 = "SI46XX_MSG_REFRESH_SYS_STATE",
				.sendFunc    = Si46xx_Msg_GetSysState_sendFunc,
				.receiveFunc = Si46xx_Msg_GetSysState_receiveFunc
		},	/* SI46XX_MSG_REFRESH_SYS_STATE */
		{
				.msgIndex    = SI46XX_MSG_GET_DIGITAL_SERVICE_LIST,
				.msgName	 = "SI46XX_MSG_GET_DIGITAL_SERVICE_LIST",
				.sendFunc    = Si46xx_Msg_GetDigitalServiceList_sendFunc,
				.receiveFunc = Si46xx_Msg_GetDigitalServiceList_receiveFunc
		},  /* SI46XX_MSG_GET_DIGITAL_SERVICE_LIST */
		{
				.msgIndex    = SI46XX_MSG_DAB_TUNE_FREQ,
				.msgName	 = "SI46XX_MSG_DAB_TUNE_FREQ",
				.sendFunc    = Si46xx_Msg_DABtuneFreq_sendFunc,
				.receiveFunc = Si46xx_Msg_DABtuneFreq_receiveFunc
		},  /* SI46XX_MSG_DAB_TUNE_FREQ */
		{
				.msgIndex	 = SI46xx_MSG_SET_FREQ_LIST,
				.msgName	 = "SI46xx_MSG_SET_FREQ_LIST",
				.sendFunc	 = Si46xx_Msg_SetFreqList_sendFunc,
				.receiveFunc = Si46xx_Msg_SetFreqList_receiveFunc
		},	/* SI46xx_MSG_SET_FREQ_LIST */
		{
				.msgIndex    = SI46XX_MSG_GET_FREQ_LIST,
				.msgName	 = "SI46XX_MSG_GET_FREQ_LIST",
				.sendFunc    = Si46xx_Msg_GetFreqList_sendFunc,
				.receiveFunc = Si46xx_Msg_GetFreqList_receiveFunc
		}	/* SI46XX_MSG_GET_FREQ_LIST     */
};


/* Basic SPI routines ---------------------------------------------------------------- */

/**
 * Send command via SPI
 */
HAL_StatusTypeDef Si46xx_SPIsend(SPI_HandleTypeDef * hspi, uint8_t * data, uint16_t len)
{

	// TODO Abfrage, ob sendebereit
	memcpy(spiBuffer, data, len);


	SI46XX_CS_ON();
	//HAL_SPI_TransmitReceive_IT(hspi, spiBuffer, (uint8_t *)NULL, len); // geht niht, NULL macht HAL_ERROR
	HAL_SPI_Transmit_IT(hspi, spiBuffer, len); //TODO: keine Pausen...
	//SI46XX_CS_OFF(); -> Durch Interrupt...

	return HAL_OK;
}

/* Callback for finished transmit */
void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef * hspi)
{
	/* Deactivate chip select */
	SI46XX_CS_OFF();
}

/**
 * Get status reply from last command
 */
HAL_StatusTypeDef Si46xx_SPIgetStatus(SPI_HandleTypeDef * hspi, uint8_t * data, uint16_t len)
{
	HAL_StatusTypeDef state = HAL_ERROR;

	/* Stop here if SPI isn't ready */
	if(hspi->State != HAL_SPI_STATE_READY)
	{
		return HAL_BUSY; // TODO: erstmal Busy. Wenn ein Fehler ist, sollte der aber auch gecatcht werden...
	}

	if(len < 4)
	{
		len = 4;
	}

	uint8_t zeroes[1+len]; // {0, ??? TODO dynamisch mit memalloc oder sowas
	memset(zeroes, 0, 1+len);

	SI46XX_CS_ON();
	state = HAL_SPI_TransmitReceive(hspi, zeroes, data, len+1, 100); // TODO irgendwann als IT Version? TODO status ist kein si46xxx.... muss gewandelt!!
	SI46XX_CS_OFF();

	return state;
}





/* Message functions to be called from stack ------------------- */
HAL_StatusTypeDef Si46xx_Msg_GetSysState_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		data[0] = SI46XX_GET_SYS_STATE;
		data[1] = 0x00;

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);

		// As maximum wait time:
		//Si46xx_SetWaitTime(100);
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_GetSysState_receiveFunc()
{
	uint8_t data[6];

	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 5);

	if(state != Si46xx_OK)
	{
		return state;
	}

	Si46xxCfg.image = data[5];

	//printf("Si46xx: Image %d\n", Si46xxCfg.image);
	printf("ssys_%d\n", Si46xxCfg.image);

	return state;
}

/* SI46XX_MSG_GET_DIGITAL_SERVICE_LIST */
HAL_StatusTypeDef Si46xx_Msg_GetDigitalServiceList_sendFunc()
{
	uint8_t data[6];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		// If tune is not marked as complete, check once if it is now
		if(Si46xxCfg.deviceStatus.STCINT == Si46xx_STCINT_INCOMPLETE)
		{
			/*state = */Si46xx_SPIgetAnalyzeStatus(data, 5); // State nicht aktualisieren, soll busy bleiben
		}

		if(Si46xxCfg.deviceStatus.STCINT == Si46xx_STCINT_COMPLETE)
		{
			data[0] = SI46XX_GET_DIGITAL_SERVICE_LIST;
			data[1] = 0x00;

			state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);
		}

		// As maximum wait time:
		//Si46xx_SetWaitTime(100); // TODO Timeout und so...
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_GetDigitalServiceList_receiveFunc()
{
	//uint8_t data[7]; // TODO 7, temporär festgelegt, was ist der maximale Wert hier?
	uint8_t * data =spiBuffer;

	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 6);
	uint16_t length = 0;

	if(state != Si46xx_OK)
	{
		return state;
	}

	length = (uint16_t) data[5] + (data[6] << 8);
	printf("Si46xx: Size of service list: %d bytes \n", length);

	//Si46xxCfg.image = spiBuffer[5];

	state = Si46xx_SPIgetAnalyzeStatus(data, 6 + length);

	if(state != Si46xx_OK || length == 0)
	{
		return state;
	}

	// Send message for debug
	printf("sSrvList_%d\n", 6+length);
	CDC_Transmit_FS(data, 6+length);

	/* TODO: Digitale Serviceliste auswerten:
	 * 8.5
 Finding a Digital Service
Before starting a data service, the host must collect information about the services that exist in the ensemble. This is
done using the GET_DIGITAL_SERVICE_LIST host command. The service list definitions for DAB is described in
“8.5.1. DAB/DMB Radio Service List”, the following table shows the format of the GET_DIGITAL_SERVICE_LIST
host command.
	 *
	 */

	uint8_t * bufPtr = data + 5; // Point to List Size
	length = bufPtr[0] + (bufPtr[1] << 8);

	uint8_t numberServices = bufPtr[4];
	printf("Si46xx: # of services: %d \n", numberServices);

	uint8_t position = 8; // initial...
	bufPtr += position;

	// Loop through services
	for(uint8_t i=0; i<numberServices; i++)
	{

		uint32_t serviceID = bufPtr[0] + (bufPtr[1] << 8) + (bufPtr[2] << 16) + (bufPtr[3] << 24);
		uint8_t pdFlag = bufPtr[4] & 0x01;

		struct serviceID_P // P/D=0
		{
			uint16_t SRV_REF:12;
			uint8_t CountryID:4;
			uint16_t RFU;
		};

		struct serviceID_D // P/D=1
		{
			uint32_t SRV_REF:20;
			uint8_t CountyID:4;
			uint8_t ECC;
		};

		uint8_t numberComponents = bufPtr[5] & 0x0F;
		char serviceLabel[16+1];
		memcpy(serviceLabel, &bufPtr[8], 16);
		serviceLabel[16] = '\0';

		printf("Si46xx: ServiceID: %lX, P/D: %d, Label: %s \n", serviceID, pdFlag, serviceLabel);

		struct serviceID_P * srvID_P;
		struct serviceID_D * srvID_D;

		switch(pdFlag)
		{
			case 0:
				srvID_P = (struct serviceID_P*) &serviceID;
				printf("Si46xx: SRV_REF: %X CountryID: %d\n", srvID_P->SRV_REF, srvID_P->CountryID);
				break;

			case 1:
				srvID_D = (struct serviceID_D*) &serviceID;
				printf("Si46xx: SRV_REF: %X CountryID: %d, ECC: %X\n", srvID_D->SRV_REF, srvID_D->CountyID, srvID_D->ECC);
				break;
		}

		printf("Si46xx: # of components: %d \n", numberComponents);

		// Jump to components
		//position += 24;
		bufPtr += 24;

		// Loop through components in the service
		for(uint8_t j=0; j<numberComponents; j++)
		{
			// Adjust pointer to current component
			//bufPtr += position;

			uint8_t tmID = bufPtr[1] >> 6;
			uint16_t componentID = 0;

			switch(tmID) // Component ID depends on the TM ID
			{
				case 0: // TMId=00 (MSC stream audio)
				case 1: // TMId=01 (MSC stream data)
				case 2: // TMId=10 (Reserved)
					componentID = bufPtr[0] & 0x3F;
					break;

				case 3: // TMId=11 (MSC packet data)
					componentID = bufPtr[0] + ((bufPtr[1] & 0x0F) << 8);
					// DGFlag is on Bit 13
					break;
			}

			uint8_t ascTy_dscTy = bufPtr[2] >> 2;


			printf("Si46xx:      TMId: %X, ComponentID: %X, ASCTy/DSCTy: %d\n", tmID, componentID, ascTy_dscTy);

			// Jump to next component in current service block
			//position += 4;
			bufPtr += 4;
		}
	}

	return state;
}

//SI46XX_MSG_DAB_TUNE_FREQ
HAL_StatusTypeDef Si46xx_Msg_DABtuneFreq_sendFunc()
{
	uint8_t data[6];
	HAL_StatusTypeDef state = HAL_BUSY;

	// TODO erst nur als Test üfr 5C
	//Si46xxCfg.freqIndex = DAB_Chan_5C;

	// Check configuration
	if(Si46xxCfg.freqIndex > 47)
	{
		return HAL_ERROR;
	}

	Si46xxCfg.deviceStatus.STCINT = Si46xx_STCINT_COMPLETE; // für den ersten... TODO

	// No other tuning should be running
	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY  && Si46xxCfg.deviceStatus.STCINT == Si46xx_STCINT_COMPLETE)
	{
		data[0] = SI46XX_DAB_TUNE_FREQ;
		data[1] = 0x00;
		data[2] = Si46xxCfg.freqIndex;
		data[3] = 0x00;

		/* ANTCAP[15:0] (data[4] und data[5])
    		When non-zero this parameter sets the antenna tuning capacitor value to (ANTCAP-1)*250 fF (31.75 pF Max).
    		If left at 0, the capacitor is automatically determined.
			Type:		U16
			Min:		0
			Max:		128
			Default:	0
			Unit:		250 fF
		 */
		data[4] = 0;
		data[5] = 0;

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 6);

		// As maximum wait time:
		//Si46xx_SetWaitTime(100); // TODO Timeout und so...
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_DABtuneFreq_receiveFunc()
{
	uint8_t data[5];
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 4);

	if(state != Si46xx_OK)
	{
		return state;
	}

	//Si46xxCfg.image = spiBuffer[5];

	if(Si46xxCfg.deviceStatus.STCINT == Si46xx_STCINT_INCOMPLETE)
	{
		printf("Si46xx: Tune not complete... \n");
	}
	else
	{
		printf("Si46xx: Tune complete! \n");
	}


	return state;
}

//SI46xx_MSG_SET_FREQ_LIST
HAL_StatusTypeDef Si46xx_Msg_SetFreqList_sendFunc() // TODO: Muss getestet werden!
{
	uint8_t data[196] = {0,};
	HAL_StatusTypeDef state = HAL_BUSY;


	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		data[0] = SI46XX_SET_FREQ_LIST;
		data[1] = (DAB_Chan_SIZE & 0xFF);
		data[2] = 0; // Normal guaranteed tuning range 168 MHz to 240 MHz
		data[3] = 0;

		for(uint8_t i=0, j=0; i<DAB_Chan_SIZE; i++, j+=4)
		{
			data[4+j+0] = (DAB_frequency_list[i].freq & 0x000000FF) >>  0;
			data[4+j+1] = (DAB_frequency_list[i].freq & 0x0000FF00) >>  8;
			data[4+j+2] = (DAB_frequency_list[i].freq & 0x00FF0000) >> 16;
			data[4+j+3] = (DAB_frequency_list[i].freq & 0xFF000000) >> 24;

			if(i>47) // maximum for list
			{
				break;
			}
		}

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, sizeof(data));

		// Reset list status as we need to check it afterwards
		Si46xxCfg.freqencyListStatus = FREQ_LIST_INVALID;
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_SetFreqList_receiveFunc()
{
	// TODO: das ist eigentlich eine generische Funktion, wie auch DABtuneFreq
	uint8_t data[5];
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 4);

	return state;
}


// SI46XX_MSG_GET_FREQ_LIST
HAL_StatusTypeDef Si46xx_Msg_GetFreqList_sendFunc()
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_BUSY;

	if(Si46xxCfg.deviceStatus.CTS == Si46xx_CTS_READY)
	{
		data[0] = SI46XX_GET_FREQ_LIST;
		data[1] = 0x00;

		state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);

		// As maximum wait time:
		//Si46xx_SetWaitTime(100); // TODO Timeout und so... muss eine andere Funktion gebaut werden
	}

	return state;
}

Si46xx_statusType Si46xx_Msg_GetFreqList_receiveFunc()
{
	uint8_t data[201];
	uint8_t * dataPtr;
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 200);

	if(state != Si46xx_OK)
	{
		return state;
	}

	printf("Si46xx: Frequencies: %d\n", data[5]);

	dataPtr = data + 9;

	// Set frequency list status to valid, reset it to invalid if it doesn't match
	Si46xxCfg.freqencyListStatus = FREQ_LIST_VALID;

	for(uint8_t i=0, j=0; i<DAB_Chan_SIZE; i++, j+=4)
	{
		// saved frequency at current point
		uint32_t currentSpiFreq = (uint32_t) dataPtr[j]+(dataPtr[j+1]<<8)+(dataPtr[j+2]<<16)+(dataPtr[j+3]<<24);

		if(DAB_frequency_list[i].freq != currentSpiFreq)
		{
			printf("Si46xx_Msg_GetFreqList_receiveFuncFrequencies index %d doesn't match!\n", i);

			Si46xxCfg.freqencyListStatus = FREQ_LIST_INVALID;
			break;
		}
	}

	// Transmit frequency list to terminal if valid
	if(Si46xxCfg.freqencyListStatus == FREQ_LIST_VALID)
	{
		// Send frequency list
		printf("sFreqList_%d\n", sizeof(DAB_frequency_list));
		CDC_Transmit_FS(DAB_frequency_list, sizeof(DAB_frequency_list));
	}

	/*for(uint8_t i=0x08; i<0xC7; i+=4)
	{
		printf("Si46xx: Freq at %d: %lu \n", i,
				(uint32_t) data[i+1]+(data[i+2]<<8)+(data[i+3]<<16)+(data[i+4]<<24));
	}*/

	return state;
}

