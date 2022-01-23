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

	printf("Si46xx: Image %d\n", Si46xxCfg.image);

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
	uint8_t data[7];

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
		uint8_t numberComponents = bufPtr[5] & 0x0F;
		char serviceLabel[16+1];
		memcpy(serviceLabel, &bufPtr[8], 16);
		serviceLabel[16] = '\0';

		printf("Si46xx: ServiceID: %lu, Label: %s \n", serviceID, serviceLabel);
		printf("Si46xx: # of components: %d \n", numberComponents);

		// Jump to components
		//position += 24;
		bufPtr += 24;

		// Loop through components in the service
		for(uint8_t j=0; j<numberComponents; j++)
		{
			// Adjust pointer to current component
			//bufPtr += position;

			uint16_t componentID = bufPtr[0] + (bufPtr[1] << 8);

			printf("Si46xx:       ComponentID: %u\n", componentID);

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
	Si46xx_statusType state = Si46xx_SPIgetAnalyzeStatus(data, 200);

	if(state != Si46xx_OK)
	{
		return state;
	}

	//Si46xxCfg.image = spiBuffer[5];


	printf("Si46xx: Frequencies: %d\n", data[5]);

	for(uint8_t i=0x08; i<0xC7; i+=4)
	{
		printf("Si46xx: Freq at %d: %lu \n", i,
				(uint32_t) data[i+1]+(data[i+2]<<8)+(data[i+3]<<16)+(data[i+4]<<24));
	}

	return state;
}

