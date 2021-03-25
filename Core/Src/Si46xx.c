/*
 * Si46xx.c
 *
 * Generic functions to work with Si46xx
 *
 *  Created on: 21.03.2021
 *      Author: kai

TODO:
- Generische Funktion, die SPI-Befehl absetzt
- Generische Funktion, die Antwort-Befehl absetzt und Antwort auswertet
	- die Länge der Rückgaben und so müsste man dann noch als Parameter übergeben (in irgendwie eine Config packen?

Befehl Senden IT -> IT vom Si46, wenn Takko, dann Befehl SPI holen

Programmierung über DMA UART->SPI??

so zS bauen?
   Configure MEMS: data rate, power mode, full scale, self test and axes
    ctrl = (uint16_t) (lis302dl_initstruct.Output_DataRate | lis302dl_initstruct.Power_Mode | \
                       lis302dl_initstruct.Full_Scale | lis302dl_initstruct.Self_Test | \
                       lis302dl_initstruct.Axes_Enable);

 */
#include "Si46xx.h"

struct Si46xx_Config Si46xxCfg;

#define SI46XX_CS_ON()  HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_RESET)
#define SI46XX_CS_OFF() HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_SET)

#define SI46XX_INTB_STATE HAL_GPIO_ReadPin(Si46xx_INTB_GPIO_Port, Si46xx_INTB_Pin)

#define SI46XX_RST_ON()  HAL_GPIO_WritePin(Si46xx_RSTB_GPIO_Port, Si46xx_RSTB_Pin, GPIO_PIN_RESET); // Reset LOW
#define SI46XX_RST_OFF() HAL_GPIO_WritePin(Si46xx_RSTB_GPIO_Port, Si46xx_RSTB_Pin, GPIO_PIN_SET);
#define SI46XX_RST_WAIT_BEFORE 10 // ms
#define SI46XX_RST_WAIT_AFTER   4 // 3.2ms




/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_GPIO_TogglePin(LED_ORANGE_Port, LED_ORANGE_Pin);
	if(GPIO_Pin == Si46xx_INTB_Pin)
	{
		Si46xxCfg.interruptFlag = Si46xx_INT_FLAG_SET;
	}
}*/

uint8_t spiBuffer[20];

/**
 * Send command via SPI
 */
Si46xx_Error Si46xx_SPIsend(SPI_HandleTypeDef * hspi, uint8_t * data, uint16_t len)
{

	// TODO Abfrage, ob sendebereit
	// TODO Data ist doch nur gepointert, der übrschreibt das doch irgendwann?!?
	memcpy(spiBuffer, data, len);


	SI46XX_CS_ON();
	//HAL_SPI_TransmitReceive_IT(hspi, spiBuffer, (uint8_t *)NULL, len); // geht niht, NULL macht HAL_ERROR
	HAL_SPI_Transmit_IT(hspi, spiBuffer, len); //TODO: keine Pausen...
	//SI46XX_CS_OFF(); -> Durch Interrupt...

	return Si46xx_OK;
}

void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef * hspi)
{
  // Set CS pin to high and raise flag
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  //spi_xmit_flag = 1;
  SI46XX_CS_OFF();
}

/**
 * Get status reply from last command
 */
Si46xx_Error Si46xx_SPIgetStatus(SPI_HandleTypeDef * hspi, uint8_t * data, uint16_t len)
{
	if(len < 4)
	{
		len = 4;
	}

	uint8_t zeroes[1+len]; // {0, ??? TODO dynamisch mit memalloc oder sowas
	memset(zeroes, 0, 1+len);

	Si46xx_Error state = Si46xx_OK;
	// TODO Abfrage, ob sendebereit

	//HAL_SPI_STATE_BUSY_RX

	SI46XX_CS_ON();
	state = HAL_SPI_TransmitReceive(hspi, zeroes, data, len+1, 100); // TODO irgendwann als IT Version? TODO status ist kein si46xxx.... muss gewandelt!!
	SI46XX_CS_OFF();

	//printf("State: %d\r\n",state);

	return state;
}

Si46xx_Error Si46xx_InitConfiguration(SPI_HandleTypeDef * hspi)
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

	Si46xxCfg.intstate = SI46XX_INTB_STATE;

	Si46xxCfg.state = Si46xx_STATE_SAFE_OFF; //Si46xx_STATE_STARTUP;

	return Si46xx_OK;
}

Si46xx_Error Si46xx_Send_PowerUp()
{
	Si46xx_Error state = Si46xx_OK;
	uint8_t data[16];

	// Prepare data into SPI buffer
	data[0]  = SI46XX_POWER_UP;
	data[1]  = (Si46xxCfg.initConfig.CTS_InterruptEnable << 7);
	data[2]  = (Si46xxCfg.initConfig.CLK_MODE << 4) | Si46xxCfg.initConfig.XOSC_TR_SIZE;
	data[3]  = (Si46xxCfg.initConfig.IBIAS & 0x7F);
	data[4]  =  Si46xxCfg.initConfig.XTAL_Freq 		   & 0xFF;
	data[5]  = (Si46xxCfg.initConfig.XTAL_Freq >>  8 ) & 0xFF;
	data[6]  = (Si46xxCfg.initConfig.XTAL_Freq >> 16 ) & 0xFF;
	data[7]  = (Si46xxCfg.initConfig.XTAL_Freq >> 24 ) & 0xFF;
	data[8]  = (Si46xxCfg.initConfig.CTUN & 0x3F);
	data[9]  = 0x00 | (1 << 4);
	data[10] = 0x00;
	data[11] = 0x00;
	data[12] = 0x00;
	data[13] = (Si46xxCfg.initConfig.IBIAS_RUN & 0x7F);
	data[14] = 0x00;
	data[15] = 0x00;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 16);

	Si46xxCfg.timeoutVal = HAL_GetTick(); // For first waiting time before triggering RST pin

	return state;
}

void Si46xx_Send_Reset(void)
{
	SI46XX_RST_ON();

	Si46xxCfg.timeoutVal = HAL_GetTick();
	Si46xxCfg.state = Si46xx_STATE_STARTUP;
}

void progress_StatusBytes(uint8_t * data)
{
	struct Si46xx_Status_Values * status = &Si46xxCfg.currentStatus;

	/* Progress into status variables */

	status->CTS      = (data[0] & 0x80)>>7;
	status->ERR_CMD  = (data[0] & 0x40)>>6;
	status->PUP      = (data[3] & 0xC0)>>6;
	status->RFFE_ERR = (data[3] & 0x20)>>5;
	status->REPOFERR = (data[3] & 0x08)>>3;
	status->CMDOFERR = (data[3] & 0x04)>>2;
	status->ARBERR   = (data[3] & 0x02)>>1;
	status->ERRNR    = (data[3] & 0x01);
}

void Si46xx_Tasks()
{
	switch(Si46xxCfg.state)
	{
		case Si46xx_STATE_SAFE_OFF:
			SI46XX_RST_ON();
			break;

		case Si46xx_STATE_STARTUP:
			if(HAL_GetTick() - Si46xxCfg.timeoutVal > SI46XX_RST_WAIT_BEFORE /*&& HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1*/)
			{
				printf("\033[1;36mSi46xx_Tasks: STARTUP\033[0m\r\n");

				SI46XX_RST_OFF();
				Si46xxCfg.timeoutVal = HAL_GetTick();

				Si46xxCfg.state = Si46xx_STATE_INIT;
			}
			break;

		case Si46xx_STATE_INIT:
			if((HAL_GetTick() - Si46xxCfg.timeoutVal) < SI46XX_RST_WAIT_AFTER)
			{
				break;
			}

			printf("\033[1;36mSi46xx_Tasks: INIT\033[0m\r\n");

			if(Si46xx_Send_PowerUp() == Si46xx_OK)
			{
				Si46xxCfg.answerBytes = 4;

				Si46xxCfg.stateBefore = Si46xxCfg.state;
				Si46xxCfg.state       = Si46xx_STATE_ANSWER;
				Si46xxCfg.stateAfter  = Si46xx_STATE_LOAD_FIRMWARE;

				Si46xxCfg.timeoutVal = HAL_GetTick(); // TODO temp
			}
			else
			{
				Si46xxCfg.state = Si46xx_STATE_SAFE_OFF;
			}
			break;

		case Si46xx_STATE_ANSWER:
			//if(Si46xxCfg.interruptFlag == Si46xx_INT_FLAG_SET)
			//if(Si46xxCfg.intstate != SI46XX_INTB_STATE)
			if(((HAL_GetTick() - Si46xxCfg.timeoutVal) > 10) && Si46xxCfg.hspi->State == HAL_SPI_STATE_READY) // TODO Temporätr
			{
				uint8_t data[1 + Si46xxCfg.answerBytes]; // TODO: Geht das??
				//uint8_t * data;
				/* Answer is ready after interrupt */ //TODO interrupt/polling driven Auswahl? Bei fehlern auch INT?
				Si46xxCfg.intstate = SI46XX_INTB_STATE;

				Si46xxCfg.timeoutVal = HAL_GetTick(); // TODO temp weiter warten, das INT-Flag tut irgendwie noch nicht

				//data = malloc(Si46xxCfg.answerBytes); // TODO: lässt malloc die auch selbst wieder los?
				// free(data);

				if(Si46xx_SPIgetStatus(Si46xxCfg.hspi, data, Si46xxCfg.answerBytes) == Si46xx_OK) // TODO ohne Else loopt er dann, wenn er busy ist
				{
					progress_StatusBytes(&data[1]);

					if // Reset device
					(
						Si46xxCfg.currentStatus.ARBERR == Si46xx_ERR_ERROR || /* An arbiter overflow has occurred. The only way to recover is for the user to reset the chip. */
						Si46xxCfg.currentStatus.ERRNR  == Si46xx_ERR_ERROR /* Fatal error has occurred. The only way to recover is for the user to reset the chip.*/
					)
					{
						Si46xx_Send_Reset();
						break;
					}
					else if // Resend last command
					(
							Si46xxCfg.currentStatus.CMDOFERR == Si46xx_ERR_ERROR || /* The command interface has overflowed, and data has been lost */
							Si46xxCfg.currentStatus.REPOFERR == Si46xx_ERR_ERROR    /* The reply interface has underflowed, and bad data has been returned to the user */
					)
					{
						Si46xxCfg.state       = Si46xxCfg.stateBefore;
						Si46xxCfg.stateBefore = Si46xx_STATE_SAFE_OFF; // als 0
						Si46xxCfg.stateAfter  = Si46xx_STATE_SAFE_OFF; // als 0
						break;
					}

					if(Si46xxCfg.currentStatus.CTS == Si46xx_CTS_READY) // Ready for next command
					{
						Si46xxCfg.state 	  = Si46xxCfg.stateAfter;
						Si46xxCfg.stateBefore = Si46xx_STATE_SAFE_OFF; // als 0
						Si46xxCfg.stateAfter  = Si46xx_STATE_SAFE_OFF; // als 0


						// TODO: Aber wenn andere Befehle ausgeführt werden?
						// TODO: das PUP-Ding macht noch 0x0? Ist der nicht im BL?
					}

				}
			}
			break;

		case Si46xx_STATE_LOAD_FIRMWARE:
			//printf("\033[1;36mSi46xx_Tasks: LOAD_FIRMWARE\033[0m\r\n");
			// TODO: warten, bis Interrupt kommt, aktuellen Status laden, dann FW schreiben
			// Szenarien: Zunächst auf UART dann warten, später aus flash, gucken, ob Flash I.O.?
			break;

		case Si46xx_STATE_BOOT:
			// TODO
			break;

		//case DAB konfiguieren: analogen Ausgang, Lautstärke 100%
		//case Sendersuchlauf (beim ersten MAl aufrufen, wnen nichts abgespeichert ist, oder manuell aufgerufen wird?)
		//case Tune auf einen Sender (letzten gespeicherten oder 0)
		//case Change Frequenz und Channel und so (später dann mit den Pfeiltasten...
		//case Get Station Text: Lied und sowas
	}
}



