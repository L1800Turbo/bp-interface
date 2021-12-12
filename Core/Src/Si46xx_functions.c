/*
 * Si46xx_functions.c
 *
 *  Created on: 15.04.2021
 *      Author: kai
 */




struct stateParams
{
	Si46xx_status * requestFunction();
	Si46xx_status * answerFunction();

	uint32_t timeout;

	struct nextState // das hier nur als Idee: Was der nächste State sein könnte. Kann man aber wohl besser aus der jeweiligen Funktion als PArameter nehmen?
	// und dann in der allg. statemachine auf Rückgabewert reagieren
	{
		enum Si46xx_States sucess;
		enum Si46xx_States fail;
	};
};

struct stateParams stateParams[Si46xx_STATE_COUNT] = // oder diese funktion allgemein... wobei das die SM deutlich vereinfachen würde
		// in der Statemachine dann eine Art Busy-Dings einbauen, wo man auf diese Funktionen zugreifen kann. Oder? am besten aufmalen
{
		{.requestFunction = Si46xx_Send_Boot, .answerFunction = Si46xx_Generic_Answer, .timeout = SPI_DEFAULT},	/* Si46xx_STATE_STARTUP oder so..... */
};


//hier: solche dann
HAL_StatusTypeDef Si46xx_Send_Boot(void)
{
	uint8_t data[2];
	HAL_StatusTypeDef state = HAL_OK;

	Si46xxCfg.timeoutValue = 300;

	data[0] = SI46XX_BOOT;
	data[1] = 0x00;

	state = Si46xx_SPIsend(Si46xxCfg.hspi, data, 2);

	return state;
}
