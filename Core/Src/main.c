/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
  
/*
- Senden mit Interrupt
- generell Senden einbauen, wann antwortet das teil åberhaupt?

- Timeouts einbauen

- messageSende stack bauen, welche Zeiten dazwischen? Timer für Slots?

- Error Catchen, wenn Nachricht > als MAximum

#- Wartezeit immer für nach der Nachricht, nicht für vor die nachfolgende Nachricht
- Befehle, die das Radio außerhalb von Antworten liefert auf Stack zurück werfen, damit später bearbneiotet wird
- RINGBUF_WAIT raus schmeißen

bpMessages.c bauen:
findMessage(bp-message) returnt enum mit bekannten nachrichten oder unknown
-> muss mehr als addr und cmd kennen...
array weiter ausbauen

- Als Compiler-Flag: Array kleiner als Enum?
-unbekannte Nachrichten sollten gespeichert werden, am besten auch den Status drum herum

- die Nachrichten auf einen eigenen Stack kleben, wenn sie zum Debuggen genommen werden sollen. Sonst fehlen immer welche...

*/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#include "usbd_cdc.h" //test

//#include <locale.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */


// TODO: das alles in struct einbinden

//uint8_t buffer_tmp[250];
uint8_t buffer[250];

extern USBD_HandleTypeDef hUsbDeviceFS; //test


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

//bp_msg_error processBpMsg(bp_msg_dt * message); // nur als Definition, später in header-File

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Printf for USB CDC */
int _write(int file, char *ptr, int len)
{
	extern USBD_HandleTypeDef hUsbDeviceFS;
	extern enum bp_comm_state bpCommState;

	char tmpBuffer[20];
	uint8_t tmpLen;

	// Add state to message
	sprintf(tmpBuffer, " %02d | ", bpCommState);

	// Store length
	tmpLen = strlen(tmpBuffer);
	
	// wait until the buffer is free
	while(((USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData))->TxState!=0); // Blockiert, wenn kein Putty dran

	// copy the string into the buffer (with size limit)
	if (len+tmpLen > sizeof(buffer))
	{ 
		len = sizeof(buffer) - tmpLen;
	}
	
	memcpy(buffer, tmpBuffer, tmpLen);
	memcpy(buffer+tmpLen, ptr, len); // TODO: alles ungetestet
	
	CDC_Transmit_FS(buffer, tmpLen+len);

	return len;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
//    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

	/* Send next message if lasting on buffer */ // TODO: muss da nicht ein Zeitversatz rein? Bekommt das Radio das hin? Was sagen die Aufzeichnungen
	//sendRingMessage(); besser von da unten aus...?
	
	/* Check next received message if transmission was successful */
	//bpMsgState.checkMsgFlg = 1; hier prüfen und oben aktivieren / kopieren?!?
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  bpCommInit();

  Si46xx_InitConfiguration(&hspi1);

  // start inactive DAB mode
  stateFlags.bpDabActive = bp_DAB_inactive;
  strcpy(stateFlags.currentDisplayMessage, "DAB-Sim");

  // TODO: Tests für Si46xx-Kommunikation  Si4684
  /*HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_SET); // CS aus
  HAL_GPIO_WritePin(Si46xx_RSTB_GPIO_Port, Si46xx_RSTB_Pin, GPIO_PIN_RESET); // Reset LOW halten

  static uint8_t zeroes[10] = {0,};
*/
  uint8_t dummy = 0x55;
  HAL_SPI_Transmit(&hspi1, (uint8_t*) &dummy, 1, 100); // Workaround: zu Beginn ist SCK HIGH, erst danach immer LOW...
/*
  while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0);
  HAL_GPIO_WritePin(LED_ORANGE_Port, LED_ORANGE_Pin, GPIO_PIN_SET);

  HAL_Delay(1000);
  HAL_GPIO_WritePin(Si46xx_RSTB_GPIO_Port, Si46xx_RSTB_Pin, GPIO_PIN_SET); // aus RST raus
  HAL_Delay(200);

#define SI46XX_POWER_UP 0x01
#define SI46XX_GET_PART_INFO 0x08
#define SI46XX_GET_SYS_STATE 0x09
  uint8_t data[16];
  uint8_t recData[7];

	data[0] = SI46XX_POWER_UP; // COMMAND
	data[1] = 0x00; // ARG1 disable CTSIEN
	data[2] = (1 << 4) | (7 << 0); // ARG2 CLK_MODE=0x1 TR_SIZE=0x7
	data[3] = 0x48; // ARG3 IBIAS=0x48
	data[4] = 0x00; // ARG4 XTAL
	data[5] = 0xF9; // ARG5 XTAL // F8
	data[6] = 0x24; // ARG6 XTAL
	data[7] = 0x01; // ARG7 XTAL 19.2MHz
	data[8] = 0x1F; // ARG8 CTUN
	data[9] = 0x00 | (1 << 4); // ARG9
	data[10] = 0x00; // ARG10
	data[11] = 0x00; // ARG11
	data[12] = 0x00; // ARG12
	data[13] = 0x00; // ARG13 IBIAS_RUN
	data[14] = 0x00; // ARG14
	data[15] = 0x00; // ARG15

	HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_RESET); // CS an
	HAL_SPI_Transmit(&hspi1, data, 16, 100);
	HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_SET); // CS aus
	HAL_Delay(1);
	HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_RESET); // CS an

	//memset(data, 0x00, 5*sizeof(uint8_t));
	//HAL_SPI_Transmit(&hspi1, data, 1, 100);
	//HAL_SPI_Receive(&hspi1, recData, 4, 100); -> Sendet immer noch Müll, siehe aufgerufene Funktion
	HAL_SPI_TransmitReceive(&hspi1, zeroes, recData, 5, 100);
	//while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_SET); // CS aus

	HAL_Delay(1);

	data[0] = SI46XX_GET_SYS_STATE;
	data[1] = 0;

	HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(&hspi1, data, 2, 100) != HAL_OK)
	{
		HAL_GPIO_WritePin(LED_RED_Port, LED_RED_Pin, GPIO_PIN_SET);
	}
	HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_SET);

	HAL_Delay(1);
	HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_RESET);

	data[0] = 0;
	//HAL_SPI_Transmit(&hspi1, data, 1, 100);
	//HAL_SPI_Receive(&hspi1, recData, 5, 100); // TODO: einer mehr in der ugreen?!?
	HAL_SPI_TransmitReceive(&hspi1, zeroes, recData, 6, 100);
	HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_SET);

	printf("bla: %X, %X, %X, %X, %X\r\n",
			recData[1],
			recData[2],
			recData[3],
			recData[4],
			recData[5]
		 );*/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  bpDebugPrint();
	  bpCommTasks();

	  Si46xx_Tasks();

	  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1)
	  {
		  HAL_GPIO_WritePin(Si46xx_RSTB_GPIO_Port, Si46xx_RSTB_Pin, GPIO_PIN_RESET); // Reset LOW halten
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 4800;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); //https://blog.the78mole.de/stm32-uart-continuous-receive-with-interrupt/

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 4800;
  huart3.Init.WordLength = UART_WORDLENGTH_9B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|CS_SPI_SI46xx_Pin|Si46xx_RSTB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin CS_SPI_SI46xx_Pin Si46xx_RSTB_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|CS_SPI_SI46xx_Pin|Si46xx_RSTB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Si46xx_INTB_Pin */
  GPIO_InitStruct.Pin = Si46xx_INTB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Si46xx_INTB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



// This is called when SPI receive is done
/*void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef * hspi)
{
  // Set CS pin to high and raise flag
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  spi_recv_flag = 1;
}*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
