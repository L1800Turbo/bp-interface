/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */

#include "cdc_interface.h"

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//uint8_t msg = 0;
//uint16_t cdcData[20];

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* Ring buffer definitions -------------------------------------------- */

 typedef struct
 {
	 uint8_t* buf;

	 uint16_t wr;	// Write index
	 uint16_t rd;	// Read index
	 uint16_t lb;	// Overflow index

	 enum bufferStatus_en // Enum to help with reactivating the USB connection after a full buffer
	 {
		 BUFFER_FREE = 0,	// Space on the buffer
		 BUFFER_FULL
	 }bufferState;

	 //uint32_t tmpCnt; // TODO für tests
 }cdc_ringbuf_dt;

 void cdc_ringbuf_init(cdc_ringbuf_dt * rb, uint8_t * buf);
 void cdc_ringbuf_clear(cdc_ringbuf_dt * rb);
 uint16_t cdc_ringbuf_length(cdc_ringbuf_dt * rb);

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
#define RX_BUFFER_MAX_WRITE_INDEX (APP_RX_DATA_SIZE - CDC_DATA_FS_MAX_PACKET_SIZE)

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

cdc_ringbuf_dt cdc_rx_rb;
cdc_ringbuf_dt cdc_tx_rb;

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
	extern cdc_ringbuf_dt cdc_rx_rb;

	cdc_ringbuf_init(&cdc_rx_rb, UserRxBufferFS);
	cdc_ringbuf_init(&cdc_tx_rb, UserTxBufferFS);

  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */

	// Update the write index for the next incoming packet
	// This function would only be called if there's enough space in the buffer
	cdc_rx_rb.wr += *Len;

	//cdc_rx_rb.tmpCnt+=*Len; // TODO nur für Tests zählen

	// Is the new value too close to the end of the FIFO ?
	if (cdc_rx_rb.wr >= RX_BUFFER_MAX_WRITE_INDEX )
	{
		// wrap-around (and save wr as lb)
		cdc_rx_rb.lb = cdc_rx_rb.wr;
		cdc_rx_rb.wr = 0;
	}

	// Tell the driver where to write the next incoming packet
	// Set pointer to next write index
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_rb.buf + cdc_rx_rb.wr);
	// Receive the next packet
	uint32_t bufSpace = RX_BUFFER_MAX_WRITE_INDEX - 1 - cdc_ringbuf_length(&cdc_rx_rb);
	if(bufSpace > CDC_DATA_FS_MAX_PACKET_SIZE) // TODO: war 0, kann aber nicht
	{
		cdc_rx_rb.bufferState = BUFFER_FREE;
		USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	}
	else
	{
		cdc_rx_rb.bufferState = BUFFER_FULL;
	}
	return (USBD_OK);

/*

	extern bp_msg_state_dt bpMsgState;
	extern enum bp_comm_state bpCommState;

	//extern void Si46xx_ReceiveFW(uint8_t * buffer, uint32_t length);
	extern void Si46xx_Boot(void);
	extern void Si46xx_TestFunctions(uint8_t param);

	//if(Si46xxCfg.state == Si46xx_STATE_LOAD_FIRMWARE || Si46xxCfg.state == Si46xx_STATE_LOAD_FIRMWARE_WAIT)
	//if(Si46xxCfg.firmwareBuf->fwStep != FW_NONE)


	if(Buf[0] == 'b')
	{
		Si46xx_Boot();
	}
	else
	{
		Si46xx_TestFunctions(Buf[0]-0x30);
	}*/


	/*if(0)
	{
		//Si46xx_TransmitFW(Buf, *Len); // TODO: aktuell Blocking
		//if(Si46xx_GetFW(Buf, *Len) == FWBUF_SIZE)
		{
			return USBD_BUSY;
		}
	}
	else
	{
		CDC_Transmit_FS(Buf, *Len);

		for(uint32_t i=0; i<*Len; i++)
		{
			if(Buf[i] == 's') // Startbyte für SPI-Geschichten
			{
			//	Si46xx_Boot();
				//Si46xx_Send_Reset();
			}
			else if(Buf[i] == ' ') // Tabulator 0x09 kann putty über CopyPaste nicht
			{
				msg++;
				//HAL_GPIO_TogglePin(LED_ORANGE_Port, LED_ORANGE_Pin);
			}
			else if(Buf[i] == '\r')
			{
				msg++;
				HAL_GPIO_TogglePin(LED_GREEN_Port, LED_GREEN_Pin);
				//printf("Plu: %X", cdcData[0]);
				//CDC_Transmit_FS((uint8_t*)cdcData, msg);

				uint8_t data[8] = {0,};
				if(cdcData[1] > 8)
				{
					cdcData[1] = 8;
				}

				for(uint8_t j=0; j<cdcData[1]; j++)
				{
					data[j] = (uint8_t) cdcData[3+j];
				}

				//memcpy(data, &cdcData[3], cdcData[1]*sizeof(uint8_t)); geht nicht, er müsste das ja immer abtrennen

				ringAdd(bpMsgState.writeBuf, buildMessage(cdcData[0], cdcData[1], cdcData[2], data, 10));
				bpCommState = BP_SEND_WAIT;

				memset(cdcData, 0, (msg*sizeof(uint16_t)));
				msg = 0;
			}
			else
			{
				uint8_t currNo = 0xF;

				// TODO nur 0-9A-F akzeptieren...
				if(Buf[i] >= '0' && Buf[i] <= '9')
				{
					currNo = Buf[i] - 0x30;
				}
				else if(Buf[i] >= 'A' && Buf[i] <= 'F')
				{
					currNo = Buf[i] - 0x41 + 10;
				}

				cdcData[msg] = (cdcData[msg]<<4) + currNo;
			}
		}
	}*/

  /*USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);*/
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, size_t Len)
{
	uint8_t result = USBD_OK;
	/* USER CODE BEGIN 7 */
	// aus: https://nefastor.com/microcontrollers/stm32/usb/stm32cube-usb-device-library/communication-device-class/

	//Step 1 : calculate the occupied space in the Tx FIFO
	int16_t cap = cdc_tx_rb.wr - cdc_tx_rb.rd;   // occupied capacity
	if (cap < 0)    // FIFO contents wrap around
	{
		cap += APP_TX_DATA_SIZE;
	}

	cap = APP_TX_DATA_SIZE - cap;      // available capacity

	// Step 2 : compare with argument
	if (cap < Len)
	{
		return USBD_BUSY;   // Not enough room to copy "buf" into the FIFO => error TODO: ist es dann nicht zu spät?
	}

	// Step 3 : does buf fit in the tail ?
	int16_t tail = APP_TX_DATA_SIZE - cdc_tx_rb.wr;
	if (tail >= Len)
	{
		// Copy buf into the tail of the FIFO
		memcpy (&cdc_tx_rb.buf[cdc_tx_rb.wr], Buf, Len);

		// Update "wr" index
		cdc_tx_rb.wr += Len;

		// In case "len" == "tail", next write goes to the head
		if (cdc_tx_rb.wr == APP_TX_DATA_SIZE)
		{
			cdc_tx_rb.wr = 0;
		}
	}
	else
	{
		// Copy the head of "buf" to the tail of the FIFO
		memcpy (&cdc_tx_rb.buf[cdc_tx_rb.wr], Buf, tail);

		// Copy the tail of "buf" to the head of the FIFO :
		memcpy (cdc_tx_rb.buf, &Buf[tail], Len - tail);

		// Update the "wr" index
		cdc_tx_rb.wr = Len - tail;
	}

  /* Original STM32 HAL
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);*/
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */

  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
 * Initialize a ring buffer
 *
 */
void cdc_ringbuf_init(cdc_ringbuf_dt * rb, uint8_t * buf)
{
	rb->buf = buf;
	rb->wr = 0;
	rb->rd = 0;
	rb->lb = 0;

	cdc_ringbuf_clear(rb);
}

void cdc_ringbuf_clear(cdc_ringbuf_dt * rb)
{
	rb->rd = rb->wr;
}

/**
 * Get lenth of given fifo
 */
uint16_t cdc_ringbuf_length(cdc_ringbuf_dt * rb)
{
	int32_t len = rb->wr - rb->rd;

	// If there was a wrap around and write is "in front": add the point where was wrapped
	if(len < 0)
	{
		len += rb->lb;
	}

	return len;
}

void cdc_Ringbuf_Tasks(void)
{
	/* RX Tasks --------------------------------*/
	// Check if data on buffer, reactivate USB receive if buffer isn't full
	if(cdc_ringbuf_length(&cdc_rx_rb) > 0)
	{

		// If the buffer was marked as full before
		if(cdc_rx_rb.bufferState == BUFFER_FULL)
		{
			// Is the buffer free in the meantime? TODO Kopie aus RX
			uint32_t bufSpace = RX_BUFFER_MAX_WRITE_INDEX - 1 - cdc_ringbuf_length(&cdc_rx_rb);
			if(bufSpace > CDC_DATA_FS_MAX_PACKET_SIZE)
			//if(RX_BUFFER_MAX_WRITE_INDEX - 1 - cdc_ringbuf_length(&cdc_rx_rb) > 0)
			{
				cdc_rx_rb.bufferState = BUFFER_FREE;

				// Receive the next packet
				USBD_CDC_ReceivePacket(&hUsbDeviceFS);
			}
			else // TODO nur für Tests
				HAL_GPIO_WritePin(LED_RED_Port, LED_RED_Pin, GPIO_PIN_SET);
		}
	}

	/* TX Tasks --------------------------------*/
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

	// Test if the USB CDC is ready to transmit
	if (hcdc->TxState == 0)
	{
		// Update the FIFO to reflect the completion of the last transmission
		cdc_tx_rb.rd = cdc_tx_rb.lb;

		// Compute how much data is in the FIFO
		int16_t cap = cdc_tx_rb.wr - cdc_tx_rb.rd;
		if (cap != 0)  // The FIFO is empty : return immediately
		{
			if (cap < 0)  // The FIFO contents wrap-around
			{
				// Send only the tail of the FIFO
				USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &cdc_tx_rb.buf[cdc_tx_rb.rd], APP_TX_DATA_SIZE - cdc_tx_rb.rd);
				USBD_CDC_TransmitPacket(&hUsbDeviceFS);
				cdc_tx_rb.lb = 0;    // Lock the tail’s data
			}
			else  // No wrap-around : send the whole FIFO
			{
				USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &cdc_tx_rb.buf[cdc_tx_rb.rd], cap);
				USBD_CDC_TransmitPacket(&hUsbDeviceFS);
				cdc_tx_rb.lb = cdc_tx_rb.wr; // lock the data
			}
		}
	}
}

/**
 * Get the current length of the RX buffer
 */
size_t cdc_ringbuf_Rx_getLength(void)
{
	return cdc_ringbuf_length(&cdc_rx_rb);
}

/**
 * Get data from the buffer
 * len: Param how many bytes schould be received, also return value how many were available/taken from the buffer
 */
uint8_t cdc_ringbufRx_get(uint8_t * buf, size_t * Len)
{
	size_t l_len = cdc_ringbuf_length(&cdc_rx_rb);

	// Limit to maximum size available or wanted
	if(*Len > l_len) // TODO: passt das mit den Sternchen?
	{
		*Len = l_len;
	}
	else
	{
		l_len = *Len;
	}

	while(l_len)
	{
		l_len--;

		// Copy through buffer
		*buf++ = cdc_rx_rb.buf[cdc_rx_rb.rd++];

		// Reset read pointer when limit reached
		if(cdc_rx_rb.rd == cdc_rx_rb.lb)
		{
			cdc_rx_rb.rd = 0;
		}
	}

	return USBD_OK;
}

void cdc_ringbufRx_clear(void)
{
	cdc_ringbuf_clear(&cdc_rx_rb);
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
