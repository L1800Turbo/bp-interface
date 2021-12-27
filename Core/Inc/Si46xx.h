/*
 * Si46xx.h
 *
 *
 *
 *  Created on: 21.03.2021
 *      Author: kai
 */

#ifndef INC_SI46XX_H_
#define INC_SI46XX_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "circular_buffer.h"

#define SI46XX_DEFAULT_SPI_WAIT 10 // ms, when looping and polling for an SPI ready state from uC or Si46xx

#define SI46XX_RST_ON()  HAL_GPIO_WritePin(Si46xx_RSTB_GPIO_Port, Si46xx_RSTB_Pin, GPIO_PIN_RESET); // Reset LOW
#define SI46XX_RST_OFF() HAL_GPIO_WritePin(Si46xx_RSTB_GPIO_Port, Si46xx_RSTB_Pin, GPIO_PIN_SET);

#define SI46XX_CS_ON()  HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_RESET)
#define SI46XX_CS_OFF() HAL_GPIO_WritePin(CS_SPI_SI46xx_GPIO_Port, CS_SPI_SI46xx_Pin, GPIO_PIN_SET)

// Enum for possible SPI commands
enum Si46xx_SPI_commands {
	/* Boot commands */
	SI46XX_RD_REPLY      = 0x00,	/* Returns the status byte and data for the last command sent to the device. 			*/
	SI46XX_POWER_UP      = 0x01,	/* Power-up the device and set system settings. 										*/
	SI46XX_HOST_LOAD     = 0x04,	/* Loads an image from the HOST over the command interface 								*/
	SI46XX_FLASH_LOAD    = 0x05, 	/* Loads an image from external FLASH over secondary SPI bus 							*/
	SI46XX_LOAD_INIT     = 0x06,	/* Prepares the bootloader to receive a new image. 										*/
	SI46XX_BOOT          = 0x07,	/* Boots the image currently loaded in RAM. 											*/
	SI46XX_GET_PART_INFO = 0x08,	/* Reports basic information about the device. 											*/
	SI46XX_GET_SYS_STATE = 0x09,	/* Reports system state information. 													*/
	SI46XX_GET_POWER_UP_ARGS = 0x0A,/* Reports basic information about the device such as arguments used during POWER_UP. 	*/

	SI46XX_SET_PROPERTY  = 0x13,	/* Sets the value of a property.														*/
};

enum Si46xx_CTS {
	Si46xx_CTS_NOT_READY = 0,
	Si46xx_CTS_READY
};

enum Si46xx_ERR_REPLY {
	Si46xx_ERR_NORMAL = 0, 	/* No api error has occurred. */
	Si46xx_ERR_ERROR 		/* Bad command, see reply byte 4 for details */

	// TODO: Byte 4 auslesen bei Error...
};

enum Si46xx_PUP_STATE {
	Si46xx_PUP_RESET = 0, 	/* The system has been reset but no POWER_UP command has been issued. The system is currently waiting on the POWER_UP command.*/
	Si46xx_PUP_RESET_HOT, 	/* Reserved */
	Si46xx_PUP_BL, 		 	/* The bootloader is currently running */
	Si46xx_PUP_APP 		 	/* An application was successfully booted and is currently running. */
};

typedef struct
{
	/* Clear-to-send (bit 7 of the STATUS field which is read by the user) Indicates whether the user may send a new firmware-interpreted command and
	 * retrieve a reply from the previous command. It has no effect for hardware-interpreted commands. When a new read or write transaction is started
	 * on the serial port, the current state of CTS is captured into a temporary register, which is considered to be the state of CTS for the duration
	 * of that serial port transaction. The captured value will govern the serial port behavior and is returned to the user in the STATUS byte on reads.
	 * Therefore, any changes made to the state of CTS by the processor will not affect a serial port transaction that is currently in progress.
	 */
	enum Si46xx_CTS CTS;
	enum Si46xx_ERR_REPLY ERR_CMD;
	enum Si46xx_PUP_STATE PUP; /* Indicates the powerup state of the system. */

	enum Si46xx_ERR_REPLY RFFE_ERR; /* When set indicates that the RF front end of the system is in an unexpected state. */
	enum Si46xx_ERR_REPLY REPOFERR; /* When set the control interface has dropped data during a reply read, which is a fatal error.
									   This is generally caused by running at a SPI clock rate that is too fast for the given data arbiter and memory speed. */
	enum Si46xx_ERR_REPLY CMDOFERR; /* When set the control interface has dropped data during a command write, which is a fatal error.
									   This is generally caused by running at a SPI clock rate that is too fast for the data arbiter and memory speed. */
	enum Si46xx_ERR_REPLY ARBERR;	/* When set an arbiter error has occurred. */
	enum Si46xx_ERR_REPLY ERRNR; /* When set a non-recoverable error has occurred. The system keep alive timer has expired. */
}Si46xx_Status_Values_dt;

enum Si46xx_Switch {
	Si46xx_DISABLE = 0,
	Si46xx_ENABLE  = 1
};

enum Si46xx_ClockMode_Config {
	Si46xx_OFF 	    = 0x0, 	/* Oscillator and buffer are powered down. 						*/
	Si46xx_XOSC 	= 0x1, 	/* Reference clock generator is in crystal mode. 				*/
	Si46xx_SING_BF 	= 0x2, 	/* Oscillator is off and circuit acts as single ended buffer. 	*/
	Si46xx_DIFF_BF 	= 0x3 	/* Oscillator is off and circuit acts as differential buffer. 	*/
};

/* Possible states for SPI command GET_SYS_STATE */
enum Si46xx_Image {
	Si46xx_BL   				= 0x0, 	/* Bootloader is active 					*/
	Si46xx_FMHD					= 0x1, 	/* FMHD is active 							*/
	Si46xx_DAB					= 0x2, 	/* DAB is active 							*/
	Si46xx_TDMB_DAB_DATA_ONLY 	= 0x3, 	/* TDMB or data only DAB image is active	*/
	Si46xx_FMHD_DEMOD			= 0x4, 	/* FMHD demod is active 					*/
	Si46xx_AMHD					= 0x5, 	/* AMHD is active 							*/
	Si46xx_AMHD_DEMOD			= 0x6, 	/* AMHD demod is active 					*/
	Si46xx_DAB_DEMOD			= 0x7 	/* DAB demod is active 						*/
};

struct Si46xx_Init_Values {
	enum Si46xx_Switch CTS_InterruptEnable; /* The bootloader will toggle a host interrupt line when CTS is available. */
    enum Si46xx_ClockMode_Config CLK_MODE; /* Choose clock mode. See refclk spec sheet for more information           */
    uint8_t XOSC_TR_SIZE;	/* XOSC TR_SIZE. See refclk spec sheet for more information.				  */
    uint8_t IBIAS;     		/* XTAL IBIAS current at startup. See refclk spec sheet for more information.
    							This parameter is only required if using the crystal oscillator.
    							10 uA steps, 0 to 1270 uA. */
    uint8_t IBIAS_RUN;		/* XTAL IBIAS current at runtime, after the XTAL oscillator has stabalized. See refclk spec sheet for more information.
        						This parameter is only required if using the crystal oscillator. 10 uA steps, 10 to 1270 uA.
        						If set to 0, will use the same value as IBIAS. */
    uint32_t XTAL_Freq;		/* XTAL Frequency in Hz. The supported crystal frequencies are:
								[5.4 MHz - 6.6 MHz], [10.8 MHz - 13.2 MHz], [16.8 MHz - 19.8 MHz], [21.6 MHz - 26.4 MHz], [27 MHz - 46.2 MHz]. */
    uint8_t CTUN;			/* CTUN. See refclk spec sheet for more information.
    							This parameter is only required if using the crystal oscillator. */
};

typedef enum
{
	Si46xx_STATE_IDLE = 0,
	Si46xx_STATE_BOOTING,
	Si46xx_STATE_BUSY
}Si46xx_state_en;


typedef enum
{
	SI46XX_MSG_NONE = 0,
	SI46XX_MSG_REFRESH_SYS_STATE,

	SI46XX_MSG_SIZE
}Si46xx_msg_en;

enum Si46xx_ISR_state_en {
	ISR_SET = 0,
	ISR_UNSET
};

typedef struct
{
	//enum Si46xx_SPI_commands cmd;
	HAL_StatusTypeDef (*sendFunc)();
	HAL_StatusTypeDef (*receiveFunc)();

}Si46xx_msg_dt;

struct Si46xx_Config
{
	/* SPI handler */
	SPI_HandleTypeDef * hspi;

	/* Initial configuration */
	struct Si46xx_Init_Values initConfig;

	/* Device status, updated with each SPI status command */
	Si46xx_Status_Values_dt deviceStatus;

	/* State of Zustandsautomat */
	Si46xx_state_en state;

	/* Waiting routines */
	uint32_t waitStamp;
	uint32_t waitTime;

	/* State of ISR trigger */
	enum Si46xx_ISR_state_en isrState;

	/* Ring buffer for messages */
	circular_buffer cb;

	enum Si46xx_Image image;
};

enum Si46xx_Wait_en {
	TIME_LEFT = 0,
	TIME_OVER
};

// Bootloader patch
const uint8_t Si46xx_Rom00Patch016[5796];
//const uint8_t Si46xx_Firmware[499760]; // BIF
const uint8_t Si46xx_Firmware[499356]; // BIN


void Si46xx_Tasks(void);

HAL_StatusTypeDef Si46xx_InitConfiguration(SPI_HandleTypeDef * hspi);
//Si46xx_Status_en Si46xx_getStatus(uint16_t answerBytes, uint8_t * returnDataPtr);
HAL_StatusTypeDef Si46xx_SPIgetStatus(SPI_HandleTypeDef * hspi, uint8_t * data, uint16_t len);
HAL_StatusTypeDef Si46xx_SPIsend(SPI_HandleTypeDef * hspi, uint8_t * data, uint16_t len);

void progress_StatusBytes(Si46xx_Status_Values_dt * status, uint8_t * data);

void Si46xx_SetWaitTime(uint32_t ms);
uint32_t Si46xx_CurrentWaitTime(void);
uint8_t Si46xx_RemainingTimeLeft(void);

#endif /* INC_SI46XX_H_ */
