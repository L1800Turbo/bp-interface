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

enum Si46xx_States
{
	Si46xx_STATE_SAFE_OFF = 0,		/* Safe off state: something serious happened, don't leave this state */
	Si46xx_STATE_STARTUP,
	Si46xx_STATE_INIT,
	Si46xx_STATE_LOAD_FIRMWARE,
	Si46xx_STATE_BOOT,

	Si46xx_STATE_ANSWER,			/* Generic state where the answer gets analyzed */
};

enum Si46xx_commands {
	/* Boot commands */
	SI46XX_RD_REPLY      = 0x00,	/* Returns the status byte and data for the last command sent to the device. 			*/
	SI46XX_POWER_UP      = 0x01,	/* Power-up the device and set system settings. 										*/
	SI46XX_HOST_LOAD     = 0x04,	/* Loads an image from the HOST over the command interface 								*/
	SI46XX_FLASH_LOAD    = 0x05, 	/* Loads an image from external FLASH over secondary SPI bus 							*/
	SI46XX_LOAD_INIT     = 0x06,	/* Prepares the bootloader to receive a new image. 										*/
	SI46XX_BOOT          = 0x07,	/* Boots the image currently loaded in RAM. 											*/
	SI46XX_GET_PART_INFO = 0x08,	/* Reports basic information about the device. 											*/
	SI46XX_GET_SYS_STATE = 0x09,	/* Reports system state information. 													*/
	SI46XX_GET_POWER_UP_ARGS = 0x0A /* Reports basic information about the device such as arguments used during POWER_UP. 	*/
};

typedef enum {
	Si46xx_OK = 0,
	Si46xx_TIMEOUT
	// TODO usw
} Si46xx_Error;

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

/*enum Si46xx_InterruptFlag {
	Si46xx_INT_FLAG_UNSET = 0,
	Si46xx_INT_FLAG_SET
};*/



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

struct Si46xx_Status_Values {
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
};

struct Si46xx_Config {
	struct Si46xx_Init_Values initConfig;
	SPI_HandleTypeDef * hspi;

	struct Si46xx_Status_Values currentStatus;

	enum Si46xx_States stateBefore;
	enum Si46xx_States state;
	enum Si46xx_States stateAfter;	/* In case a following state is used (after answer) */

	uint8_t answerBytes;			/* How many bytes should the answer contain? Depends on the preceding CMD, min 4 bytes */
	uint32_t timeoutVal;

	//enum Si46xx_InterruptFlag interruptFlag;
	uint8_t intstate;

};

Si46xx_Error Si46xx_InitConfiguration(SPI_HandleTypeDef * hspi);
void Si46xx_Send_Reset(void);
void Si46xx_Tasks(void);


#endif /* INC_SI46XX_H_ */
