/*
 * Si46xx_data_types.h
 *
 *  Created on: Dec 31, 2021
 *      Author: kai
 */

#ifndef INC_SI46XX_DATA_TYPES_H_
#define INC_SI46XX_DATA_TYPES_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "Si46xx_DAB_frequencies.h"
#include "circular_buffer.h"

/* SPI states ---------------------------------------------------------------------------- */

enum Si46xx_CTS {
	Si46xx_CTS_NOT_READY = 0,
	Si46xx_CTS_READY
};

enum Si46xx_STCINT {
	Si46xx_STCINT_INCOMPLETE = 0x0,	/* Tune complete has not been triggered. Do not send a new TUNE/SEEK command.    */
	Si46xx_STCINT_COMPLETE   = 0x1	/* Tune complete has been triggered. It is safe to send a new TUNE/SEEK command. */
};

enum Si46xx_ERR_REPLY {
	Si46xx_ERR_NORMAL = 0, 	/* No api error has occurred. */
	Si46xx_ERR_ERROR 		/* Bad command, see reply byte 4 for details */

	// TODO: Byte 4 auslesen bei Error...
};

enum Si46xx_ISR_state_en {
	ISR_SET = 0,
	ISR_UNSET
};

enum Si46xx_Switch {
	Si46xx_DISABLE = 0,
	Si46xx_ENABLE  = 1
};

// Enum for state machine
typedef enum
{
	Si46xx_STATE_IDLE = 0,
	Si46xx_STATE_BOOTING,
	Si46xx_STATE_BUSY
}Si46xx_state_en;

enum Si46xx_PUP_STATE {
	Si46xx_PUP_RESET = 0, 	/* The system has been reset but no POWER_UP command has been issued. The system is currently waiting on the POWER_UP command.*/
	Si46xx_PUP_RESET_HOT, 	/* Reserved */
	Si46xx_PUP_BL, 		 	/* The bootloader is currently running */
	Si46xx_PUP_APP 		 	/* An application was successfully booted and is currently running. */
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

typedef struct
{
	/* Clear-to-send (bit 7 of the STATUS field which is read by the user) Indicates whether the user may send a new firmware-interpreted command and
	 * retrieve a reply from the previous command. It has no effect for hardware-interpreted commands. When a new read or write transaction is started
	 * on the serial port, the current state of CTS is captured into a temporary register, which is considered to be the state of CTS for the duration
	 * of that serial port transaction. The captured value will govern the serial port behavior and is returned to the user in the STATUS byte on reads.
	 * Therefore, any changes made to the state of CTS by the processor will not affect a serial port transaction that is currently in progress.
	 */
	enum Si46xx_CTS CTS;
	enum Si46xx_STCINT STCINT;		/* Seek/Tune Complete */

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

/* DAB states ---------------------------------------------------------------------------- */

typedef struct // Struct for a frequency and channel
{
	enum DAB_frequencies channel;	// Channel with the according frequency
}dab_channel_dt;

// TODO hier dann service kram und so hin


/* Config enums -------------------------------------------------------------------------- */

/* Possible sources for firmware */
typedef enum
{
	FW_SRC_UC = 0,	/* Firmware on uC to be transferred to device 			*/
	FW_SRC_USB,		/* Firmware on connected PC to be transferred by USB 	*/
	FW_SRC_FLASH,	/* Firmware on flash connected to Si46xx				*/

	FW_SRC_size
}fw_source_dt;


/* Config structures---------------------------------------------------------------------- */

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

	/* Values for DAB mode */
	enum Si46xx_Image image;

	/* Frequency parameters */
	enum Si46xx_frequencyList_status {FREQ_LIST_INVALID, FREQ_LIST_VALID} freqencyListStatus;
	enum DAB_frequencies freqIndex;

	struct wantedService
	{
		uint32_t serviceID;
		uint32_t componentID;
	}wantedService;

};

#endif /* INC_SI46XX_DATA_TYPES_H_ */
