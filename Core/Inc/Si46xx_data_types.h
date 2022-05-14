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

#define MAX_SERVICES 50
#define MAX_COMPONENTS 15

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

enum Si46xx_INT {
	Si46xx_NORMAL = 0,
	Si46xx_INTERRUPT
};

enum Si46xx_ISR_state_en {
	ISR_SET = 0,
	ISR_UNSET,
	ISR_INACTIVE, // for functions without interrupt pin (before patching interrupt doesn't work)
	ISR_FIRST_FLAG // after bootloader patch, ISR rises; from now on using ISRs should be possible, neithertheless, the current function must be finished
};

enum Si46xx_Switch {
	Si46xx_DISABLE = 0,
	Si46xx_ENABLE  = 1
};

// Enum for state machine
typedef enum
{
	Si46xx_STATE_IDLE = 0,
	Si46xx_STATE_SENDING,
	Si46xx_STATE_RESET,
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

/* Properties */
enum properties
{
	//INT_CTL (0x00)
 	INT_CTL_ENABLE = 0x0000,
 	INT_CTL_REPEAT = 0x0001,

	// DIGITAL_SERVICE (0x81)
	DIGITAL_SERVICE_INT_SOURCE    = 0x8100,
	DIGITAL_SERVICE_RESTART_DELAY = 0x8101,

	// DAB_EVENT (0xB3)
	DAB_EVENT_INTERRUPT_SOURCE    = 0xB300, /* Configures which dab events will set the DEVENTINT status bit. 	         */
	DAB_EVENT_MIN_SVRLIST_PERIOD  = 0xB301, /* Configures how often service list notifications can occur. 	             */
	DAB_EVENT_MIN_FREQINFO_PERIOD = 0xB303, /* Configures how often frequency information notifications can occur. 	     */
	DAB_EVENT_AUDIO_DEMUTE        = 0xB304, /* DAB_EVENT_AUDIO_DEMUTE_DELAY configures the audio demute interrupt delay. */

	// DAB (0xB4)
	DAB_XPAD_ENABLE = 0xB400,
	DAB_DRC_OPTION  = 0xB401,
};

enum INT_CTL_ENABLE_flags
{
	INT_CTL_DFICIEN    = (1 << 14),
	INT_CTL_DEVNTIEN   = (1 << 13),
	INT_CTL_CTSIEN     = (1 <<  7), // Interrupt when CTS is set
	INT_CTL_ERR_CMDIEN = (1 <<  6),
	INT_CTL_DACQIEN    = (1 <<  5), // Interrupt when DACQINT is set
	INT_CTL_DSRVIEN    = (1 <<  4),	// Interrupt when DSRVINT is set
	INT_CTL_RSQIEN     = (1 <<  3),
	INT_CTL_ACFIEN     = (1 <<  1),
	INT_CTL_STCIEN     = (1 <<  0)  // Interrupt when STCINT is set
};

enum DAB_EVENT_INTERRUPT_SOURCE_flags
{
	RECFG_INTEN    = (1 << 7),
	RECFGWRN_INTEN = (1 << 6),
	AUDIO_INTEN    = (1 << 5),
	ANNO_INTEN     = (1 << 4),
	OESERV_INTEN   = (1 << 3),
	SERVLINK_INTEN = (1 << 2),
	FREQINFO_INTEN = (1 << 1),
	SRVLIST_INTEN  = (1 << 0)
};

enum DAB_XPAD_ENABLE_flags
{
	ALL_OTHER_ENABLE  = (1 << 15), // Enables all other user application types.
	UNKNOWN_ENABLE    = (1 << 14), // Enables unknown type. If FIG 0/13 is missing, User application type is unknown..
	JOURNALINE_ENABLE = (1 << 13), // Enables journaline.
	MIDDLEWARE_ENABLE = (1 << 12), // Enables middleware.
	VOICE_ENABLE      = (1 << 11), // Enables voice applications.
	IPDC_ENABLE       = (1 << 10), // Enables IPDC services.
	DMB_ENABLE        = (1 <<  9), // Enables DMB.
	DABJAVA_ENABLE    = (1 <<  8), // Enables DAB JAVA.
	EPG_ENABLE        = (1 <<  7), // Enables EPG.
	TMC_ENABLE        = (1 <<  6), // Enables TMC.
	DGPS_ENABLE       = (1 <<  5), // Enables DGPS.
	TPEG_ENABLE       = (1 <<  4), // Enables TPEG.
	MOT_BWS_ENABLE    = (1 <<  3), // Enables MOT Broadcaset Web Site.
	MOT_SLS_ENABLE    = (1 <<  2), // Enables MOT slideshow.
	DLS_ENABLE        = (1 <<  0)  // Enables PAD delivered DLS packets. (default)
};

struct property
{
	enum properties index;
	uint16_t data;
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

	enum Si46xx_STCINT STC;		/* Seek/Tune Complete */

	/* DIGRAD status values */
	enum Si46xx_VALID_en
	{
		NOT_VALID = 0,
		IS_VALID  = 1
	}VALID; /* When set to 1, the RSSI is at or above the valid threshold. It is recommended that the valid bit be used as part of tune validation.
				Once STC is set the valid bit can be checked to verify that then tune has passed both the RSSI valid threshold and that acquisition
				has been achived. The host should set the RSSI thershold, validation time and acquisition time to achieve solid tune time performance.
				Doing this helps insure an accurate tune indication and helps to decrease scan times due to quick station disqualification. */

	enum Si46xx_ACQ_en
	{
		NO_ACQ = 0,
		IS_ACQ = 1
	}ACQ; /* When set to 1 the ensemble is acquired. */

	enum Si46xx_FICERR_en
	{
		NO_FICERR    = 0,
		FICERR_ERROR = 1  // Unrecoverable errors
	}FICERR; /* When set to 1 the ensemble is experiencing FIC errors. Signal quality has been degraded and acquisition may be lost. */

	int8_t RSSI; // Received signal strength indicator in dBuV.
	int8_t SNR;  // Indicates the current estimate of the digital SNR in dB.
	uint8_t FIC_QUALITY; // Indicates the current estimate of the ensemble's FIC quality. The number provided is between 0 and 100.
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

// Enum for SPI status of current function
typedef enum
{
	Si46xx_OK = 0,
	Si46xx_BUSY,
	Si46xx_SPI_ERROR,
	Si46xx_MESSAGE_ERROR,
	Si46xx_DEVICE_ERROR
}Si46xx_statusType;

/* DAB states ---------------------------------------------------------------------------- */

typedef struct
{
	uint16_t componentID;
	uint8_t tmID;

	uint8_t ascTy_dscTy;	// ASCTy: Audio service component type, DSCTy: Data service component type TODO: enum

}dab_component_t;

typedef struct
{
	uint32_t serviceID;
	enum program_data_flag
	{
		AUDIO_SERVICE = 0,
		DATA_SERVICE  = 1
	}pdFlag;
	//uint8_t pdFlag;
	uint8_t numberComponents;
	char serviceLabel[16+1];

	dab_component_t components[MAX_COMPONENTS];
}dab_service_t;

typedef struct // Struct for a frequency and channel
{
	enum DAB_frequencies channel;	// Channel with the according frequency

	uint16_t listSize;
	uint8_t numServices;
	uint16_t version;

	uint16_t ensembleID;
	char ensembleLabel[16];

	dab_service_t services[MAX_SERVICES];
}dab_channel_dt;



/* Config enums -------------------------------------------------------------------------- */

/* Possible sources for firmware */
enum fw_source
{
	FW_SRC_UC = 0,	/* Firmware on uC to be transferred to device 			*/
	FW_SRC_USB,		/* Firmware on connected PC to be transferred by USB 	*/
	FW_SRC_FLASH,	/* Firmware on flash connected to Si46xx				*/

	FW_SRC_size
};

typedef enum
{
	USB_FW_NONE = 0,
	USB_FW_WAITING,		// If we wait for the first CDC response
	USB_FW_TRANSFERRED, // Show if CDC has a block transferred
	USB_FW_BUSY // in case the buffer is being used and shouldn't be overwritten by CDC
}usb_fw_en;

typedef struct
{
	enum step_en { // TODO: auch noch weg
		FW_NONE = 0,
		FW_BOOTLOADER_PATCH,
		FW_FIRMWARE,

		FW_size
	}step;

	// Configuration where the files are located
	//fw_source_dt fw_source[FW_size];	// TODO: Das wo anders hin, diesen ganzen Struct in firmware mitnehmen

	/*enum fwTransfer // Indicator to wait for transfer being finished in multiple steps
	{
		SPI_FW_IDLE = 0,
		SPI_FW_BUSY
	}fw_spi_busy;*/
	uint32_t current_flash_address; // If external flash is used: the current address to use

	// Which one is the current firmware source?
	enum fw_source current_fw_source;

	enum fw_dst
	{
		FW_DST_SI46XX_RAM = 0, // Write to RAM of Si46xx
		FW_DST_FLASH,		   // Write to flash by Si46xx
		FW_DST_SPI_FLASH	   // Write to flash by direct SPI communication
	}current_fw_destination;

	usb_fw_en usbFw_wanted;

	uint8_t * fwBufPtr;
	uint32_t fwBufSize;
}Si46xx_firmware_dt;


/* Config structures---------------------------------------------------------------------- */

struct Si46xx_Config
{
	/* SPI handler */
	SPI_HandleTypeDef * hspi;

	/* Initial configuration */
	struct Si46xx_Init_Values initConfig;

	/* Version information about the SI46XX device */
	struct device_information_dt
	{
		uint8_t chipRevision;
		uint8_t romID;
		uint16_t partNumber;

		uint8_t softwareRevision[3]; // Version with subrevision
	}deviceInformation;

	/* Device status, updated with each SPI status command */
	Si46xx_Status_Values_dt deviceStatus;

	/* Analyzed status after a received message */
	Si46xx_statusType analyzedStatus;

	/* Current property for functions TODO: Umplatzieren zu functions? */
	struct property currentProperty;

	/* State of generic radio state machine */ // TODO hier raus, in si46xx
	enum radio_states_en
	{
		Si46xx_Radio_Idle = 0,
		Si46xx_Radio_Start,
		Si46xx_Radio_Start_Wait,
		Si46xx_Radio_Patch,
		Si46xx_Radio_Patch_Wait,
		Si46xx_Radio_FlashFirmware,
		Si46xx_Radio_FlashFirmware_Wait,
		Si46xx_Radio_Boot,
		Si46xx_Radio_Boot_Wait,
		Si46xx_Radio_Properties,
		Si46xx_Radio_Properties_Wait,
		Si46xx_Radio_Config,
		Si46xx_Radio_Config_Wait,
		// Konfiguration wie Senderlisten hier...

	}radio_states;

	/* State of SPI function state machine */
	Si46xx_state_en function_state;

	/* Waiting routines */
	uint32_t waitStamp;
	uint32_t waitTime;

	/* State of ISR trigger */
	enum Si46xx_ISR_state_en isrState;

	/* Ring buffer for messages */
	circular_buffer cb;

	/* firmware information */
	Si46xx_firmware_dt firmware;
	enum fw_source firmware_source;  /* Source for the current main firmware      */
	uint32_t firmware_flash_address; /* Default flash address containing firmware */

	/* Values for DAB mode */
	enum Si46xx_Image image;

	/* Frequency parameters */
	enum Si46xx_frequencyList_status {FREQ_LIST_INVALID, FREQ_LIST_VALID} freqencyListStatus;

	struct events
	{
		// TODO: hier noch alle Events eintragen und drauf eingehen

		// Von RD_REPLY
		/*
		 * DACQINT:
		 * Digital radio link change interrupt indicator. Indicates that something in the digital radio ensemble acquisition status has changed.
		 * Service by sending the DAB_DIGRAD_STATUS command.
		 */
		enum Si46xx_INT digital_radio_link_change;
		enum Si46xx_INT seek_tune_complete;
		enum Si46xx_INT acquisition_state_change;
		enum Si46xx_INT digital_service_data;
		enum Si46xx_INT digital_radio_event_change;

		// Von SI46XX_MSG_GET_EVENT_STATUS
		uint8_t freq_info_int:1; 	/* New Frequency Information interrupt. Indicates that new Frequency Information is available. The Frequency Information list is retrieved with the DAB_GET_FREQ_INFO command. The rate at which frequency information interrupts can occur is defined by the DAB_EVENT_MIN_FREQINFO_PERIOD property. */
		uint8_t service_list_int:1; /* New service list interrupt. Indicates that a new digital service list is available. The new service list is retrieved with the GET_DIGITAL_SERVICE_LIST command. */
	}events;

	enum DAB_frequencies freqIndex;  // TODO: aktueller FreqInd, um mit Channeldata zu vergleichen?

	struct wantedService
	{
		uint8_t serviceID;
		uint8_t componentID;
	}wantedService;

	dab_channel_dt channelData; // to contain the data of the current channel

};

#endif /* INC_SI46XX_DATA_TYPES_H_ */
