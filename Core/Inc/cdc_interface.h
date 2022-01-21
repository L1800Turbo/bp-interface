/*
 * cdc_interface.h
 *
 *  Created on: Dec 31, 2021
 *      Author: kai
 */

#ifndef INC_CDC_INTERFACE_H_
#define INC_CDC_INTERFACE_H_

#include <stdint.h>

typedef struct
{
	enum fileTransfer_en
	{
		CDC_FILE_TRANSFER_INACTIVE = 0,
		CDC_FILE_TRANSFER_ACTIVE	// TODO: Zunächst nur für Si46xx Firmware, später mehr?
	}fileTransfer_state;

	uint8_t* fileTransfer_ptr;

}cdc_interface_dt;

void cdc_Interface_Init(void);
void cdc_Interface_Tasks(void);

#endif /* INC_CDC_INTERFACE_H_ */
