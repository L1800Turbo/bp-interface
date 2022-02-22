/*
 * Si46xx_DAB_frequencies.h
 *
 *  Created on: Dec 30, 2021
 *      Author: kai
 */

#ifndef INC_SI46XX_DAB_FREQUENCIES_H_
#define INC_SI46XX_DAB_FREQUENCIES_H_

#include <stdint.h>

enum DAB_frequencies { // in kHz
	DAB_Chan_5A = 0,
	DAB_Chan_5B,
	DAB_Chan_5C,
	DAB_Chan_5D,
	DAB_Chan_6A,
	DAB_Chan_6B,
	DAB_Chan_6C,
	DAB_Chan_6D,
	DAB_Chan_7A,
	DAB_Chan_7B,
	DAB_Chan_7C,
	DAB_Chan_7D,
	DAB_Chan_8A,
	DAB_Chan_8B,
	DAB_Chan_8C,
	DAB_Chan_8D,
	DAB_Chan_9A,
	DAB_Chan_9B,
	DAB_Chan_9C,
	DAB_Chan_9D,
	DAB_Chan_10A,
	DAB_Chan_10B,
	DAB_Chan_10C,
	DAB_Chan_10D,
	DAB_Chan_11A,
	DAB_Chan_11B,
	DAB_Chan_11C,
	DAB_Chan_11D,
	DAB_Chan_12A,
	DAB_Chan_12B,
	DAB_Chan_12C,
	DAB_Chan_12D,
	DAB_Chan_13A,
	DAB_Chan_13B,
	DAB_Chan_13C,
	DAB_Chan_13D,
	DAB_Chan_13E,
	DAB_Chan_13F,

	DAB_Chan_SIZE
}DAB_frequencies;

/* NOTE: Frequencies also included inside Si46xx by default: 210096kHz, 217088kHz, 224096kHz */

typedef struct
{
	uint32_t freq;
	char name[4];
}DAB_frequency_dt;

DAB_frequency_dt DAB_frequency_list[DAB_Chan_SIZE];

#endif /* INC_SI46XX_DAB_FREQUENCIES_H_ */
