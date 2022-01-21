/*
 * Si46xx_DAB_frequencies.h
 *
 *  Created on: Dec 30, 2021
 *      Author: kai
 */

#ifndef INC_SI46XX_DAB_FREQUENCIES_H_
#define INC_SI46XX_DAB_FREQUENCIES_H_

enum DAB_frequencies { // in kHz
	DAB_Chan_5A  = 174928,
	DAB_Chan_5B  = 176640,
	DAB_Chan_5C  = 178352,
	DAB_Chan_5D  = 180064,
	DAB_Chan_6A  = 181936,
	DAB_Chan_6B  = 183648,
	DAB_Chan_6C  = 185360,
	DAB_Chan_6D  = 187072,
	DAB_Chan_7A  = 188928,
	DAB_Chan_7B  = 190640,
	DAB_Chan_7C  = 192352,
	DAB_Chan_7D  = 194064,
	DAB_Chan_8A  = 195936,
	DAB_Chan_8B  = 197648,
	DAB_Chan_8C  = 199360,
	DAB_Chan_8D  = 201072,
	DAB_Chan_9A  = 202928,
	DAB_Chan_9B  = 204640,
	DAB_Chan_9C  = 206352,
	DAB_Chan_9D  = 208064,
	DAB_Chan_10A = 209936,
	DAB_Chan_10B = 211648,
	DAB_Chan_10C = 213360,
	DAB_Chan_10D = 215072,
	DAB_Chan_11A = 216928,
	DAB_Chan_11B = 218640,
	DAB_Chan_11C = 220352,
	DAB_Chan_11D = 222064,
	DAB_Chan_12A = 223936,
	DAB_Chan_12B = 225648,
	DAB_Chan_12C = 227360,
	DAB_Chan_12D = 229072,
	DAB_Chan_13A = 230784,
	DAB_Chan_13B = 232496,
	DAB_Chan_13C = 234208,
	DAB_Chan_13D = 235776,
	DAB_Chan_13E = 237488,
	DAB_Chan_13F = 239200,

	DAB_Chan_SIZE
};

/* NOTE: Frequencies also included inside Si46xx by default: 210096kHz, 217088kHz, 224096kHz */

#endif /* INC_SI46XX_DAB_FREQUENCIES_H_ */
