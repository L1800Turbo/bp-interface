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

#include "main.h"



#include "Si46xx_data_types.h"
#include "Si46xx_SPI_functions.h"
#include "Si46xx_DAB_frequencies.h"

#define SI46XX_BOOT_MAX_BUF_SIZE 4092

enum Si46xx_Wait_en {
	TIME_LEFT = 0,
	TIME_OVER
};

// Bootloader patch
const uint8_t Si46xx_Rom00Patch016[5796];
//const uint8_t Si46xx_Firmware[499760]; // BIF
const uint8_t Si46xx_Firmware[499356]; // BIN

void set_Si46xx_ISR(void);
void Si46xx_Reset(void);
uint8_t Si46xx_isBusy(void);

void Si46xx_Tasks(void);

HAL_StatusTypeDef Si46xx_InitConfiguration(SPI_HandleTypeDef * hspi);

HAL_StatusTypeDef Si46xx_SPIgetStatus(SPI_HandleTypeDef * hspi, uint8_t * data, uint16_t len);
//Si46xx_statusType Si46xx_SPIgetAnalyzeStatus(uint8_t * data, uint16_t len); // TODO: durch das gegenseitig includen macht das ärger

HAL_StatusTypeDef Si46xx_SPIsend(SPI_HandleTypeDef * hspi, uint8_t * data, uint16_t len);

void progress_StatusBytes(Si46xx_Status_Values_dt * status, uint8_t * data);

void Si46xx_SetWaitTime(uint32_t ms);
uint32_t Si46xx_CurrentWaitTime(void);
uint8_t Si46xx_RemainingTimeLeft(void);

#endif /* INC_SI46XX_H_ */
