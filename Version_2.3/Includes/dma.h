/*
 * dma.h
 *
 *  Created on: 23.07.2016
 *      Author: Lanwer
 */

#ifndef SOURCES_DMA_H_
#define SOURCES_DMA_H_


#include "MK22F51212.h"


#define TOTAL_CHANNEL_NUMBER_VOLTAGE 5
#define TOTAL_CHANNEL_NUMBER_CURRENT 4

uint16_t vo[5];
uint8_t adc_mux_voltage[TOTAL_CHANNEL_NUMBER_VOLTAGE];

uint16_t cu[4];
uint8_t adc_mux_current[TOTAL_CHANNEL_NUMBER_CURRENT];


void DMA_init(void);

#endif /* SOURCES_DMA_H_ */


