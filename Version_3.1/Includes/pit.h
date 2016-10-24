/*
 * pit.h
 *
 *  Created on: Nov 25, 2014
 *      Author: Manuel Alejandro
 */

#ifndef PIT_H_
#define PIT_H_

#include "MK22F51212.h"





#define ADC_CHANNEL 8 // Channel 12 (PTB2)  ==> Änderung Channel 8 (PTB0)
#define LED_BLUE  5 // PTB21  ==> Änderung PTD5

void pit_init(void);
void PIT_IRQHandler(void);

#endif /* PIT_H_ */
