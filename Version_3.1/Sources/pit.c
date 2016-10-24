/*
 * pit.c
 *
 *  Created on: Nov 25, 2014
 *      Author: Manuel Alejandro
 */

#include "pit.h"
#include "Regulator.h"

/* Initializes the PIT module to produce an interrupt every second
 *
 * */
void pit_init(void)
{
	// Enable PIT clock
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;


	// Turn on PIT
	PIT_MCR = 0;

	// Configure PIT to produce an interrupt every 1s
	//PIT_LDVAL0 = 0x1312CFF;	// 1/20Mhz = 50ns   (1s/50ns)-1= 19,999,999 cycles or 0x1312CFF
	PIT_LDVAL0 = 2000;	// 1/20Mhz = 50ns   (1s/50ns)-1= 19,999,999 cycles or 0x1312CFF
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK; // Enable interrupt and enable timer

	// Enable interrupt registers ISER and ICPR
	NVIC_EnableIRQ(PIT0_IRQn);
}

/*	Handles PIT interrupt if enabled
 *
 * 	Starts conversion in ADC0 with single ended channel 8 (PTB0) as input
 *
 * */
void PIT0_IRQHandler(void)
{
	// Clear interrupt
	PIT_TFLG0 = PIT_TFLG_TIF_MASK;


	// Toggle green LED
	  GPIOA_PTOR |= (1<<12);

	Regulator_start();



}
