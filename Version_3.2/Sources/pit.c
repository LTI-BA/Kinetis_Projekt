/*
 * pit.c
 *
 *  Created on: Nov 25, 2014
 *      Author: Manuel Alejandro
 *  Modified:   Anouar Raddaoui
 *  changed the time of the interrupt and added the regulation code to the ISR
 */

#include "pit.h"
#include "Regulator.h"

// Initializes the PIT module to produce an interrupt every second
void pit_init(void) {

	// Enable PIT clock
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;

	// Turn on PIT
	PIT_MCR = 0;

	// Configure PIT to produce an interrupt every 33 us
	// 1/60Mhz = 16.67 ns   (33 us/ 16.67ns) -1 = 2000 cycles
	PIT_LDVAL0 = 2000;
	// Enable interrupt and enable timer
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

	// configure the green LED on the PCB to toggle in the ISR
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTA_MASK;
	PORTA_PCR12 = PORT_PCR_MUX(1) | PORT_PCR_DSE(0);
	GPIOA_PDDR |= (1 << 12);

	// Enable interrupt registers ISER and ICPR
	NVIC_EnableIRQ(PIT0_IRQn);
}

//	Handles PIT interrupt if enabled
//  Starts conversion in ADC0 with single ended channel 8 (PTB0) as input

void PIT0_IRQHandler(void) {

	// Clear interrupt
	PIT_TFLG0 = PIT_TFLG_TIF_MASK;

	// toggle the green LED
	GPIOA_PTOR |= (1 << 12);

	// start the regulation process
	Regulator_start();

}
