/*
 * VREF.c
 *
 *  Created on: 13.07.2016
 *      Author: Lanwer
 */


#include "MK22F51212.h"
#include "VREF.h"


void VREF_Init(void) {

	// enable the clock gate to the VREF module.
	SIM_SCGC4 |= SIM_SCGC4_VREF_MASK ;

	// See page 858 : 37.3.3 and follow the steps
    // CHOPEN Bit (chop oscillator enable) should be written to 1 to achieve the performance stated in the data sheet.
	// If the internal voltage regulator is being used (REGEN bit is set to 1), the chop oscillator must also be enabled.
	// Set the trim bits to get a resulting VREF of 1.2 V ( 0.5 mV each step)
	// the chosen number (32) was get by multimeter measurements to get a VREF = 1.209
	VREF_TRM |= VREF_TRM_CHOPEN_MASK | VREF_TRM_TRIM(32) ;

	// Configure the VREF_SC register to the desired settings with the internal regulator disabled, VREF_SC[REGEN] = 0
	// High power buffer mode enabled
	// Enable the internal VREF
	// The ICOMPEN Bit should be enabled to achieve the performance stated in the data sheet
	VREF_SC |= VREF_SC_REGEN(0) | VREF_SC_MODE_LV(1) | VREF_SC_VREFEN_MASK | VREF_SC_ICOMPEN_MASK ;

	// wait at least 300ns (how to ?)

	// Enable the internal regulator by setting VREF_SC[REGEN] to 1
	VREF_SC |= VREF_SC_REGEN_MASK ;

	// Wait until internal voltage stable
    while ((VREF_SC & VREF_SC_VREFST_MASK) == 0);

}

