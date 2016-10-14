/*
 * EPWM.c
 *
 *  Created on: 06.07.2016
 *      Author: Lanwer
 */

#include "MK22F51212.h"


// Setting the duty cycle of the EPWM , a = 0..100   (%)
void PWM_set_dutycycle (int a) {

//Loading updated values is enabled.
FTM0_PWMLOAD|= FTM_PWMLOAD_LDOK_MASK ;

//  match value for the output modes for all the channels
FTM0_C0V=(a*170)/100 ; //  match value for the output modes (CH0)
FTM0_C1V=(a*170)/100 ; //  match value for the output modes (CH1)
FTM0_C2V=(a*170)/100 ; //  match value for the output modes (CH2)
FTM0_C3V=(a*170)/100 ; //  match value for the output modes (CH3)
FTM0_C4V=(a*170)/100 ; //  match value for the output modes (CH4)
FTM0_C5V=(a*170)/100 ; //  match value for the output modes (CH5)
FTM0_C6V=(a*170)/100 ; //  match value for the output modes (CH6)
FTM0_C7V=(a*170)/100 ; //  match value for the output modes (CH7)
}

void PWM_set_dutycycle1 (int a) {

//Loading updated values is enabled.
FTM0_PWMLOAD|= FTM_PWMLOAD_LDOK_MASK ;
//  match value for the output modes ( CH4 and CH5 ) for the PWM signals of the first Buck converter
FTM0_C4V=(a*170)/100 ; //  match value for the output modes (CH4)
FTM0_C5V=(a*170)/100 ; //  match value for the output modes (CH5)

}

void PWM_set_dutycycle2 (int a) {

//Loading updated values is enabled.
FTM0_PWMLOAD|= FTM_PWMLOAD_LDOK_MASK ;
//  match value for the output modes ( CH0 and CH1 ) for the PWM signals of the second Buck converter
FTM0_C0V=(a*170)/100 ; //  match value for the output modes (CH0)
FTM0_C1V=(a*170)/100 ; //  match value for the output modes (CH1)

}

void PWM_set_dutycycle3 (int a) {

//Loading updated values is enabled.
FTM0_PWMLOAD|= FTM_PWMLOAD_LDOK_MASK ;
//  match value for the output modes ( CH0 and CH1 ) for the PWM signals of the third Buck converter
FTM0_C2V=(a*170)/100 ; //  match value for the output modes (CH2)
FTM0_C3V=(a*170)/100 ; //  match value for the output modes (CH3)

}

void PWM_set_dutycycle4 (int a) {

//Loading updated values is enabled.
FTM0_PWMLOAD|= FTM_PWMLOAD_LDOK_MASK ;
//  match value for the output modes ( CH0 and CH1 ) for the PWM signals of the fourth Buck converter
FTM0_C6V=(a*170)/100 ; //  match value for the output modes (CH6)
FTM0_C7V=(a*170)/100 ; //  match value for the output modes (CH7)

}


// Enables the deadtime insertion for all the channels (n) and (n+1).
void PWM_deadtime_enable (void) {
	FTM0_COMBINE |= FTM_COMBINE_DTEN0_MASK | FTM_COMBINE_DTEN1_MASK |FTM_COMBINE_DTEN2_MASK |FTM_COMBINE_DTEN3_MASK   ;
}



// Divide the system clock by 1 (deadtime prescaler) and Deadtime value : Deadtime insert value = (DTPS × DTVAL).  (Maximaler wert = 63)
void PWM_set_deadtime (int b, int c)  {                              // b: deadtime prescaler value b= 0..3  ;  c: Deadtime Value c= 0..63 ;
FTM0_DEADTIME = FTM_DEADTIME_DTPS(b)| FTM_DEADTIME_DTVAL(c) ;
}

// Set the EPWM frequency throigh the MOD and CNTIN and FTM clock values.
void PWM_set_frequency(void) {
	// writing the modulo value for the FTM Counter ==> Frequency
	FTM0_MOD=170;
	//  setting the initial value for the FTM counter to 0
	FTM0_CNTIN = 0x00;
	// clock source selection : System clock and Prescaler Factor Selection : devide by 1
	FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);
}

// Initialiase the Flextimer to work as an edge aligned PWM in complementary mode
// (enabling the PORT clocks (SIM_SCGC5) and the FTM clock, Muxing the pins for the channels, enabling the FTM)
void PWM_init(void) {
	/*
	// The used Channel 0 ==> 7 are on port C and D ==> Enable the clock of PORT C and D (SIM_SCGC5)
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;
	// Setting the used pins for FTM function
	// FTM0 channel 0 (PTC1 ==> J24_6)
	PORTC_PCR1 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK;
	// FTM0 channel 1 (PTC2 ==> J24_8)
	PORTC_PCR2 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK;
	// FTM0 channel 2
	PORTC_PCR5 = PORT_PCR_MUX(7) | PORT_PCR_DSE_MASK;
	// FTM0 channel 3
	PORTC_PCR4 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK;
	// FTM0 channel 4
	PORTD_PCR4 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK;
	// FTM0 channel 5
	PORTD_PCR5 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK;
	// FTM0 channel 6
	PORTD_PCR6 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK;
	// FTM0 channel 7
	PORTD_PCR7 = PORT_PCR_MUX(4) | PORT_PCR_DSE_MASK;

	*/
	// The used Channels 0 ==> 7 are on port C and D ==> Enable the clock of PORT C and D (SIM_SCGC5)
		SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;
		// Setting the used pins for FTM function

		// FTM0 channel 0
		PORTC_PCR1 = PORT_PCR_MUX(4);
		// FTM0 channel 1
		PORTC_PCR2 = PORT_PCR_MUX(4) ;
		// FTM0 channel 2
		PORTC_PCR5 = PORT_PCR_MUX(7) ;
		// FTM0 channel 3
		PORTC_PCR4 = PORT_PCR_MUX(4) ;
		// FTM0 channel 4
		PORTD_PCR4 = PORT_PCR_MUX(4) ;
		// FTM0 channel 5
		PORTD_PCR5 = PORT_PCR_MUX(4) ;
		// FTM0 channel 6
		PORTD_PCR6 = PORT_PCR_MUX(4) ;
		// FTM0 channel 7
		PORTD_PCR7 = PORT_PCR_MUX(4) ;

	//  enable FTM0 and FTM0 module clock
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK ;
	//  Write protection is disabled. Write protected bits can now be written.
	FTM0_FMS &= ~FTM_FMS_WPEN_MASK ;
	// FTM enable and Write protection disabled , to be able to write in FTMx_CnSC and FTMx_SC and other registers
	FTM0_MODE |= FTM_MODE_WPDIS_MASK | FTM_MODE_FTMEN_MASK ;
	// Edge Aligned PWM low-true pulses (set Output on match)
	FTM0_C0SC= (FTM_CnSC_MSB(1) | FTM_CnSC_MSA(0) | FTM_CnSC_ELSB(0) | FTM_CnSC_ELSA(1) );
	FTM0_C1SC= (FTM_CnSC_MSB(1) | FTM_CnSC_MSA(0) | FTM_CnSC_ELSB(0) | FTM_CnSC_ELSA(1) );
	FTM0_C2SC= (FTM_CnSC_MSB(1) | FTM_CnSC_MSA(0) | FTM_CnSC_ELSB(0) | FTM_CnSC_ELSA(1) );
	FTM0_C3SC= (FTM_CnSC_MSB(1) | FTM_CnSC_MSA(0) | FTM_CnSC_ELSB(0) | FTM_CnSC_ELSA(1) );
	FTM0_C4SC= (FTM_CnSC_MSB(1) | FTM_CnSC_MSA(0) | FTM_CnSC_ELSB(0) | FTM_CnSC_ELSA(1) );
	FTM0_C5SC= (FTM_CnSC_MSB(1) | FTM_CnSC_MSA(0) | FTM_CnSC_ELSB(0) | FTM_CnSC_ELSA(1) );
	FTM0_C6SC= (FTM_CnSC_MSB(1) | FTM_CnSC_MSA(0) | FTM_CnSC_ELSB(0) | FTM_CnSC_ELSA(1) );
	FTM0_C7SC= (FTM_CnSC_MSB(1) | FTM_CnSC_MSA(0) | FTM_CnSC_ELSB(0) | FTM_CnSC_ELSA(1) );

	// Enables Complementary mode and the PWM synchronization for the combined channels.
	//In Complementary mode the channel (n+1) output is the inverse of the channel (n) output.
	FTM0_COMBINE |= FTM_COMBINE_COMP0_MASK | FTM_COMBINE_COMP1_MASK |FTM_COMBINE_COMP2_MASK |FTM_COMBINE_COMP3_MASK |
			        FTM_COMBINE_SYNCEN0_MASK| FTM_COMBINE_SYNCEN1_MASK| FTM_COMBINE_SYNCEN2_MASK| FTM_COMBINE_SYNCEN3_MASK;


// --------------------------------------------------------------------------------------------------//
	// Deadtime configuration:
	// Enables the deadtime insertion in the channels (n) and (n+1).
	PWM_deadtime_enable() ;

	// Deadtime calculation [s]= (DTPS × DTVAL) / (60 Mhz)
	// a: deadtime prescaler value DTPS:  b= 0..3 ( see p. 914 for the prescaler values); 0 or 1 means devide by 1; 2 means devide by 4 ; 3 means devide by 16 ;
	// b: Deadtime Value c= 0..63 ;
	// Example : for DTPS = 2  (means 4) and DTVAL = 3 ==> Deadtime [s] = (4 * 3) / 60 Mhz = 200 ns
	PWM_set_deadtime (2,3) ;

// --------------------------------------------------------------------------------------------------//
	// Set the EPWM frequency throigh the MOD and CNTIN and FTM clock values.
	// PWM Frequency = FTM Frequency / ( MOD - CNTIN +1 ) = 60 Mhz / ( 170 - 0 + 1 ) = 350.87 Khz
	PWM_set_frequency() ;


// --------------------------------------------------------------------------------------------------//
	// PWM Synchronization: (synchronization was already enabled in FTMx_CMOBINE register)
    // to enable the update of FTM Registers (such as CnV register to set the duty cycle), we need to configure the PWM synchronization
	// for that, I followed the flowchart in AN4560 (Page 5)
	/* When the SWSYNC bit (Software synch) is set to 1, the duty cycle update synchronization is controlled by CNTMAX and CNTMIN. These bits define where
       the duty cycle update takes place. If CNTMAX is set to 1, the duty cycle update is done when the FTM
	   counter reaches its maximum value. If CNTMIN is set to 1, the duty cycle update is done when the FTM
	   counter reaches its minimum value. The software duty cycle update is typically for PMSM motor control applications.
	   So it should be also fine to work with it for the buck converter */
	FTM0_SYNC|= FTM_SYNC_CNTMIN_MASK  | FTM_SYNC_SWSYNC_MASK ;

	// The enhanced PWM synchronization mode is recommended for motor control and power conversion applications ==> SYNCONF[SYNCMODE] = 1
	/* If the SWRSTCNT bit is set, the FTM counter restarts with FTM_CNTIN register value and the FTM_MOD and FTM_CnV registers are updated immediately.
	   If the SWRSTCNT bit is cleared, the FTM counter continues to count normally and the FTM_MOD and FTM_CnV register update at the next loading point. */
	// SWWRBUF The software trigger activates MOD, CNTIN, and CV registers synchronization.
	FTM0_SYNCONF|= FTM_SYNCONF_SYNCMODE_MASK |  FTM_SYNCONF_SWWRBUF_MASK | FTM_SYNCONF_SWRSTCNT_MASK ;

	// PWM Synchronization mode : no restrictions
	 FTM0_MODE &= ~FTM_MODE_PWMSYNC_MASK ;


}


