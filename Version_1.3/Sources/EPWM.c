/*
 * EPWM.c
 *
 *  Created on: 06.07.2016
 *      Author: Lanwer
 */

#include "MK22F51212.h"
#include "mcu_tracer.h"

//Set up protection functions
//required write enable.
void FTM_protect_init(){

	//Clock to the comparator
	SIM_SCGC4|=SIM_SCGC4_CMP_MASK;
	SIM_SCGC5|=SIM_SCGC5_PORTC_MASK; ////////////////////


	// Init Overcurrent protection

	// for the positive input of the CMP0, the IN1 is used (PTC7)
	///////////////////////// THIS SHOULD NOT BE NECESSARY
	PORTC_PCR7= PORT_PCR_MUX(0);

	// Enable the comparator
	CMP0_CR1=CMP_CR1_EN_MASK;

	CMP0_CR0=CMP_CR0_FILTER_CNT(0);
	// Enable DAC, VIn1, Voltage 19/60*Vref
	// For the test, a max current of 1A is used
	// ==> U_shunt_max = 0,1 x 1  = 0,1V
	// the signal is amplified with the OP (8x)
	// so the maximum voltage is 0,8V which is routed to the INP1 of CMP0
	// the OR Function realized with the diodes reduces the voltage coming to INP1 pin by approximately 0,45 V
	// So the maximum voltage to compare to the DAC Reference is : 0,8V - 0,45V = 0,35 V
	// VIn1 = V_ref = 1.2113 V
	// Voltage (19/64) *Vref = 0,3596 V

	CMP0_DACCR=CMP_DACCR_DACEN(1)|CMP_DACCR_VRSEL(0)|CMP_DACCR_VOSEL(18);
	CMP0_MUXCR=CMP_MUXCR_PSEL(1)|CMP_MUXCR_MSEL(7); //+=Sense -=DAC

	// Init Overvoltage protection
	// config port
	// for the positive input of the CMP0, the IN1 is used (PTC3)
	///////////////////////// THIS SHOULD NOT BE NECESSARY
	PORTC_PCR3= PORT_PCR_MUX(0);

	// Enable the comparator
	CMP1_CR1=CMP_CR1_EN_MASK;
	// Set filter to react after 7 consequent samples
	CMP1_CR0=CMP_CR0_FILTER_CNT(0);

	//CMP1_SCR= CMP_SCR_IER_MASK ;


	//Overvoltage Level Calculation:
	//(0.5+Value)*17V
	//800V: 47

	// Enable DAC, VIn1, Voltage 40/60*Vref
	// For the test, a max output voltage of is 40V :
	// Passing through the voltage divider it becomes:
	// 40 x 7,5 / (1000 + 7,5) = 0,298 V
	// the OR Function realized with the diodes reduces the voltage coming to INP1 pin by approximately 0,45 V
	// So the maximum voltage to compare to the DAC Reference is : V = 0,35 V
	// VIn1 = V_ref = 1.2113 V
	// Voltage (19/64) *Vref = 0,3596 V

	// For the test, a voltage of 0,8V is applied directly to the CMP1_IN1
	// The OR function realized by the Schottky diodes is bypassed to avoid applying high voltages to the output during the tests
	// VIn1 = V_ref = 1.2113 V
	// Voltage (40/64) *Vref = 0,757 V
	// So the applied voltage (0,8) should do the work !
	CMP1_DACCR=CMP_DACCR_DACEN(1) | CMP_DACCR_VRSEL(1)|CMP_DACCR_VOSEL(0);


	// select input source (CMP1_In1)
	CMP1_MUXCR=CMP_MUXCR_PSEL(1)|CMP_MUXCR_MSEL(7); //+=Sense -=DAC




	//Select comparator outputs (CMP0 out for OC Prot and CMP1 out for OV Prot) as fault source of FTM0
	SIM_SOPT4|=(SIM_SOPT4_FTM0FLT1_MASK)|(SIM_SOPT4_FTM0FLT0_MASK);

	// Fault control is enabled for all channels, and the selected mode is the manual fault clearing.
	// write protected ==> to disable the protection: MODE[WPDIS] = 1
	FTM0_MODE|= FTM_MODE_WPDIS_MASK | FTM_MODE_FAULTM(3);

	// Enabling the fault input 0 and 1
	FTM0_FLTCTRL=FTM_FLTCTRL_FAULT0EN_MASK|FTM_FLTCTRL_FAULT1EN_MASK;

	// Enabling the fault control for all the channels  0 ==> 7
	//FTM0->COMBINE|=FTM_COMBINE_FAULTEN0_MASK|FTM_COMBINE_FAULTEN1_MASK|FTM_COMBINE_FAULTEN2_MASK|FTM_COMBINE_FAULTEN3_MASK;

	// Clearing the Fault Detection Flag
	FTM0_FMS&=(~FTM_FMS_FAULTF_MASK);


}

void FTM_fault_interrupt_enable(){
	//inits the fault interrupt function
	FTM0_MODE|=FTM_MODE_FAULTIE_MASK;
	NVIC_EnableIRQ(FTM0_IRQn);
}

void FTM0_IRQHandler(void){


	FTM0_MODE&=~FTM_MODE_FAULTIE_MASK;
	//CMP0 works on Overcurrent
	if(FTM0_FMS & FTM_FMS_FAULTF0_MASK){
		mcu_tracer_msg("Overcurrent detected. PWM off.");

		//GPIOD_PSOR = (1<<3);


	}else if(FTM0_FMS & FTM_FMS_FAULTF1_MASK){
		mcu_tracer_msg("Overvoltage detected. PWM off.");
		//GPIOD_PSOR = (1<<3);
	}else{
		mcu_tracer_msg("OverX: Unknown error. Fix me!");
	}
	//////////////////////////////
	//PWM_set_dutycycle(0) ;

}

void FTM_clear_error(void){
	//Clears overcurrent error (follow description in FAULTF Field on page 920)
	// reading the FMS register
	uint32_t fms=FTM0_FMS;
	// Check if there is a fault detected
	if(!(fms & FTM_FMS_FAULTIN_MASK)){
		// resetting the FAULTF bit in fms variable
		fms&=(~FTM_FMS_FAULTF_MASK);
		// copying the fms variable to FMS register
		FTM0_FMS=fms;
		//Enable Interrupts
		FTM_fault_interrupt_enable();
		mcu_tracer_msg("OverX: Errors reseted.");
		//GPIOD_PCOR = (1<<3);
	}else{
		mcu_tracer_msg("OverX: Error still persistend. Cannot reset.");
	}
}














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


	// disable pin for all the gate drivers in case of over current or over voltage
	PORTD_PCR3 = PORT_PCR_MUX(1) | PORT_PCR_DSE(0);
    // Set direction : output
	GPIOD_PDDR|=(1<<3);
	/*
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
	*/


	//  enable FTM0 and FTM0 module clock
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK ;
	//  Write protection is disabled. Write protected bits can now be written.
	FTM0_FMS &= ~FTM_FMS_WPEN_MASK ;
	// FTM enable and Write protection disabled , to be able to write in FTMx_CnSC and FTMx_SC and other registers
	FTM0_MODE |= FTM_MODE_WPDIS_MASK | FTM_MODE_FTMEN_MASK ;


	// FTM Protection function
	FTM_protect_init();


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
			        FTM_COMBINE_SYNCEN0_MASK| FTM_COMBINE_SYNCEN1_MASK| FTM_COMBINE_SYNCEN2_MASK| FTM_COMBINE_SYNCEN3_MASK |
	                FTM_COMBINE_FAULTEN0_MASK | FTM_COMBINE_FAULTEN1_MASK | FTM_COMBINE_FAULTEN2_MASK | FTM_COMBINE_FAULTEN3_MASK;


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

	 FTM_fault_interrupt_enable();


	 /////////////////
	 //FTM_clear_error();


}


