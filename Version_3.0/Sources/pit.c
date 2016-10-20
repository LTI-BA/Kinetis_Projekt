/*
 * pit.c
 *
 *  Created on: Nov 25, 2014
 *      Author: Manuel Alejandro
 */

#include "pit.h"

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

	    actual_error1 = I_ref - current[0] ;
	  	actual_error2 = I_ref - current[1] ;
	    actual_error3 = I_ref - current[2] ;
	  	actual_error4 = I_ref - current[3] ;

	  	 if (  ( (actual_error1 >0) & (current[0] < I_ref) ) | ( (actual_error1 < 0) & (current[0] > 0) )    ) {
	  	 integral_error1 += actual_error1 * dt ;
	  	 // calculating the new duty cycle
	  	 duty_c1 = Kp1 * actual_error1 + Ki1 * integral_error1 ;
	  	 duty_cycle1 = (int) ( duty_c1 * 100000 ) ;
	  	 PWM_set_dutycycle1(duty_cycle1);
	  	 }


	  		if (  ( (actual_error2 >0) & (current[1] < I_ref) ) | ( (actual_error2 < 0) & (current[1] > 0) )    ) {
	  	 integral_error2 += actual_error2 * dt ;
	  	 // calculating the new duty cycle
	  	 duty_c2 = Kp2 * actual_error2 + Ki2 * integral_error2 ;
	  	 duty_cycle2 = (int) ( duty_c2 * 100000 ) ;
	  	 PWM_set_dutycycle2(duty_cycle2);
	  	 }

	  		if (  ( (actual_error3 >0) & (current[2] < I_ref) ) | ( (actual_error3 < 0) & (current[2] > 0) )    ) {
	  		    integral_error3 += actual_error3 * dt ;
	  		    // calculating the new duty cycle
	  		    duty_c3 = Kp3 * actual_error3 + Ki3 * integral_error3 ;
	  		    duty_cycle3 = (int) ( duty_c3 * 100000 ) ;
	  		    PWM_set_dutycycle3(duty_cycle3);
	  		    }



	  		if (  ( (actual_error4 >0) & (current[3] < I_ref) ) | ( (actual_error4 < 0) & (current[3] > 0) )    ) {
	  	    integral_error4 += actual_error4 * dt ;
	  	    // calculating the new duty cycle
	  	    duty_c4 = Kp4 * actual_error4 + Ki4 * integral_error4 ;
	  	    duty_cycle4 = (int) ( duty_c4 * 100000 ) ;
	  	    PWM_set_dutycycle4(duty_cycle4);
	  	    }

}
