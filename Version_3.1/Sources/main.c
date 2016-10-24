/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
#include "MK22F51212.h"
#include "EPWM.h"
#include "adc.h"
#include "VREF.h"
#include "dma.h"

int main(void)
{
   // uint16_t ADC_value ;

	ADC_init() ;
	DMA_init() ;
   // PWM_init();
   //PWM_set_dutycycle(50) ;

   for (;;)
	{
		//ADC_value = ADC_read(8) ;
	    //PWM_set_dutycycle((ADC_value *100)/65535) ;


	}

// Polarity When the fault signal appears, the FTM channel signals are disabled, and kept to a safe logic, which is defined in FTM_POL register. For example, if the POL0=0, when fault signals happen, the FTM_CH0 pin PWM signal is disabled, the FTM_CH0 pin is forced to become low logic. If the POL0=1, when fault signals happen, the FTM_CH0 pin PWM signal is disabled, the FTM_CH0 pin is forced to become high logic.
// ==> noch zu machen
    //return 0;
}

*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "MK22F51212.h"
#include "adc.h"
#include "dma.h"
#include "EPWM.h"
#include "uart1.h"
#include "mcu_tracer.h"
#include "VREF.h"
#include "taskcall.h"
#include "pit.h"


void clear_error(int64_t delay);
void regulator  (int64_t delay);
void PWM_reset (int64_t delay);

taskcall_t task_clear_error={"a",0,NULL,&clear_error,NULL,NULL};
taskcall_t task_regulator={"b",0,NULL,&regulator,NULL,NULL};
taskcall_t task_PWM_reset={"c",0,NULL,&PWM_reset,NULL,NULL};

float cur [4] ;
float vol [5] ;
float current[4];
float voltage[5];

int i = 0;

extern int s;
extern int c;
uint32_t clear_error_order=0;
uint32_t reset_order = 0;
uint32_t start_order = 0;


//float val ;
//uint16_t value;

uint32_t duty_cycle =  0;
uint32_t duty_cycle1 = 0;
uint32_t duty_cycle2 = 0;
uint32_t duty_cycle3 = 0;
uint32_t duty_cycle4 = 0;

// regulation variables
// execution time of the loop
float I_ref = 0.311;

float integral_error1 = 0;
float integral_error2 = 0;
float integral_error3 = 0;
float integral_error4 = 0;

float actual_error1 =   0;
float actual_error2 =   0;
float actual_error3 =   0;
float actual_error4 =   0;

float duty_c1 = 0;
float duty_c2 = 0;
float duty_c3 = 0;
float duty_c4 = 0;


float dt = 0.015;

float Kp = 0;
float Ki = 0;

float Kp1 = 7;
float Ki1 = 6;

float Kp2 = 7;
float Ki2 = 6;

float Kp3 = 7;
float Ki3 = 6;

float Kp4 = 7;
float Ki4 = 6;
/*
void PWM_reset (int64_t delay){

    if (reset_order) {
    	PWM_set_dutycycle(0);
    	integral_error1 = 0;
		integral_error2 = 0;
		integral_error3 = 0;
		integral_error4 = 0;
    	Kp = 0;
    	Ki = 0;
    	reset_order = 0;
    }



	_taskcall_task_register_time(&task_PWM_reset,(120000000/40));

}
*/

/*
void regulator (int64_t delay){

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

/*
	if (  ( (actual_error2 >0) & (current[1] < I_ref) ) | ( (actual_error2 < 0) & (current[1] > 0) )    ) {
    integral_error2 += actual_error2 * dt ;
    // calculating the new duty cycle
    duty_c2 = Kp2 * actual_error2 + Ki2 * integral_error2 ;
    duty_cycle2 = (int) ( duty_c2 * 100000 ) ;
    PWM_set_dutycycle2(duty_cycle2);
    }

/*
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


  GPIOA_PTOR |= (1<<12);
  _taskcall_task_register_time(&task_regulator,(120000000/120000));

}


/*
void clear_error(int64_t delay){

	uint32_t fms=FTM0->FMS;
	//if !(CMP0_SCR & CMP_SCR_COUT_MASK | CMP1_SCR & CMP_SCR_COUT_MASK ) {
	if (!(CMP1_SCR & CMP_SCR_COUT_MASK ) & (fms & FTM_FMS_FAULTF_MASK)) {
		fms&=(~FTM_FMS_FAULTF_MASK);
		FTM0->FMS=fms;
		//Enable Interrupts
		FTM_fault_interrupt_enable();
		mcu_tracer_msg("OverX: Errors reseted.");
        PWM_set_dutycycle(10);
	}

	/*
void FTM_clear_error(void){
	//Clears overcurrent error
	uint32_t fms=FTM0->FMS;
	if(!(fms & FTM_FMS_FAULTIN_MASK)){
		fms&=(~FTM_FMS_FAULTF_MASK);
		FTM0->FMS=fms;
		//Enable Interrupts
		FTM_fault_interrupt_enable();
		mcu_tracer_msg("OverX: Errors reseted.");
	}else{
		mcu_tracer_msg("OverX: Error still persistend. Cannot reset.");
	}
}

*/


/*
	//Clears overcurrent error
	uint32_t fms=FTM0->FMS;

	if(!(fms & FTM_FMS_FAULTIN_MASK)){
		mcu_tracer_msg("OverX: Errors reseted.");
		//PWM_set_dutycycle(30);
	}
/*
	uint32_t fms=FTM0->FMS;
		//if !(CMP0_SCR & CMP_SCR_COUT_MASK | CMP1_SCR & CMP_SCR_COUT_MASK ) {

	if  (fms & FTM_FMS_FAULTF_MASK) {

		 if (!(CMP1_SCR & CMP_SCR_COUT_MASK )) {
			fms&=(~FTM_FMS_FAULTF_MASK);
			FTM0->FMS=fms;
			//Enable Interrupts
			FTM_fault_interrupt_enable();
			mcu_tracer_msg("OverX: Errors reseted.");
	        //PWM_set_dutycycle(10);
			s=0 ;
		 }
	}
*/
/*
	uint32_t fms=FTM0->FMS;
			//if !(CMP0_SCR & CMP_SCR_COUT_MASK | CMP1_SCR & CMP_SCR_COUT_MASK ) {

		if ( s )  {

			         if (!(CMP1_SCR & CMP_SCR_COUT_MASK )) {
				        fms&=(~FTM_FMS_FAULTF_MASK);
				        FTM0->FMS=fms;
			        	//Enable Interrupts
			         	FTM_fault_interrupt_enable();
			         	mcu_tracer_msg("OverX: voltage Errors reseted.");
		                //PWM_set_dutycycle(10);
				        s=0 ;
				        GPIOA_PCOR |= (1<<13);
			         }
		}


		if ( c )  {

			         if (!(CMP0_SCR & CMP_SCR_COUT_MASK )) {
				        fms&=(~FTM_FMS_FAULTF_MASK);
				        FTM0->FMS=fms;
			        	//Enable Interrupts
			         	FTM_fault_interrupt_enable();
			         	mcu_tracer_msg("OverX: current Errors reseted.");
		                //PWM_set_dutycycle(10);
				        c=0 ;
				        GPIOA_PCOR |= (1<<5);
			         }
		}


if (clear_error_order) {

	uint32_t fms=FTM0->FMS;
			//if !(CMP0_SCR & CMP_SCR_COUT_MASK | CMP1_SCR & CMP_SCR_COUT_MASK ) {

		if ( s )  {

			         if (!(CMP1_SCR & CMP_SCR_COUT_MASK )) {
				        fms&=(~FTM_FMS_FAULTF_MASK);
				        FTM0->FMS=fms;
			        	//Enable Interrupts
			         	FTM_fault_interrupt_enable();
			         	mcu_tracer_msg("OverX: voltage Errors reseted.");
		                //PWM_set_dutycycle(10);
				        s=0 ;
				        GPIOA_PCOR |= (1<<13);
				        GPIOD_PCOR = (1<<3);
				        clear_error_order=0;
			         }
		}


		if ( c )  {

			         if (!(CMP0_SCR & CMP_SCR_COUT_MASK )) {
				        fms&=(~FTM_FMS_FAULTF_MASK);
				        FTM0->FMS=fms;
			        	//Enable Interrupts
			         	FTM_fault_interrupt_enable();
			         	mcu_tracer_msg("OverX: current Errors reseted.");
		                //PWM_set_dutycycle(10);
				        c=0 ;
				        GPIOA_PCOR |= (1<<5);
				        GPIOD_PCOR = (1<<3);
				        clear_error_order=0;
			         }
		}
}
	_taskcall_task_register_time(&task_clear_error,(120000000/29));
}

*/

void main(void){

	VREF_Init();
	SIM_SCGC5|=SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTA_MASK ;
	PORTA_PCR12 = PORT_PCR_MUX(1) | PORT_PCR_DSE(0);
	GPIOA_PDDR|=(1<<12);

	//mcu_tracer_config();
	//FTM_protect_init() ;
    ADC_init() ;
    DMA_init() ;

    PWM_init() ;

   PWM_set_dutycycle(0);
   /*
   for(i=0;i<1000000;i++)
   {}
*/
   pit_init() ;
  // PWM_set_dutycycle3(7000000) ;
	 //FTM_clear_error();
//FTM_protect_init();
   // _taskcall_task_register_time(&task_regulator,(120000000));
//	_taskcall_task_register_time(&task_clear_error,(120000000));
	//_taskcall_task_register_time(&task_PWM_reset,(120000000));
	//_taskcall_start();

for (;;) {
	//FTM_clear_error() ;
	//mcu_tracer_process();
	//FTM_clear_error() ;
	//PWM_set_dutycycle(40);
	//PWM_set_dutycycle(duty_cycle);
//ADC_value = vo[0] ;
	// PWM_set_dutycycle4(duty_cycle4);

// 12 Bit AUflösung
	 cur[0] = cu[0] * 1.2114 / 4095 ;
	 cur[1] = cu[1] * 1.2114 / 4095 ;
	 cur[2] = cu[2] * 1.2114 / 4095 ;
	 cur[3] = cu[3] * 1.2114 / 4095 ;
/*
// 10 Bit AUflösung
	 cur[0] = cu[0] * 1.2114 / 1024 ;
	 cur[1] = cu[1] * 1.2114 / 1024 ;
	 cur[2] = cu[2] * 1.2114 / 1024 ;
	 cur[3] = cu[3] * 1.2114 / 1024 ;
*/



// 16 bit resoltion
	/*
 cur[0] = cu[0] * 1.2114 / 65535 ;
 cur[1] = cu[1] * 1.2114 / 65535 ;
 cur[2] = cu[2] * 1.2114 / 65535 ;
 cur[3] = cu[3] * 1.2114 / 65535 ;
/*
 vol[0] = vo[0] * 1.2113 / 65535 ;
 vol[1] = vo[1] * 1.2113 / 65535 ;
 vol[2] = vo[2] * 1.2113 / 65535 ;
 vol[3] = vo[3] * 1.2113 / 65535 ;
 vol[4] = vo[4] * 1.2113 / 65535 ;
 */

 // 12-bit config
 vol[0] = vo[0] * 1.2113 / 4095 ;
 vol[1] = vo[1] * 1.2113 / 4095 ;
 vol[2] = vo[2] * 1.2113 / 4095 ;
 vol[3] = vo[3] * 1.2113 / 4095 ;
 vol[4] = vo[4] * 1.2113 / 4095 ;
/*
 // 10-bit config
 vol[0] = vo[0] * 1.2113 / 1024 ;
 vol[1] = vo[1] * 1.2113 / 1024 ;
 vol[2] = vo[2] * 1.2113 / 1024 ;
 vol[3] = vo[3] * 1.2113 / 1024 ;
 vol[4] = vo[4] * 1.2113 / 1024 ;
 // real Values
 ////////////////////////////
/*
 // 8-bit config
  vol[0] = vo[0] * 1.2113 / 256 ;
  vol[1] = vo[1] * 1.2113 / 256 ;
  vol[2] = vo[2] * 1.2113 / 256 ;
  vol[3] = vo[3] * 1.2113 / 256 ;
  vol[4] = vo[4] * 1.2113 / 256 ;
  */


  // real Values
  ////////////////////////////

 // current values: (die genaue Beziehung zw. U_out und U_sens ist noch zu implementieren)
 current[0] = cur[0] * 10 / 8 ;
 current[1] = cur[1] * 10 / 8 ;
 current[2] = cur[2] * 10 / 8 ;
 current[3] = cur[3] * 10 / 8 ;

/*
 // Voltage values:
 voltage[0] = vol[0] * 1007.5 / 7.5 ;
 voltage[1] = vol[1] * 1007.5 / 7.5 ;
 voltage[2] = vol[2] * 1007.5 / 7.5 ;
 voltage[3] = vol[3] * 1007.5 / 7.5 ;
 voltage[4] = vol[4] * 1007.5 / 7.5 ;


FTM_clear_error();

 */

 //FTM_clear_error();
 // Voltage values:
 voltage[0] = vol[0] * 1006.458 / 7.485 ;
 voltage[1] = vol[1] * 1006.458 / 7.485 ;
 voltage[2] = vol[2] * 1006.458 / 7.485 ;
 voltage[3] = vol[3] * 1006.458 / 7.485 ;
 voltage[4] = vol[4] * 1006.485 / 7.485 ;

/*
 actual_error1 = I_ref - current[0] ;
	actual_error2 = I_ref - current[1] ;
 actual_error3 = I_ref - current[2] ;
	actual_error4 = I_ref - current[3] ;
	 /*
	if (  ( (actual_error1 >0) & (current[0] < I_ref) ) | ( (actual_error1 < 0) & (current[0] > 0) )    ) {
	 integral_error1 += actual_error1 * dt ;
	 // calculating the new duty cycle
	 duty_c1 = 8.5 * actual_error1 + 7 * integral_error1 ;
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
	    duty_c4 = 9.6 * actual_error4 + 8.2 * integral_error4 ;
	    duty_cycle4 = (int) ( duty_c4 * 100000 ) ;
	    PWM_set_dutycycle4(duty_cycle4);
	    }


*/

 //GPIOA_PTOR |= (1<<12);

/*
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


//FTM_clear_error();
 //mcu_tracer_process();
 // flag= FTM0->FMS & 11111111;

/*
 value = ADC_read(8) ;
 val = value * 1.2114 / 65535;
*/
// Clock has to be 15 or 7,5 Mhz !!! (max = 12 Mhz)
// Deatime value was changed to 333 ns instead of the 200 ns ==> (2,5) instead of (2,3)
//value = vo[0] ;
	//PWM_set_dutycycle((ADC_value *100)/65535) ;
	//ADC_value = cu[0] ;
}

}
