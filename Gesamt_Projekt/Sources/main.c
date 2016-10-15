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
#include "MK22F51212.h"
#include "adc.h"
#include "dma.h"
#include "EPWM.h"
#include "uart1.h"
#include "mcu_tracer.h"

float cur [4] ;
float vol [5] ;
float current[4];
float voltage[5];
//float val ;
//uint16_t value;
uint8_t duty_cycle = 0;


void main(void)
{
    mcu_tracer_config();
	ADC_init() ;
    DMA_init() ;
    // PWM_init() ;
    // PWM_set_dutycycle(duty_cycle);

for (;;) {
	mcu_tracer_process();
	//PWM_set_dutycycle(40);
	//PWM_set_dutycycle(duty_cycle);
//ADC_value = vo[0] ;
 cur[0] = vo[0] * 1.2114 / 65535 ;
 cur[1] = vo[1] * 1.2114 / 65535 ;
 cur[2] = cu[2] * 1.2114 / 65535 ;
 cur[3] = cu[3] * 1.2114 / 65535 ;

 vol[0] = vo[0] * 1.2113 / 65535 ;
 vol[1] = vo[1] * 1.2113 / 65535 ;
 vol[2] = vo[2] * 1.2113 / 65535 ;
 vol[3] = vo[3] * 1.2113 / 65535 ;
 vol[4] = vo[4] * 1.2113 / 65535 ;


 // real Values
 ////////////////////////////

 // current values: (die genaue Beziehung zw. U_out und U_sens ist noch zu implementieren)
 current[0] = cur[0] * 10 / 8 ;
 current[1] = cur[1] * 10 / 8 ;
 current[2] = cur[2] * 10 / 8 ;
 current[3] = cur[3] * 10 / 8 ;


 // Voltage values:
 voltage[0] = vol[0] * 1007.5 / 7.5 ;
 voltage[1] = vol[1] * 1007.5 / 7.5 ;
 voltage[2] = vol[2] * 1007.5 / 7.5 ;
 voltage[3] = vol[3] * 1007.5 / 7.5 ;
 voltage[4] = vol[4] * 1007.5 / 7.5 ;


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
