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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "MK22F51212.h"
#include "adc.h"
#include "dma.h"
#include "EPWM.h"
#include "uart1.h"
#include "pit.h"
#include "VREF.h"
//#include "mcu_tracer.h"
//#include "taskcall.h"

float cur[4];
float vol[5];
float current[4];
float voltage[5];

int i = 0;

extern int s;
extern int c;

uint32_t duty_cycle = 0;
uint32_t duty_cycle1 = 0;
uint32_t duty_cycle2 = 0;
uint32_t duty_cycle3 = 0;
uint32_t duty_cycle4 = 0;

// regulation variables

// execution time of the loop
float I_ref = 0.313;

float integral_error1 = 0;
float integral_error2 = 0;
float integral_error3 = 0;
float integral_error4 = 0;

float actual_error1 = 0;
float actual_error2 = 0;
float actual_error3 = 0;
float actual_error4 = 0;

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

void main(void) {

// initialize the reference voltage of the mcu
	VREF_Init();

// initialize the peripherals
	ADC_init();
	DMA_init();
	PWM_init();

// set the duty cycles to 0
	PWM_set_dutycycle(0);

	for (i = 0; i < 1000000; i++) {
	}

// start the regulation
	pit_init();

	for (;;) {

		// 12 Bit AUflösung
		cur[0] = cu[0] * 1.2114 / 4095;
		cur[1] = cu[1] * 1.2114 / 4095;
		cur[2] = cu[2] * 1.2114 / 4095;
		cur[3] = cu[3] * 1.2114 / 4095;

		// 12-bit config
		vol[0] = vo[0] * 1.2113 / 4095;
		vol[1] = vo[1] * 1.2113 / 4095;
		vol[2] = vo[2] * 1.2113 / 4095;
		vol[3] = vo[3] * 1.2113 / 4095;
		vol[4] = vo[4] * 1.2113 / 4095;

		// real Values
		// current values: (die genaue Beziehung zw. U_out und U_sens ist noch zu implementieren)
		current[0] = cur[0] * 10 / 8;
		current[1] = cur[1] * 10 / 8;
		current[2] = cur[2] * 10 / 8;
		current[3] = cur[3] * 10 / 8;

		// Voltage values:
		voltage[0] = vol[0] * 1006.458 / 7.485;
		voltage[1] = vol[1] * 1006.458 / 7.485;
		voltage[2] = vol[2] * 1006.458 / 7.485;
		voltage[3] = vol[3] * 1006.458 / 7.485;
		voltage[4] = vol[4] * 1006.485 / 7.485;

	}

}
