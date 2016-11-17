/*
 * Regulator.c
 *
 *  Created on: 24.10.2016
 *      Author: Lanwer
 */
#include "Regulator.h"
#include "EPWM.h"

void Regulator_start(void){

	actual_error1 = I_ref - current[0];
	actual_error2 = I_ref - current[1];
	actual_error3 = I_ref - current[2];
	actual_error4 = I_ref - current[3];

	if (((actual_error1 > 0) & (current[0] < I_ref))
			| ((actual_error1 < 0) & (current[0] > 0))) {
		integral_error1 += actual_error1 * dt;
		// calculating the new duty cycle
		duty_c1 = Kp1 * actual_error1 + Ki1 * integral_error1;
		duty_cycle1 = (int) (duty_c1 * 100000);
		PWM_set_dutycycle1(duty_cycle1);
	}

	if (((actual_error2 > 0) & (current[1] < I_ref))
			| ((actual_error2 < 0) & (current[1] > 0))) {
		integral_error2 += actual_error2 * dt;
		// calculating the new duty cycle
		duty_c2 = Kp2 * actual_error2 + Ki2 * integral_error2;
		duty_cycle2 = (int) (duty_c2 * 100000);
		PWM_set_dutycycle2(duty_cycle2);
	}

	if (((actual_error3 > 0) & (current[2] < I_ref))
			| ((actual_error3 < 0) & (current[2] > 0))) {
		integral_error3 += actual_error3 * dt;
		// calculating the new duty cycle
		duty_c3 = Kp3 * actual_error3 + Ki3 * integral_error3;
		duty_cycle3 = (int) (duty_c3 * 100000);
		PWM_set_dutycycle3(duty_cycle3);
	}

	if (((actual_error4 > 0) & (current[3] < I_ref))
			| ((actual_error4 < 0) & (current[3] > 0))) {
		integral_error4 += actual_error4 * dt;
		// calculating the new duty cycle
		duty_c4 = Kp4 * actual_error4 + Ki4 * integral_error4;
		duty_cycle4 = (int) (duty_c4 * 100000);
		PWM_set_dutycycle4(duty_cycle4);
	}
}



