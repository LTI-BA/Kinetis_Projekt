/*
 * EPWM.h
 *
 *  Created on: 06.07.2016
 *      Author: Lanwer
 */

#ifndef INCLUDES_EPWM_H_
#define INCLUDES_EPWM_H_


void PWM_init(void);
void PWM_deadtime_enable(void);
void PWM_set_deadtime (int b, int c);
void PWM_set_frequency(void);
void PWM_set_dutycycle (int a) ;
void PWM_set_dutycycle1 (int a) ;
void PWM_set_dutycycle2 (int a) ;
void PWM_set_dutycycle3 (int a) ;
void PWM_set_dutycycle4 (int a) ;
void FTM_clear_error(void);
void FTM_protect_init(void);
void FTM_fault_interrupt_enable(void);
void FTM0_IRQHandler(void);

#endif /* INCLUDES_EPWM_H_ */
