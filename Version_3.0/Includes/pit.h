/*
 * pit.h
 *
 *  Created on: Nov 25, 2014
 *      Author: Manuel Alejandro
 */

#ifndef PIT_H_
#define PIT_H_

#include "MK22F51212.h"

extern uint32_t duty_cycle ;
extern uint32_t duty_cycle1;
extern uint32_t duty_cycle2 ;
extern uint32_t duty_cycle3 ;
extern uint32_t duty_cycle4;

// regulation variables
// execution time of the loop
extern float I_ref ;

extern float integral_error1 ;
extern float integral_error2 ;
extern float integral_error3 ;
extern float integral_error4 ;

extern float actual_error1 ;
extern float actual_error2 ;
extern float actual_error3 ;
extern float actual_error4 ;

extern float duty_c1;
extern float duty_c2 ;
extern float duty_c3 ;
extern float duty_c4;


extern float dt ;

extern float Kp;
extern float Ki ;

extern float Kp1 ;
extern float Ki1;

extern float Kp2 ;
extern float Ki2 ;

extern float Kp3 ;
extern float Ki3 ;

extern float Kp4 ;
extern float Ki4 ;

extern float current[4];
extern float voltage[5];



#define ADC_CHANNEL 8 // Channel 12 (PTB2)  ==> Änderung Channel 8 (PTB0)
#define LED_BLUE  5 // PTB21  ==> Änderung PTD5

void pit_init(void);
void PIT_IRQHandler(void);

#endif /* PIT_H_ */
