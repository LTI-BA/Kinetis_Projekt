/*
 * Regulator.h
 *
 *  Created on: 24.10.2016
 *      Author: Lanwer
 */

#ifndef INCLUDES_REGULATOR_H_
#define INCLUDES_REGULATOR_H_

#include "MK22F51212.h"

void Regulator_start(void);

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





#endif /* INCLUDES_REGULATOR_H_ */
