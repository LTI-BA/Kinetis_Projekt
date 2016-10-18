

#ifndef ADC_H_
#define ADC_H_

#include "MK22F51212.h"


void ADC_init(void);
int ADC_cal(void);
unsigned short ADC_read(unsigned char ch);

#endif /* ADC_H_ */
