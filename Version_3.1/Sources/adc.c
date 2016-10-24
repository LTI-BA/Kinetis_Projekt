
#include "MK22F51212.h"
#include "VREF.h"
#include "adc.h"

//The calibration function sets the offset calibration value, the minus-side calibration values, and the plus-side calibration values.
/* For best calibration results: (See page 795 of the reference manual)
• Set hardware averaging to maximum, that is, SC3[AVGE]=1 and SC3[AVGS]=11 for an average of 32
• Set ADC clock frequency fADCK less than or equal to 4 MHz
• VREFH=VDDA
• Calibrate at nominal voltage and temperature
return 1 if failed ==> ADC accuracy specifications are not guaranteed, otherweise return 0 */
int ADC_calibrate(void)
{

	// Calibrating ADC0
	// Set ADC clock frequency fADCK less than or equal to 4 MHz ==> input clock is the ALTCLK OSCERCLK (see page 105) [8 Mhz]
	// Divide the clock by 4 ==>  fADK = 2Mhz (this clock is just used for the calibration process)
	// choose the 16-bits mode
	ADC0_CFG1 |= ADC_CFG1_ADICLK(2)| ADC_CFG1_ADIV(2) | ADC_CFG1_MODE(3);
	// Set hardware averaging to maximum, that is, SC3[AVGE]=1 and SC3[AVGS]=11 (3) for an average of 32
	// and then start the calibration
	ADC0_SC3 |= ADC_SC3_AVGS(3) | ADC_SC3_AVGE_MASK | ADC_SC3_CAL_MASK;
	// Wait for the calibration to be done
	while(ADC0_SC3 & ADC_SC3_CAL_MASK);
	// Check if any error occured while calibrating and return 1 if failed ==> ADC accuracy specifications are not guaranteed, otherweise return 0
    if(ADC0_SC3 & ADC_SC3_CALF_MASK)
		    return 1;

	// follow the steps on page 796:

    // Initialize or clear a 16-bit variable in RAM.
    uint16_t variable = 0;
    // Add the plus-side calibration results CLP0, CLP1, CLP2, CLP3, CLP4, and CLPS to the variable.
    variable += ADC0_CLPS +  ADC0_CLP0 + ADC0_CLP1 + ADC0_CLP2 + ADC0_CLP3 + ADC0_CLP4;
    // Divide the variable by two.
    variable /= 2;
    // Set the MSB of the variable.
    variable |= 0b1000000000000000;
	// Store the value in the plus-side gain calibration register PG.
    ADC0_PG = variable;
	// Repeat the procedure for the minus-side gain calibration value.
	variable = 0;
	variable += ADC0_CLMS + ADC0_CLM0 + ADC0_CLM1 + ADC0_CLM2 + ADC0_CLM3 + ADC0_CLM4;
	variable /= 2;
	variable |= 0b1000000000000000;
	ADC0_MG = variable;


	// Calibrating ADC1
	// Set ADC clock frequency fADCK less than or equal to 4 MHz ==> input clock is the ALTCLK OSCERCLK (see page 105) [8 Mhz]
	// Devide the clock by 4 ==>  fADK = 2Mhz (this clock is just used for the calibration process)
	// choose the 16-bits mode
	ADC1_CFG1 |= ADC_CFG1_ADICLK(2)| ADC_CFG1_ADIV(2) | ADC_CFG1_MODE(3);
	// Set hardware averaging to maximum, that is, SC3[AVGE]=1 and SC3[AVGS]=11 (3) for an average of 32
	// and then start the calibration
	ADC1_SC3 |= ADC_SC3_AVGS(0) | ADC_SC3_AVGE_MASK | ADC_SC3_CAL_MASK;
	// Wait for the calibration to be done
	while(ADC1_SC3 & ADC_SC3_CAL_MASK);
	// Check if any error occured while calibrating and return 1 if failed ==> ADC accuracy specifications are not guaranteed, otherweise return 0
    if(ADC1_SC3 & ADC_SC3_CALF_MASK)
		    return 1;
	// follow the steps on page 796:

    // Initialize or clear a 16-bit variable in RAM.
    variable = 0;
    // Add the plus-side calibration results CLP0, CLP1, CLP2, CLP3, CLP4, and CLPS to the variable.
    variable += ADC1_CLPS +  ADC1_CLP0 + ADC1_CLP1 + ADC1_CLP2 + ADC1_CLP3 + ADC1_CLP4;
    // Divide the variable by two.
    variable /= 2;
    // Set the MSB of the variable.
    variable |= 0b1000000000000000;
	// Store the value in the plus-side gain calibration register PG.
    ADC1_PG = variable;
	// Repeat the procedure for the minus-side gain calibration value.
	variable = 0;
	variable += ADC1_CLMS + ADC1_CLM0 + ADC1_CLM1 + ADC1_CLM2 + ADC1_CLM3 + ADC1_CLM4;
	variable /= 2;
	variable |= 0b1000000000000000;
	ADC1_MG = variable;

	       return 0;
}


/* ADC_init()
 * Calibrates and initializes ADC0 and ADC1 to perform conversions from different channels
 * and generate DMA requests at the end of the conversion, triggering the DMA channel to transfer the ADC result to SRAM
 * ADC1 is used for current values and ADC0 for voltage values
 */
void ADC_init(void)
{
	// Configure a voltage reference to get a VREF = 1.209 V
	//VREF_Init();

	// Enable the necessary clocks : 1) for the ADC0 and ADC1 Modules ; 2) for the PORTB and PORTC
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK | SIM_SCGC6_ADC1_MASK;	// ADC0 and ADC1 clock
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;	// PTB clock and PORT C

	// pin muxing for the ADC0 channels:
	// pin muxing for pin number 35 (PTB0/LLWU_P5); choose the alternative 0 : ADC0_SE8
	PORTB_PCR0 |= PORT_PCR_MUX(0) | PORT_PCR_DSE_MASK;
	// pin muxing for pin number 36 (PTB1); choose the alternative 0 : ADC0_SE9
	PORTB_PCR1 |= PORT_PCR_MUX(0) | PORT_PCR_DSE_MASK;
	// pin muxing for pin number 37 (PTB2); choose the alternative 0 : ADC0_SE12
	PORTB_PCR2 |= PORT_PCR_MUX(0) | PORT_PCR_DSE_MASK;
	// pin muxing for pin number 38 (PTB3); choose the alternative 0 : ADC0_SE13
	PORTB_PCR3 |= PORT_PCR_MUX(0) | PORT_PCR_DSE_MASK;
    // pin 18 for ADC0_SE23 is not on ports

	// pin muxing for the ADC1 channels
	// pin muxing for pin number 53 (PTC8) ; choose the alternative 0 : ADC0_SE4b
	PORTC_PCR8  = PORT_PCR_MUX(0);
	// pin muxing for pin number 54 (PTC9) ; choose the alternative 0 : ADC0_SE5b
	PORTC_PCR9  = PORT_PCR_MUX(0);
	// pin muxing for pin number 55 (PTC10); choose the alternative 0 : ADC0_SE6b
	PORTC_PCR10 = PORT_PCR_MUX(0);
	// pin muxing for pin number 56 (PTC11); choose the alternative 0 : ADC0_SE7b
	PORTC_PCR11 = PORT_PCR_MUX(0);


	// Calibrate the ADCs
	ADC_calibrate();
	ADC_calibrate();
	ADC_calibrate();



    //  1.2 V VREF_OUT is connected as the VALT reference option
	ADC0_SC2 |= ADC_SC2_REFSEL(1) ;
	ADC1_SC2 |= ADC_SC2_REFSEL(1) ;


	// Reset the CFG1- register
	ADC0_CFG1 = 0;
	ADC1_CFG1 = 0;

	// Set ADC clock frequency fADCK as the bus Clock (60 Mhz)
	// Devide the clock by 8
	// configure the module for 16-bit resolution
	/* Sampling time configuration : sample time is the time that the internal switch is closed and sample capacitor is charged.
	Long sample Time has been chosen for better results: This allows higher impedance inputs to be accurately sampled or to maximize
	conversion speed for lower impedance inputs. Longer sample times can also be used to lower overall
	power consumption when continuous conversions are enabled if high conversion rates are not required. */
	ADC0_CFG1 |=  ADC_CFG1_ADICLK(0)| ADC_CFG1_ADIV(2) | ADC_CFG1_MODE(1) | ADC_CFG1_ADLSMP(1) ;
	//ADC1_CFG1 |=  ADC_CFG1_ADICLK(0)| ADC_CFG1_ADIV(3) | ADC_CFG1_MODE(3) | ADC_CFG1_ADLSMP(1) ;
	// 12 bit resolution
	ADC1_CFG1 |=  ADC_CFG1_ADICLK(0)| ADC_CFG1_ADIV(2) | ADC_CFG1_MODE(1) | ADC_CFG1_ADLSMP(1) ;

	// DMA is enabled and will assert the ADC DMA request during an ADC conversion complete event noted
	// when any of the SC1n[COCO] flags is asserted
	// ADC Software trigger mode: means that the ADC conversions are started by writing the channel number in the ADCx_SC1A register
	ADC0_SC2 |= ADC_SC2_DMAEN_MASK | ADC_SC2_ADTRG(0);
	ADC1_SC2 |= ADC_SC2_DMAEN_MASK | ADC_SC2_ADTRG(0);

	// Default longest sample time; 20 extra ADCK cycles; 24 ADCK cycles total ==> this results a very stable state compared to short sample time
	// High-speed conversion sequence selected with 2 additional ADCK cycles to total conversion time.
	// for the ADC1, the channels ADxxb are used
	ADC0_CFG2 = ADC_CFG2_ADLSTS(0) | ADC_CFG2_ADHSC_MASK ;
	//ADC0_CFG2 = ADC_CFG2_ADLSTS(2)  ;
	//ADC1_CFG2 = ADC_CFG2_ADLSTS(0) | ADC_CFG2_ADHSC_MASK | ADC_CFG2_MUXSEL(1) ;
	ADC1_CFG2 = ADC_CFG2_ADLSTS(2) | ADC_CFG2_ADHSC_MASK | ADC_CFG2_MUXSEL(1) ;

	// SC1A is used for both software and hardware trigger modes of operation.
	// Disable the module till the first read function == writing the number 31 in ADCH
	ADC0_SC1A = ADC_SC1_ADCH(31);
	ADC1_SC1A = ADC_SC1_ADCH(31);
	// Start the conversion of Channel 8 to start the DMA cycle
	ADC0_SC1A = ADC_SC1_ADCH(8);
	ADC1_SC1A = ADC_SC1_ADCH(4);
}



/* This function reads the channel given in the parantheses and returns the result saved in the ADCx_RA register
   The channel number is listed under ADCH in the ADCx_SC1n registerFor our case we used the channel ADC0_SE8  */
unsigned short ADC_read(unsigned char channel)
{   // writes to SC1A subsequently initiate a new conversion, if SC1[ADCH] contains a value other than all 1s. (other than disabled)
	ADC0_SC1A = (channel & ADC_SC1_ADCH_MASK) ;
	// Conversion in progress. wait for it to finish
	while(ADC0_SC2 & ADC_SC2_ADACT_MASK);
	// wait for the conversion to complete
	while(!(ADC0_SC1A & ADC_SC1_COCO_MASK));
	return ADC0_RA;
}

