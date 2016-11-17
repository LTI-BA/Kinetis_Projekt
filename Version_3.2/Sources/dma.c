/*
 * dma.c
 *
 *  Created on: 23.07.2016
 *      Author: Lanwer
 *  based on: AN4590, Using DMA to Emulate ADC Flexible Scan Mode on Kinetis K Series 
 */

#include "dma.h"

void DMA_init(void) {

	extern uint16_t vo[5];
	extern uint8_t adc_mux_voltage[TOTAL_CHANNEL_NUMBER_VOLTAGE];

	extern uint16_t cu[4];
	extern uint8_t adc_mux_current[TOTAL_CHANNEL_NUMBER_CURRENT];

	// ADC0 channels used for voltage sensing, saved in SRAM
	adc_mux_voltage[0] = 9;
	adc_mux_voltage[1] = 12;
	adc_mux_voltage[2] = 13;
	adc_mux_voltage[3] = 23;
	adc_mux_voltage[4] = 8;

	// ADC1 channels used for current sensing, saved in SRAM
	adc_mux_current[0] = 5;
	adc_mux_current[1] = 6;
	adc_mux_current[2] = 7;
	adc_mux_current[3] = 4;

	// Enable clock for DMAMUX and DMA
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
	SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;

// Configuring the DMA Channels 6 and 7 for the current values coming from ADC1

        //**** Setting the CHCGF register to 0, to start the configuration of the channel
	DMAMUX_CHCFG6 = 0;

	//**** Source address, constant buffer in SRAM
	DMA_TCD6_SADDR = (uint32_t) &adc_mux_current[0];

	//**** Destination address, ADC1 control register
	DMA_TCD6_DADDR = (uint32_t) &ADC1_SC1A;

	//**** Source address increment, data is 8-bit, 1 byte
	DMA_TCD6_SOFF = 0x01;

	//**** Destination address increment in bytes, no increment needed
	DMA_TCD6_DOFF = 0x00;

	//**** Source and destination data width specification, both source and destination are 8-bit
	DMA_TCD6_ATTR = DMA_ATTR_SSIZE(0) | DMA_ATTR_DSIZE(0);

	//**** Number of bytes for minor loop (one data transfer), ADC1 input setting value is
	//**** 8 bits long, so 1-byte transfer
	DMA_TCD6_NBYTES_MLNO = 0x01;

	//**** Channel linking and major loop setting reload value after major loop finish,
	//**** no linking after minor loop, major loop transfers number 0x04
	DMA_TCD6_CITER_ELINKNO = 0x04;

	//**** Channel linking and major loop setting, no linking after minor loop,
	//**** major loop transfers number 0x04
	DMA_TCD6_BITER_ELINKNO = 0x04;

	//**** Source address reload after major loop finish, must be subtracted from last
	//**** pointer value, sampling channel number is 3 each and 1 byte long, 1 x 4 = 4
	//**** and must be subtract -4
	DMA_TCD6_SLAST = -4;

	//**** Destination address reload after major loop finish, no address reload needed
	DMA_TCD6_DLASTSGA = 0x00;

	//**** Common channel setting, no linking after major loop, no IRQ request enable
	DMA_TCD6_CSR = 0x00;

	// ******************************************************************************************************************

	//**** Setting the CHCGF register to 0, to start the configuration of the channel
	DMAMUX_CHCFG7 = 0;

	//**** Source address, ADC0_RA
	DMA_TCD7_SADDR = (uint32_t) &ADC1_RA;

	//**** Destination address, SRAM buffer [0]
	DMA_TCD7_DADDR = (uint32_t) &cu[0];

	//**** Source address increment; data is still read from the same address, no increment needed
	DMA_TCD7_SOFF = 0x00;

	//**** Destination address increment in bytes, increment for next buffer address
	//**** 16 bit => 2 bytes
	DMA_TCD7_DOFF = 0x02;

	//**** Source and destination data width specification, both source and destination is 16-bit
	DMA_TCD7_ATTR = DMA_ATTR_SSIZE(1) | DMA_ATTR_DSIZE(1);

	//**** Number of bytes for minor loop (one data transfer), ADC0 result is 16 bits long, so
	//**** 2-byte transfer
	DMA_TCD7_NBYTES_MLNO = 0x02;

	//**** Channel linking and major loop setting reload value after major loop finishes,
	//**** linking after minor loop is enabled, major loop transfers number 4 (0x04).
	DMA_TCD7_CITER_ELINKYES = (DMA_CITER_ELINKYES_ELINK_MASK
			| DMA_CITER_ELINKYES_LINKCH(6) | 0x04);

	//**** Channel linking and major loop setting, linking after minor loop is enabled to
	//**** channel 0 (0x0000), major loop transfers number 4 (0x04)
	DMA_TCD7_BITER_ELINKYES = (DMA_CITER_ELINKYES_ELINK_MASK
			| DMA_BITER_ELINKYES_LINKCH(6) | 0x04);

	//**** Source address reload after major loop finishes, no reload needed
	DMA_TCD7_SLAST = 0x00;

	//**** Destination address reload after major loop finishes,
	//**** must be subtracted from last pointer value, sample number is 4 each and 2 bytes long,
	//**** 2 x 4 = 8 and must be subtract -8
	DMA_TCD7_DLASTSGA = -8;

	//**** Common channel setting, linking after major loop enable to channel 2,
	//**** IRQ request is generated after major loop complete
	DMA_TCD7_CSR = DMA_CSR_INTMAJOR_MASK | DMA_CSR_MAJORELINK_MASK
			| DMA_CSR_MAJORLINKCH(6);

	//**** Channel 7 transfers ADC1 result data from ADC1_RA to SRAM buffer.
	//**** Enable Channel 3 and set ADC1 as DMA request source (source number 41)
	DMAMUX_CHCFG7 = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(41);

	//**** Channel 6 transfers next ADC1 input setting from constant buffer to ADC1_SC1A.
	//**** DMA transfer request source - always requestor
	DMAMUX_CHCFG6 = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(63);

// Configuring the DMA Channels 0 and 1 for the voltage values coming from ADC0

	//**** Setting the CHCGF register to 0, to start the configuration of the channel
	DMAMUX_CHCFG0 = 0;

	//**** Source address, constant buffer in SRAM
	DMA_TCD0_SADDR = (uint32_t) &adc_mux_voltage[0];

	//**** Destination address, ADC0 control register
	DMA_TCD0_DADDR = (uint32_t) &ADC0_SC1A;

	//**** Source address increment, data is 8-bit, 1 byte
	DMA_TCD0_SOFF = 0x01;

	//**** Destination address increment in bytes, no increment needed
	DMA_TCD0_DOFF = 0x00;

	//**** Source and destination data width specification, both source and destination are 8-bit
	DMA_TCD0_ATTR = DMA_ATTR_SSIZE(0) | DMA_ATTR_DSIZE(0);

	//**** Number of bytes for minor loop (one data transfer), ADC0 input setting value is
	//**** 8 bits long, so 1-byte transfer
	DMA_TCD0_NBYTES_MLNO = 0x01;

	//**** Channel linking and major loop setting reload value after major loop finish,
	//**** no linking after minor loop, major loop transfers number 0x03
	DMA_TCD0_CITER_ELINKNO = 0x05;

	//**** Channel linking and major loop setting, no linking after minor loop,
	//**** major loop transfers number 0x03
	DMA_TCD0_BITER_ELINKNO = 0x05;

	//**** Source address reload after major loop finish, must be subtracted from last
	//**** pointer value, sampling channel number is 3 each and 1 byte long, 1 x 5 = 5
	//**** and must be subtract -5
	DMA_TCD0_SLAST = -5;

	//**** Destination address reload after major loop finish, no address reload needed
	DMA_TCD0_DLASTSGA = 0x00;

	//**** Common channel setting, no linking after major loop, no IRQ request enable
	DMA_TCD0_CSR = 0x00;

	// ******************************************************************************************************************

	//**** Channel 1 transfers ADC0 result data from ADC0_RA to SRAM buffer.
	//*** Enable Channel 1 and set ADC0 as DMA request source (source number 40)
	DMAMUX_CHCFG1 = 0;

	//**** Source address, ADC0_RA
	DMA_TCD1_SADDR = (uint32_t) &ADC0_RA;

	//**** Destination address, SRAM buffer [0]
	DMA_TCD1_DADDR = (uint32_t) &vo[0];

	//**** Source address increment; data is still read for the same address, no increment needed
	DMA_TCD1_SOFF = 0x00;

	//**** Destination address increment in bytes, increment for next buffer address
	//**** 16 bit => 2 bytes
	DMA_TCD1_DOFF = 0x02;

	//**** Source and destination data width specification, both source and destination is 16-bit
	DMA_TCD1_ATTR = DMA_ATTR_SSIZE(1) | DMA_ATTR_DSIZE(1);

	//**** Number of bytes for minor loop (one data transfer), ADC0 result is 16 bits long, so
	//**** 2-byte transfer
	DMA_TCD1_NBYTES_MLNO = 0x02;

	//**** Channel linking and major loop setting reload value after major loop finishes,
	//**** linking after minor loop is enabled, major loop transfers number 5 (0x05).
	DMA_TCD1_CITER_ELINKYES = (DMA_CITER_ELINKYES_ELINK_MASK
			| DMA_CITER_ELINKYES_LINKCH(0) | 0x05);

	//**** Channel linking and major loop setting, linking after minor loop is enabled to
	//**** channel 0 (0x0000), major loop transfers number 5 (0x05)
	DMA_TCD1_BITER_ELINKYES = (DMA_BITER_ELINKYES_ELINK_MASK
			| DMA_BITER_ELINKYES_LINKCH(0) | 0x05);

	//**** Source address reload after major loop finishes, no reload needed
	DMA_TCD1_SLAST = 0x00;

	//**** Destination address reload after major loop finishes,
	//**** must be subtracted from last pointer value, sample number is 5 each and 2 bytes long,
	//**** 2 x 5 = 10 and must be subtract -10
	DMA_TCD1_DLASTSGA = -10;

	//**** Common channel setting, linking after major loop enable to channel 0,
	//**** IRQ request is generated after major loop complete
	DMA_TCD1_CSR = DMA_CSR_INTMAJOR_MASK | DMA_CSR_MAJORELINK_MASK
			| DMA_CSR_MAJORLINKCH(0);

	//**** Channel 1 transfers ADC0 result data from ADC0_RA to SRAM buffer.
	//**** Enable Channel 1 and set ADC0 as DMA request source (source number 40)
	DMAMUX_CHCFG1 = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(40);

	//**** Channel 0 transfers next ADC0 input setting from constant buffer to ADC0_SC1A.
	//**** DMA transfer request source - always requestor
	DMAMUX_CHCFG0 = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(63);

	//**** Enable request signal for channel 1 and 7
	DMA_ERQ = DMA_ERQ_ERQ7_MASK | DMA_ERQ_ERQ1_MASK;

}
