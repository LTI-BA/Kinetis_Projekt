/*
 * uart1.c
 *
 *  Created on: 02.05.2016
 *      Author: Michael
 */


//This is my own implementation of UART1 communication, to learn how kinetis works.

//Includes
#include <stdio.h>
#include "MK22F51212.h"
#include "system_MK22F51212.h"
#include "uart1.h"
#include "mcu_tracer.h"

//Configuration options
//=================================================================================
// UART to use
//
#define UART  UART1
#define UART_CLOCK SystemCoreClock

#define UART_DMA_RX  11
#define UART_DMA_TX  10

//
void UART1_init_defaultparam()  {
   // Enable clock to UART
   SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;

   // Enable clock to port pins used by UART
   SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

   // Select Tx & Rx pins to use
   SIM->SOPT5 &= ~(SIM_SOPT5_UART0RXSRC_MASK|SIM_SOPT5_UART0TXSRC_MASK);

   // Set Tx & Rx Pin function
   PORTE->PCR[0] = PORT_PCR_MUX(3);
   PORTE->PCR[1] = PORT_PCR_MUX(3);
}

/*
 * Initialises the UART
 *
 * @param baudrate - the baud rate to use e.g. DEFAULT_BAUD_RATE
 */

void UART1_startup(int baudrate) {
	UART1_init_defaultparam();

   // Disable UART before changing registers
   UART->C2=0;

   // Calculate UART clock setting (5-bit fraction at right)
   int scaledBaudValue = (2*UART_CLOCK)/(baudrate);

   // Set Baud rate register
   UART->BDH = (UART->BDH&~UART_BDH_SBR_MASK) | UART_BDH_SBR((scaledBaudValue>>(8+5)));
   UART->BDL = UART_BDL_SBR(scaledBaudValue>>5);
   // Fractional divider to get closer to the baud rate
   UART->C4 = (UART->C4&~UART_C4_BRFA_MASK) | UART_C4_BRFA(scaledBaudValue);

   //Activate Fifo
   UART->C2|=UART_C2_TIE_MASK|UART_C2_RIE_MASK; //Enable transmit mask
   //Activate DMA transfer request
   UART->C5|=UART_C5_TDMAS_MASK|UART_C5_RDMAS_MASK;

   UART1_dmaconfig();
   UART1_dma_config_rec();
   UART->PFIFO |= (UART_PFIFO_TXFE_MASK);

   //DMA





   // Enable UART Tx & Rx
   UART->C2|= UART_C2_TE_MASK|UART_C2_RE_MASK;
   //framing, parity, noise, overrun interrupt enable
   UART->C3= UART_C3_FEIE_MASK|UART_C3_PEIE_MASK|UART_C3_NEIE_MASK|UART_C3_ORIE_MASK;
   NVIC_EnableIRQ(UART1_ERR_IRQn);
}

void UART1_txChar(int ch) {
   while ((UART->S1 & UART_S1_TDRE_MASK) == 0) {
      // Wait for Tx buffer empty
      __asm__("nop");
   }
   UART->D = ch;
}




//Make the buffer large enough, so that a overflow never occures.
//If overflow occurs, data is lost.
#define REC_BUF_SIZE 200
uint8_t  recievebuf[REC_BUF_SIZE];
uint8_t  dummyzero=0;
uint16_t volatile recpos;

void UART1_dmaconfig(void){
	//Configures the DMA to feed UART Transmitter



	SIM_SCGC6|=SIM_SCGC6_DMAMUX_MASK;
	SIM_SCGC7|=SIM_SCGC7_DMA_MASK;

	DMAMUX_CHCFG_REG(DMAMUX,UART_DMA_TX)=0; //set to zero during configuration

	//Enable request signal for channel 1
	DMA0->ERQ|=(1<<UART_DMA_TX);

	//Set memory address
	DMA0->TCD[UART_DMA_TX].SADDR=(uint32_t) &dummyzero;
	DMA0->TCD[UART_DMA_TX].DADDR=(uint32_t) &(UART->D);

	//Offset for source and destination
	DMA0->TCD[UART_DMA_TX].SOFF=1; //We increment the address by one, after each cycle
	DMA0->TCD[UART_DMA_TX].DOFF=0;

	//Source and destination data transfer size
	DMA0->TCD[UART_DMA_TX].ATTR=DMA_ATTR_SSIZE(0)|DMA_ATTR_DSIZE(0);


	DMA0->TCD[UART_DMA_TX].NBYTES_MLNO=1;

	DMA0->TCD[UART_DMA_TX].CITER_ELINKNO=DMA_CITER_ELINKNO_CITER(1);
	DMA0->TCD[UART_DMA_TX].BITER_ELINKNO=DMA_BITER_ELINKNO_BITER(1);

	DMAMUX_CHCFG_REG(DMAMUX,UART_DMA_TX)=DMAMUX_CHCFG_ENBL_MASK|DMAMUX_CHCFG_SOURCE(5); //always enabled
	DMA0->TCD[UART_DMA_TX].CSR=DMA_CSR_DREQ_MASK;




/*
	if(DMA0->ES){
		//Page 396
		if(DMA0->ES & (1<<0)) {
			//Destination Bus error
			redLedOn();
		}
		if(DMA0->ES & (1<<1)) {
			//Source Bus error
			redLedOn();
		}
		if(DMA0->ES & (1<<2)) {
			//Scatter/Gather Configuration Error
			redLedOn();
		}
		if(DMA0->ES & (1<<3)) {
			//NBYTES/CITER configuration Error
			redLedOn();
		}
		if(DMA0->ES & (1<<4)) {
			//Destination Offset error
			redLedOn();
		}
		if(DMA0->ES & (1<<5)) {
			//Destination Address Error
			redLedOn();
		}
		if(DMA0->ES & (1<<6)) {
			//Source offset error
			redLedOn();
		}
		if(DMA0->ES & (1<<7)) {
			//Source address error
			redLedOn();
		}
	}else{
		redLedOff();
	}*/

}

int UART1_dma_complete(void){
	//Channel Done wait
	return ((DMA0->TCD[UART_DMA_TX].CSR) & DMA_CSR_DONE_MASK);
}

void UART1_dma_complete_wait(void){
	while(!((DMA0->TCD[UART_DMA_TX].CSR) & DMA_CSR_DONE_MASK)){
		if((UART->S1 & UART_S1_TDRE_MASK)) {
			if((UART->S1 & UART_S1_TDRE_MASK)) {
				break; //give two reads before quiting.
			}
		}
	}
}

void UART1_txBulk(uint8_t *data, uint16_t len){
	//Wait until dma is complete
	UART1_dma_complete_wait();
	DMAMUX_CHCFG_REG(DMAMUX,UART_DMA_TX)=0; //set to zero during configuration

	//Enable request signal for channel 1
	DMA0->ERQ|=(1<<UART_DMA_TX);

	//Set memory address
	DMA0->TCD[UART_DMA_TX].SADDR=(uint32_t) (data);
	DMA0->TCD[UART_DMA_TX].DADDR=(uint32_t) &(UART->D);

	//Source and destination data transfer size
	DMA0->TCD[UART_DMA_TX].ATTR=DMA_ATTR_SSIZE(0)|DMA_ATTR_DSIZE(0);
	//Set number of minor loops
	DMA0->TCD[UART_DMA_TX].CITER_ELINKNO=DMA_CITER_ELINKNO_CITER(len);
	DMA0->TCD[UART_DMA_TX].BITER_ELINKNO=DMA_BITER_ELINKNO_BITER(len);

	DMAMUX_CHCFG_REG(DMAMUX,UART_DMA_TX)=DMAMUX_CHCFG_ENBL_MASK|DMAMUX_CHCFG_SOURCE(5); //always enabled
	DMA0->TCD[UART_DMA_TX].CSR=DMA_CSR_DREQ_MASK;
}

void UART1_dma_config_rec(void){

	DMAMUX_CHCFG_REG(DMAMUX,UART_DMA_RX)=0; //set to zero during configuration

	//Enable request signal for channel 1
	DMA0->ERQ|=(1<<UART_DMA_RX);

	//Set memory address
	DMA0->TCD[UART_DMA_RX].SADDR=(uint32_t) &(UART->D);
	DMA0->TCD[UART_DMA_RX].DADDR=(uint32_t) &recievebuf;

	//Offset for source and destination
	DMA0->TCD[UART_DMA_RX].SOFF=0; //We increment the address by one, after each cycle
	DMA0->TCD[UART_DMA_RX].DOFF=1;

	//Source and destination data transfer size
	DMA0->TCD[UART_DMA_RX].ATTR=DMA_ATTR_SSIZE(0)|DMA_ATTR_DSIZE(0);

	DMA0->TCD[UART_DMA_RX].NBYTES_MLNO=1;

	DMA0->TCD[UART_DMA_RX].CITER_ELINKNO=DMA_CITER_ELINKNO_CITER(REC_BUF_SIZE);
	DMA0->TCD[UART_DMA_RX].BITER_ELINKNO=DMA_BITER_ELINKNO_BITER(REC_BUF_SIZE);

	DMAMUX_CHCFG_REG(DMAMUX,UART_DMA_RX)=DMAMUX_CHCFG_ENBL_MASK|DMAMUX_CHCFG_SOURCE(4); //page 77
	DMA0->TCD[UART_DMA_RX].CSR=0;
	DMA0->TCD[UART_DMA_RX].SLAST=0;
	DMA_TCD2_DLASTSGA=-REC_BUF_SIZE;

	recpos=0;
/*
	if(DMA0->ES){
		//Page 396
		if(DMA0->ES & (1<<0)) {
			//Destination Bus error
			redLedOn();
		}
		if(DMA0->ES & (1<<1)) {
			//Source Bus error
			redLedOn();
		}
		if(DMA0->ES & (1<<2)) {
			//Scatter/Gather Configuration Error
			redLedOn();
		}
		if(DMA0->ES & (1<<3)) {
			//NBYTES/CITER configuration Error
			redLedOn();
		}
		if(DMA0->ES & (1<<4)) {
			//Destination Offset error
			redLedOn();
		}
		if(DMA0->ES & (1<<5)) {
			//Destination Address Error
			redLedOn();
		}
		if(DMA0->ES & (1<<6)) {
			//Source offset error
			redLedOn();
		}
		if(DMA0->ES & (1<<7)) {
			//Source address error
			redLedOn();
		}
	}else{
		redLedOff();
	}*/
}

uint16_t UART1_rec_size(void){
	return ((DMA0->TCD[UART_DMA_RX].BITER_ELINKNO)-(DMA0->TCD[UART_DMA_RX].CITER_ELINKNO));
}

void UART1_rec_buf_reset(void){
	DMAMUX_CHCFG_REG(DMAMUX,UART_DMA_RX)=0; //set to zero during configuration

	//Enable request signal for channel 1
	DMA0->ERQ|=(1<<UART_DMA_RX);

	//Set memory address
	DMA0->TCD[UART_DMA_RX].SADDR=(uint32_t) &(UART->D);
	DMA0->TCD[UART_DMA_RX].DADDR=(uint32_t) &recievebuf;

	//Offset for source and destination
	DMA0->TCD[UART_DMA_RX].SOFF=0; //We increment the address by one, after each cycle
	DMA0->TCD[UART_DMA_RX].DOFF=1;

	//Source and destination data transfer size
	DMA0->TCD[UART_DMA_RX].ATTR=DMA_ATTR_SSIZE(0)|DMA_ATTR_DSIZE(0);

	DMA0->TCD[UART_DMA_RX].NBYTES_MLNO=1;

	DMA0->TCD[UART_DMA_RX].CITER_ELINKNO=DMA_CITER_ELINKNO_CITER(REC_BUF_SIZE);
	DMA0->TCD[UART_DMA_RX].BITER_ELINKNO=DMA_BITER_ELINKNO_BITER(REC_BUF_SIZE);

	DMAMUX_CHCFG_REG(DMAMUX,UART_DMA_RX)=DMAMUX_CHCFG_ENBL_MASK|DMAMUX_CHCFG_SOURCE(4); //page 77
	DMA0->TCD[UART_DMA_RX].CSR=0;
	DMA0->TCD[UART_DMA_RX].SLAST=0;
	DMA_TCD2_DLASTSGA =-REC_BUF_SIZE;
	recpos=0;
}

//returns the elements in the buffer
char UART1_buffercontent(void){
	return UART1_rec_size()-recpos;
}

char UART1_getch(void){
	char returnchar;
	returnchar=recievebuf[recpos++];
    if(recpos>=UART1_rec_size()){
    	UART1_rec_buf_reset();
    }
	//returnchar=(recpos)+'0';
	//recpos=recpos+1;
	return returnchar;
}

extern uint32_t led_lauflicht;
void UART1_ERR_IRQHandler() {
   // Clear & ignore any pending errors
   if ((UART->S1 & (UART_S1_FE_MASK|UART_S1_OR_MASK|UART_S1_PF_MASK|UART_S1_NF_MASK)) != 0) {
	  led_lauflicht=2;
	  // Discard data (& clear status)
      (void)UART->D;
      mcu_tracer_config();
   }
}
