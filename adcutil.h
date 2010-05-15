#ifndef ADCUTIL_H
#define ADCUTIL_H

//---------------------------------------------------------------------
// ADC
//---------------------------------------------------------------------

#define ADC_VREF_MV 3300.0	//ADC reference volatage in miliVolts

double adc_to_mv(double adc){
	return adc * ADC_VREF_MV / 1023.0f;
}


//DMA Buffer is splited in segments each segments contains sample from one the channels , AN0..AN6
//[AN0 SAMPLE 1,AN0 SAMPLE 2, ... ] ,  [AN1 SAMPLE 1,AN1 SAMPLE 2, ... ] , ...

#define DMA_SEG_COUNT 6		//Number of segments in DMA buffer (one for each channel)
#define DMA_SEG_LEN	  8		//Lenght of each segments in words (number of samples per channel, stored in DMA buffer at one time)
#define DMA_TOTAL_LEN (DMA_SEG_COUNT*DMA_SEG_LEN)	//=48 Total length of DMA buffer

unsigned int adcDmaA[DMA_SEG_COUNT][DMA_SEG_LEN] __attribute__((space(dma),aligned(128)));	//IMPORTANT: align data to a power of 2 >= DMA_TOTAL_LEN*2 
unsigned int adcDmaB[DMA_SEG_COUNT][DMA_SEG_LEN] __attribute__((space(dma),aligned(128)));	//see http://ww1.microchip.com/downloads/en/DeviceDoc/70182b.pdf, page 22-31


#if 128 <  DMA_TOTAL_LEN * 2
	#error "Update aligned(..) above"; //memo if DMA_TOTAL_LEN changes
#endif


unsigned long adc_new_data();

void adc_init(){
	TRISAbits.TRISA0 = 1;	//Configure analog ports as inputs
	TRISAbits.TRISA1 = 1;	//
	TRISAbits.TRISA2 = 1;	//
	TRISBbits.TRISB1 = 1;	//
	TRISBbits.TRISB2 = 1;	//
	TRISBbits.TRISB3 = 1;	//

	AD1PCFGL = 0b1111111111000000;	//Configure ports  AN0..AN6 as analog, all others as digital (on POR all ports are analog by default)
	AD1CSSL =  0b00111111;	//set inputs to be scanned AN0..AN6

	AD1CON1 = 0;			//defaults (see datasheet)
	AD1CON1bits.SSRC =0b111;//Auto Convert
	AD1CON1bits.ASAM = 1;	//Auto-sample after each conversion

	AD1CON2 = 0; 	 						//defaults (see datasheet), 
	AD1CON2bits.CSCNA = 1;					//scan mode
	AD1CON2bits.SMPI = DMA_SEG_COUNT -1;	//6 segments in DMA buffer 
										
			
	AD1CON3 = 0;	 		//defaults (see datasheet)

	#define ADCS_PARAM 60 -1
	AD1CON3bits.ADCS =ADCS_PARAM; 	//Tad = (ADCS<5:0>  + 1) * Tcy  = (ADCS + 1) / Fcy  = 60 / 40Mhz  = 1.5uS (min required Tad is 76nS for 10-bit mode)
	#define SAMC_PARAM 30
	AD1CON3bits.SAMC = SAMC_PARAM;	//sample time =  SAMC<4:0> * Tad  =  30 * 1.5uS = 45 uS  
									//Total sample+conversion time per chanel = 12(conversion) + 30(sample) = 42Tad * 1.5us = 63uS 

	#define DMA_INTERVAL_US  (63 * DMA_TOTAL_LEN) 	// = 63 * 48 = 3024 us , this should be integer, otherwise use conversion to integer

	#if ADCS_PARAM != 60 -1 || SAMC_PARAM != 30 || DMA_SEG_COUNT != 6 || DMA_SEG_LEN !=8
		#error "Update DMA_INTERVAL_US above"
	#endif

	
	AD1CON4bits.DMABL=0b011;//DMA buffer has 2^DMABL = 8 words per channel (per segment), 
							
	#if 8 != DMA_SEG_LEN
		#error "Update AD1CON4 above"; //memo if DMA_SEG_LEN changes
	#endif


	AD1CHS0 = 0;	 		//defaults (see datasheet)
	AD1CHS123= 0;			//defaults (see datasheet)


	//Setup DMA buffer for ADC
	//See: http://ww1.microchip.com/downloads/en/DeviceDoc/70182b.pdf
	//Also sample: http://ww1.microchip.com/downloads/en/DeviceDoc/70183A.pdf, page 16-56

	DMA0CONbits.AMODE =0b10;					// Configure DMA for Peripheral indirect addressing mode
	DMA0CONbits.MODE = 0b10;					// Configure DMA for Continuous, Ping-Pong mode 
	DMA0PAD = (volatile unsigned int)&ADC1BUF0;	// Point DMA to ADC1BUF0
	DMA0CNT = DMA_TOTAL_LEN-1; 					// DMA transfer count (6 segments, each with 4 words)
	DMA0REQ =13;								// Select ADC1 as DMA Request source

	DMA0STA = __builtin_dmaoffset(&adcDmaA);
	DMA0STB = __builtin_dmaoffset(&adcDmaB);

	IFS0bits.DMA0IF = 0;	//Clear the DMA interrupt flag bit
	IEC0bits.DMA0IE = 1;	//Set the DMA interrupt enable bit
	DMA0CONbits.CHEN=1; 	// Enable DMA

	
	AD1CON1bits.ADON = 1;	//Enable ADC 
	
	while(! adc_new_data());//wait for first set of data

}


unsigned int (*adcDmaPtr)[DMA_SEG_COUNT][DMA_SEG_LEN]; 
#define adcDma (*adcDmaPtr)				//convinient macro to access last updated buffer
volatile unsigned int adcDmaAge = 0;	//number of times interrupt was call since last succesful adc_new_data

void __attribute__((__interrupt__,no_auto_psv)) _DMA0Interrupt(void)
{
	//_LAT(pinLed) = DMACS1bits.PPST0;	//for debug

	if(adcDmaAge<0xFFFF) adcDmaAge++;  		

	if(DMACS1bits.PPST0)				//adcDMAptr stores last access buffer by Ping-Pong DMA mode
		adcDmaPtr = &adcDmaA;
	else
		adcDmaPtr = &adcDmaB;


	IFS0bits.DMA0IF = 0; //Clear the DMA0 Interrupt Flag
}


double adcAvg[DMA_SEG_LEN]; 
unsigned int adcMin[DMA_SEG_LEN]; 
unsigned int adcMax[DMA_SEG_LEN]; 

//if no new data, returns 0
//if there's data, function returns interval in us since last succesful call to adc_new_data
unsigned long adc_new_data(){
	unsigned long interval_us = 0;
	if(adcDmaAge > 0){
		interval_us = (unsigned long)adcDmaAge * DMA_INTERVAL_US;	//how much time passed since last succesful call to this function
		adcDmaAge = 0;												//reset, will be incremented by DMA interupt

		//compute average ADC values
		unsigned char i,j;
		for(i=0;i<DMA_SEG_COUNT;i++){
			adcAvg[i] = 0;
			for(j=0;j<DMA_SEG_LEN;j++){
				if(0==j){
					adcMin[i]= adcDma[i][j];
					adcMax[i]= adcDma[i][j];
				}else{
					if(adcDma[i][j]<adcMin[i]) adcMin[i]= adcDma[i][j];
					if(adcDma[i][j]>adcMax[i]) adcMax[i]= adcDma[i][j];
				}
				adcAvg[i] += adcDma[i][j];				//use pointer to last updated buffer adcDmaA or adcDmaB, that is updated by DMA interrupt
			}
			adcAvg[i] /= DMA_SEG_LEN;
		};		
	}

	return interval_us;
}


//this is used for ADC debug
void adc_debug_dump(){
	unsigned char i,j;
	//printf("adcDMAptr=%u adcDmaAge= %u\n", (unsigned int)adcDmaPtr, adcDmaAge);
	for(i=0;i<DMA_SEG_COUNT;i++){
		printf("an%u A=",i);
		for(j=0;j<DMA_SEG_LEN;j++){
			printf("%04u ",adcDmaA[i][0]);
		}
		printf(" adcAvg=%#6.1f (%#6.1f mV) \n",adcAvg[i],adc_to_mv(adcAvg[i]));
		printf("an%u B=",i);
		for(j=0;j<DMA_SEG_LEN;j++){
			printf("%04u ",adcDmaB[i][0]);
		}
		printf(" adcMin=%u adcMax=%u\n",adcMin[i],adcMax[i]);
	};
	printf("\n\n" );
}




#endif
