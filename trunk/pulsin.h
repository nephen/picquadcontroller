#ifndef PULSIN_H
#define PULSIN_H

//---------------------------------------------------------------------
// PULSIN (USING CN interrupts, Timer2)
//---------------------------------------------------------------------

#define PULSIN_LEN	4						//max number of pulsin pins

#define US_TO_TK2(tk) ((tk))				// us to Timer2 ticks

unsigned int pulsinPulse[PULSIN_LEN];		//array holding pulse length in Timer2 ticks


volatile unsigned int* pulsinPortPtr[PULSIN_LEN];		//Pointer to input port registers
unsigned char pulsinPortBit[PULSIN_LEN];				//Bit number in the port register
unsigned int pulsinTimer[PULSIN_LEN];					//timer2 value when ON state strted
unsigned char pulsinOn[PULSIN_LEN];						//signal is currently High
unsigned int pulsinPulse[PULSIN_LEN];					//last measured pulse in timer2 ticks
unsigned int pulsinOverflows[PULSIN_LEN];				//count overflows

#define PULSIN_ASSIGN_PIN(i,pin,CNIEBit)		pin(_TRIS_F) = 1; pulsinPortPtr[i] = 	pin(_PORT_PTR_F); \
												pulsinPortBit[i] = pin(_BIT_F); CNIEBit = 1;

void pulsin_init(){
	unsigned char i;
	for(i=0;i<PULSIN_LEN;i++){
		pulsinPulse[i] = 0;
		pulsinPortPtr[i] = 0;
		pulsinOn[i] = 1;		 //consider pulse is on, but we'll discard first pulse
		pulsinOverflows[i] = 2;	 //this will invalidate first pulse 
	}

	//Configure Timer2 1tick = 1us (8Mhz FCY)
	ConfigIntTimer2(T2_INT_ON & T2_INT_PRIOR_3);
 	OpenTimer2(T2_ON & T2_GATE_OFF & T2_IDLE_STOP &
               T2_PS_1_8 & 
               T2_SOURCE_INT, 0xFFFF
	);

	IFS1bits.CNIF = 0;		//Reset CN interrupt
	IEC1bits.CNIE = 1;		//Enable CN interrupts
}

void __attribute__((__interrupt__, auto_psv)) _T2Interrupt(void){
	unsigned char i;
	for(i=0;i<PULSIN_LEN;i++){
		pulsinOverflows[i]++;
	}
	
	
    IFS0bits.T2IF = 0;    // Clear Timer interrupt flag
}  


void __attribute__ ((__interrupt__, auto_psv)) _CNInterrupt(void)
{
	unsigned char i;
	for(i=0;i<PULSIN_LEN;i++){
		if(pulsinPortPtr[i]){
			if((*pulsinPortPtr[i]) & (1U << pulsinPortBit[i])){
				if(0 == pulsinOn[i]){
					pulsinTimer[i] = ReadTimer2();
					pulsinOn[i] = 1;
					pulsinOverflows[i] = 0;
				}
			}else{
				//need a startup value for pulsinOn to avoid startup glitches !
				if(1 == pulsinOn[i]){
					//note that it's ok if ReadTimer2() < pulsinTimer[i], overflow works correctly here
					//as long as timer2 period is 0xFFFF
					if(pulsinOverflows[i]<2){
						pulsinPulse[i] = ReadTimer2() - pulsinTimer[i];	  
					}else{
						pulsinPulse[i] = 0;
					}
					pulsinOn[i] = 0;		
				}
			}
		}
	}



	IFS1bits.CNIF = 0; // Clear CN interrupt
}


#endif
