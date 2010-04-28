#ifndef PULSOUT_H
#define PULSOUT_H

//---------------------------------------------------------------------
// PULSOUT - SERVO OUTPUT (USING TIMER1)
//---------------------------------------------------------------------
#define PULSOUT_LEN	4

#define US_TO_TK1(us) ((us))								// convert uS to Timer1  ticks 

#define PULSOUT_INTERVAL_US	20000							//interval for standard servos

unsigned int pulsoutPulse[PULSOUT_LEN];						//array holding pulse length in Timer1 ticks
unsigned char pulsoutIndex = 0;								//currently processed pulse index
unsigned int pulsoutPause ; 								//pause in Timer1 ticks (after all pulses have been sent)

volatile unsigned int* pulsoutLatchPtr[PULSOUT_LEN];		//Pointer to output latches registers
unsigned char pulsoutLatchBit[PULSOUT_LEN];					//Bit number in the output latch register


#define PULSEOUT_ASSIGN_PIN(i,pin)		pin(_TRIS_F) = 0; pulsoutLatchPtr[i] = 	pin(_LAT_PTR_F); pulsoutLatchBit[i] = pin(_BIT_F)
void pulsout_init(){
	unsigned char i;
	for(i=0;i<PULSOUT_LEN;i++){
		pulsoutPulse[i] = US_TO_TK1(1500);
		pulsoutLatchPtr[i] = 0;
	}

	pulsoutIndex = 0;	

	//Configure Timer1 1tick = 1us (8Mhz FCY)
	ConfigIntTimer1(T1_INT_ON & T1_INT_PRIOR_3);
 	OpenTimer1(T1_ON & T1_GATE_OFF & T1_IDLE_STOP &
               T1_PS_1_8 & T1_SYNC_EXT_OFF &
               T1_SOURCE_INT, PULSOUT_INTERVAL_US
	);

}


void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void){
	unsigned char pulsoutIndexLast = pulsoutIndex-1;
	
	//bring last pulse low
	if(pulsoutIndex > 0){
		 if(pulsoutLatchPtr[pulsoutIndexLast])
			(*pulsoutLatchPtr[pulsoutIndexLast]) &= !(1U << pulsoutLatchBit[pulsoutIndexLast]);	//clear latch bit
	}else{
		pulsoutPause = US_TO_TK1(PULSOUT_INTERVAL_US);
	}

	if(pulsoutIndex < PULSOUT_LEN){
		//bring new pulse up
		PR1 = pulsoutPulse[pulsoutIndex];
		if(pulsoutLatchPtr[pulsoutIndex])
			(*pulsoutLatchPtr[pulsoutIndex]) |= 1 << pulsoutLatchBit[pulsoutIndex];		//set latch bit
		pulsoutPause -= pulsoutPulse[pulsoutIndex];
		pulsoutIndex++;
	}else{
		//pause for the remaining time
		PR1 = pulsoutPause;
		pulsoutIndex = 0;
	}

	
	
    IFS0bits.T1IF = 0;    // Clear Timer interrupt flag
}  




#endif
