#ifndef SOUND_H
#define SOUND_H

//---------------------------------------------------------------------
// SOUND (USING 2 CHANNEL PWM MODULE 2, Timer3)
//---------------------------------------------------------------------
#define US_TO_TK3(us) ((us))	// convert uS to Timer3 ticks 

unsigned int soundLoops;
unsigned int soundDurationMs;
unsigned int soundDurationOnMs;
unsigned int soundDurationOffMs;

//this macro is used so that led follows sound 
#define SOUND_ENABLE(v)	P2TCONbits.PTEN = (v); _PORT(pinLed) = !P2TCONbits.PTEN;

void sound_init(){
	//setup PWM ports
	PWM2CON1 = 0;			//clear all bits (use defaults)
	PWM2CON1bits.PMOD1 = 0; //PWM1Ly,PWM1Hy are in independent running mode
	PWM2CON1bits.PEN1L = 1; //PWM1L1 PWM OUTPUT
	PWM2CON1bits.PEN1H = 0; //PWM1H1 NORMAL I/O

	//PWM mode and prescaler
	P2TCON = 0;
	P2TCONbits.PTMOD = 0b00;			//Free-runing mode		
	P2TCONbits.PTCKPS = 0b11; 			// 1:64 prescaler


	P2TMR = 0;
	SOUND_ENABLE(0);
	soundLoops = 0;
}

void sound_stop(){
	soundLoops = 0;
	SOUND_ENABLE(0);
}

#define sound(freqHz,durationMs) sound_loop(freqHz,durationMs,0,1)

//repeates a  SOUND-PAUSE sequence 
unsigned char sound_loop(unsigned int freqHz,unsigned int durationOnMs,unsigned int durationOffMs,unsigned int loops){
	if(soundLoops > 0) return 0; 	//must stop / complete pervious sound before starting a new one

	if(freqHz>0){
		P2TPER = FCY  / 64 / freqHz -1;
		P2DC1 = (P2TPER+1);  //50% <=> P2TPER since duty cycle resolution is Tcy/2
	}else{
		P2DC1 = 0;				//silence
	}

	soundDurationOnMs = durationOnMs;
	soundDurationOffMs = durationOffMs;
	soundDurationMs = soundDurationOnMs;
	soundLoops = loops;
	TMR3 = 0;

	SOUND_ENABLE(1);

	//Configure Timer2 1tick = 1us (8Mhz FCY)
	ConfigIntTimer3(T1_INT_ON & T1_INT_PRIOR_1);
 	OpenTimer3(T3_ON & T3_GATE_OFF & T3_IDLE_STOP &
               T3_PS_1_8 & 
               T3_SOURCE_INT, US_TO_TK3(1000) //interrupt every 1mS
	);

	return 1;
}



void __attribute__((__interrupt__, auto_psv)) _T3Interrupt(void){
	if(soundDurationMs>0){
		soundDurationMs--;
	}else{
		if(soundLoops > 1){
			soundLoops--;
			SOUND_ENABLE(!P2TCONbits.PTEN);		//swap value of P2TCONbits.PTEN 
			soundDurationMs = P2TCONbits.PTEN ? soundDurationOnMs : soundDurationOffMs;
		}else{
			soundLoops = 0;
			CloseTimer3();
			P2TCONbits.PTEN = 0;
		}
	}
    IFS0bits.T3IF = 0;    // Clear Timer interrupt flag
}  


#endif
