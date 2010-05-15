#ifndef LED_H
#define LED_H

//---------------------------------------------------------------------
// LED
//---------------------------------------------------------------------

unsigned int ledState = 0;
volatile unsigned long ledCounter = 0;

#define LED_ON	0xFFFF			//led on, normal operation
#define LED_OFF	0				//led off, not used 
#define BLINK_SLOW	50			//interval in 10ms, startup / calibration
#define BLINK_FAST	5			//interval in 10ms, error/panic condition

void led();

//Setup pin port, and timer used for led blinking
void led_init(){
	_TRIS(pinLed) = 0;

	//Configure Timer1 5ticks = 1us (40Mhz FCY, 1:8 prescaler)
 	OpenTimer1(T1_ON & T1_GATE_OFF & T1_IDLE_STOP &
               T1_PS_1_8 & 
               T1_SOURCE_INT, 50000	//interrupt every 10ms
	);

	led(LED_ON);
}

void led(unsigned int state){
	if(ledState != state){
		ledState = state;
		if(LED_ON == state){ 
			_LAT(pinLed) = 1;
			ConfigIntTimer1(T1_INT_OFF & T1_INT_PRIOR_3);
		}else if (LED_OFF == state){
			_LAT(pinLed) = 0;
			ConfigIntTimer1(T1_INT_OFF & T1_INT_PRIOR_3);
		}else{
			_LAT(pinLed) = 1;
			ledCounter = 0;
			TMR1 = 0;
			ConfigIntTimer1(T1_INT_ON & T1_INT_PRIOR_3);
		}
	}
}


//this interrupt counts timer overflows for each channel
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void){
	ledCounter++;
	if(ledCounter > ledState){
		ledCounter = 0;
		_LAT(pinLed) = !_LAT(pinLed);
	}
    IFS0bits.T1IF = 0;    // Clear Timer interrupt flag
}  


#endif
