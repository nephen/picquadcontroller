#include "p33FJ128MC802.h"

#include <libpic30.h>

#include "oscilator.h"

#include <pwm.h>
#include <timer.h>
#include <pwm.h>
#include<uart.h>
#include <stdio.h>
#include <adc.h>

//---------------------------------------------------------------------
// I/O PINS
//---------------------------------------------------------------------


#define pinLed(f)  		f(B,5)		//Led, OUTPUT



//#define pinSound(f) 	f(B,9)		//Sound, PWM OUTPUT


#define pinRX(f)  		f(B,7)		//RP7, UART RX, INPUT , IF CHANGED UPDATE uart_init()
#define pinTX(f)  		f(B,4)		//RP4, UART TX, OUTPUT, IF CHANGED UPDATE uart_init()

/*
#define pinEN1(f) 		f(B,14)		//Motor 1 Enable (PWM), OUTPUT
#define pinMA1(f)		f(B,15)		//Motor 1 Terminal A , OUTPUT
#define pinMB1(f)		f(B,11)		//Motor 1 Terminal B , OUTPUT


#define pinEN2(f) 		f(B,12)		//Motor 2 Enable (PWM), OUTPUT
#define pinMA2(f)		f(B,13)		//Motor 2 Terminal A, OUTPUT
#define pinMB2(f)		f(B,10)		//Motor 2 Terminal B, OUTPUT

#define pinServo0(f)	f(A,2)		//Servo 0
#define pinServo1(f)	f(A,3)		//Servo 1

#define pinPulsin0(f)  	f(A,4)		//CN0
#define pinPulsin1(f)  	f(B,4)		//CN1
*/


//---------------------------------------------------------------------
// LIBS
//---------------------------------------------------------------------

#include "macroutil.h"
#include "adcutil.h"

//#include "pulsin.h"
//#include "pulsout.h"
//#include "sound.h"
//#include "motor.h"
#include "uartutil.h"
//#include "rccontrol.h"


//---------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------


int main (void)
{
	//---------------------------------------------------------------------
	// OSCILATOR
	//---------------------------------------------------------------------
	oscilator_init();



	//---------------------------------------------------------------------
	// I/O PORTS
	//---------------------------------------------------------------------
	_TRIS(pinLed) = 0;
	_LAT(pinLed) = 1;


	//---------------------------------------------------------------------
	// ADC
	//---------------------------------------------------------------------
	adc_init();

	//---------------------------------------------------------------------
	// UART
	//---------------------------------------------------------------------
	uart_init();

	printf("Start\n");




	//---------------------------------------------------------------------
	// BLINK/UART TEST
	//---------------------------------------------------------------------


	while(1){
		__delay_ms(1001);
		//_LAT(pinLed) = !_LAT(pinLed);

		unsigned long interval_us = adc_new_data();

		if(interval_us){
			printf("interval_us = %lu\n" , interval_us);
			adc_debug_dump();		
		}
	}

/*


	//---------------------------------------------------------------------
	// PULSOUT
	//---------------------------------------------------------------------
	pulsout_init();
	PULSEOUT_ASSIGN_PIN(0,pinServo0);
	PULSEOUT_ASSIGN_PIN(1,pinServo1);


	//---------------------------------------------------------------------
	// PULSIN
	//---------------------------------------------------------------------
	pulsin_init();
	PULSIN_ASSIGN_PIN(0,pinPulsin0,CNEN1bits.CN0IE);
	PULSIN_ASSIGN_PIN(1,pinPulsin1,CNEN1bits.CN1IE);

	//---------------------------------------------------------------------
	// MOTOR
	//---------------------------------------------------------------------
	motor_init();


	//---------------------------------------------------------------------
	// LOOP
	//---------------------------------------------------------------------

	printf("\n*** BotController ***\n");

	//rc_main_loop();

	

	while (1){
		//power_check();

		int pos = uart_read_packet();
		int i;


		if(pos>=0){
			_LAT(pinLed) = 0;

			//printf("-{%u}",pos);	while(BusyUART1());
			__delay_ms(20);

			WriteUART1(0b01010101);while(BusyUART1());//send a byte  to "wake up" radio,this is not part of any the packet !
			for(i=0;i<PACKET_LEN;i++){
				WriteUART1(receiveBuffer[pos+i]);while(BusyUART1());	//send packet
			}
			WriteUART1(255);while(BusyUART1());	//END OF PACKET

			_LAT(pinLed) = 1;

		}


	};

*/
	return 0;
}

//---------------------------------------------------------------------
// THE END
//---------------------------------------------------------------------
