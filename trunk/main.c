#include "p33FJ128MC802.h"

#include <libpic30.h>

#include "oscilator.h"

#include <pwm.h>
#include <timer.h>
#include <pwm.h>
#include<uart.h>
#include <stdio.h>
#include <adc.h>
#include <math.h>

//---------------------------------------------------------------------
// I/O PINS
//---------------------------------------------------------------------


#define pinLed(f)  		f(B,5)		//Led, OUTPUT

//#define pinSound(f) 	f(B,9)		//Sound, PWM OUTPUT


#define pinRX(f)  		f(B,7)		//RP7, UART RX, INPUT , IF CHANGED UPDATE uart_init()
#define pinTX(f)  		f(B,4)		//RP4, UART TX, OUTPUT, IF CHANGED UPDATE uart_init()

#define pinPulsin0(f)  	f(B,15)	
#define pinPulsin0CN	CNEN1bits.CN11IE	
#define pinPulsin1(f)  	f(B,13)		
#define pinPulsin1CN	CNEN1bits.CN13IE
#define pinPulsin2(f)  	f(B,11)		
#define pinPulsin2CN	CNEN1bits.CN15IE
#define pinPulsin3(f)  	f(B,9)
#define pinPulsin3CN	CNEN2bits.CN21IE

#define pinMotor0(f)  	f(B,14)	//Front Motor, PWM1H1
#define pinMotor1(f)  	f(B,12)	//Back Motor, PWM1H2
#define pinMotor2(f)  	f(B,10)	//Left Motor, PWM1H1
#define pinMotor3(f)  	f(B,8)	//Right Motor, PWM2H1

//---------------------------------------------------------------------
// LIBS
//---------------------------------------------------------------------

#include "macroutil.h"
#include "adcutil.h"
#include "pulsin.h"
#include "motor.h"
#include "uartutil.h"
#include "imu.h"
#include "led.h"

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
	// LED
	//---------------------------------------------------------------------
	led_init();
	led(BLINK_SLOW);

	//---------------------------------------------------------------------
	// ADC
	//---------------------------------------------------------------------
	adc_init();

	//---------------------------------------------------------------------
	// UART
	//---------------------------------------------------------------------
	uart_init();
	printf("\n*** PicQuadController ***\n");

	//---------------------------------------------------------------------
	// ADC TEST
	//---------------------------------------------------------------------
	/*
	while(1){
		__delay_ms(1001);
		//_LAT(pinLed) = !_LAT(pinLed);

		unsigned long interval_us = adc_new_data();

		if(interval_us){
			printf("interval_us = %lu\n" , interval_us);
			adc_debug_dump();		
		}

	}
	*/

	//---------------------------------------------------------------------
	// PULSIN
	//---------------------------------------------------------------------
	pulsin_init();
	PULSIN_ASSIGN_PIN(0,pinPulsin0,pinPulsin0CN);
	PULSIN_ASSIGN_PIN(1,pinPulsin1,pinPulsin1CN);
	PULSIN_ASSIGN_PIN(2,pinPulsin2,pinPulsin2CN);
	PULSIN_ASSIGN_PIN(3,pinPulsin3,pinPulsin3CN);

	//---------------------------------------------------------------------
	// MOTOR
	//---------------------------------------------------------------------
	motor_init();

	//---------------------------------------------------------------------
	// MOTOR & PULSIN TEST
	//---------------------------------------------------------------------
	/*
	while(1){
		__delay_ms(100);

		pulsin_process();
		led(PULSIN_PANIC ? BLINK_FAST : LED_ON);
	

		printf("*PLS %05d %05d %05d %05d\n" , TK2_TO_US(pulsinPulse[0]),TK2_TO_US(pulsinPulse[1]),TK2_TO_US(pulsinPulse[2]),TK2_TO_US(pulsinPulse[3]));
		printf(" CMD  %04d  %04d  %04d  %04d FLT:%05d\n",cmd[ROL],cmd[PTC],cmd[THR],cmd[YAW], pulsinFaultCount );


		int i;
		for(i = 0 ; i<4 ; i ++){
			motorDuty[i] = MAX(cmd[THR],0);
		}
		motor_update_duty();


	}
	*/
	


	//---------------------------------------------------------------------
	// IMU
	//---------------------------------------------------------------------
	imu_init();



	//---------------------------------------------------------------------
	// MAIN LOOP
	//---------------------------------------------------------------------

	led(LED_ON);

	while (1){
		//unsigned char i;
		unsigned long interval_us = adc_new_data();

		pulsin_process();
		led(PULSIN_PANIC ? BLINK_FAST : LED_ON);
	
		if(interval_us){
			//adc_debug_dump();

			getEstimatedInclination(interval_us);

			double target =  0.3 * cmd[PTC] / 100.0;
			double error = RwEst[0] - target;
			double delta = 30 * error;
			int maxDuty = MAX(cmd[THR],0);

			int minDuty = maxDuty - (int)(fabs(delta)+0.5);
			minDuty = MAX(0,minDuty);

			if(delta > 0 ){
				motorDuty[0] = maxDuty;
				motorDuty[1] = minDuty;
			}else{
				motorDuty[1] = maxDuty;
				motorDuty[0] = minDuty;
			}

			printf("%lu,%d,%d,%.3f,%.3f,%.3f,%d,%d\n" , interval_us,cmd[THR],cmd[PTC],RwAcc[0],RwEst[0],error,motorDuty[0],motorDuty[1]);
			//for(i=0;i<3;i++) printf("%#8.3f,",RwAcc[i]);
			//for(i=0;i<3;i++) printf("%#8.3f,",RwEst[i]);


			motor_update_duty();


		}	
	};


	return 0;
}

//---------------------------------------------------------------------
// THE END
//---------------------------------------------------------------------
