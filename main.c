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


//#define pinRX(f)  		f(B,7)		//RP7, UART RX, INPUT , IF CHANGED UPDATE uart_init()

#define pinTX(f)  		f(B,4)		//RP4, UART TX, OUTPUT, IF CHANGED UPDATE uart_init()

#define pinPulsin0(f)  	f(B,15)	
#define pinPulsin0CN	CNEN1bits.CN11IE	
#define pinPulsin1(f)  	f(B,13)		
#define pinPulsin1CN	CNEN1bits.CN13IE
#define pinPulsin2(f)  	f(B,11)		
#define pinPulsin2CN	CNEN1bits.CN15IE
#define pinPulsin3(f)  	f(B,9)
#define pinPulsin3CN	CNEN2bits.CN21IE
#define pinPulsin4(f)  	f(B,7)
#define pinPulsin4CN	CNEN2bits.CN23IE
#define pinPulsin5(f)  	f(A,4)
#define pinPulsin5CN	CNEN1bits.CN0IE



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
	// PULSIN
	//---------------------------------------------------------------------
	pulsin_init();
	PULSIN_ASSIGN_PIN(0,pinPulsin0,pinPulsin0CN);
	PULSIN_ASSIGN_PIN(1,pinPulsin1,pinPulsin1CN);
	PULSIN_ASSIGN_PIN(2,pinPulsin2,pinPulsin2CN);
	PULSIN_ASSIGN_PIN(3,pinPulsin3,pinPulsin3CN);
	PULSIN_ASSIGN_PIN(4,pinPulsin4,pinPulsin4CN);
	PULSIN_ASSIGN_PIN(5,pinPulsin5,pinPulsin5CN);

	//---------------------------------------------------------------------
	// MOTOR
	//---------------------------------------------------------------------
	motor_init();

	//---------------------------------------------------------------------
	// IMU
	//---------------------------------------------------------------------
	imu_init();

	//---------------------------------------------------------------------
	// MAIN LOOP
	//---------------------------------------------------------------------

	led(LED_ON);

	#define MAX_DELTA	10.0				//maximum delta in duty of any motor and throtle
	#define MAX_THR  (100.0-MAX_DELTA*2)	//maximum allowed throtle 
	float motorDutyNew[4] = {0,0,0,0};		//new duty cycles for motors
	float deltaRoll,deltaPitch, deltaYaw;
	float Kp_RollPitch,Kp_Yaw;				//proportional feedback coeficients (adjustable via ch5/6 on transmitter)
	

	while (1){
		unsigned long interval_us = adc_new_data();

		pulsin_process();
		led(PULSIN_PANIC ? BLINK_FAST : LED_ON);


		float throtle = MIN(MAX_THR,cmd[THR]);
		//float deltaMax = MIN(MAX_DELTA,throtle/2);


		if(interval_us > 0 ){
			//we have fresh adc samples
			getEstimatedInclination(interval_us);

			//MAX_PROP ~=MAX_DELTA / MAX_MEASURED = 10 / 0.3 deg/ms = 33
			Kp_Yaw = map_to_range(cmd[VRA],0.0,100.0, 0.0, 33.0);
			//MAX_PROP ~=MAX_DELTA / MAX_MEASURED = 10 / 0.3 = 33;
			Kp_RollPitch = map_to_range(cmd[VRA],0.0,100.0, 0.0, 33.0);

			float measuredYaw = put_in_range(getGyroOutput(2),-0.3,0.3);  // -0.3 ... 0.3 deg / ms	//CCW rotation <=> rateYaw >0
			deltaYaw = measuredYaw * Kp_Yaw; 

			float measuredPitch = put_in_range(RwEst[0],-0.3,0.3);
			deltaPitch = measuredPitch * Kp_RollPitch; 

			float measuredRoll = put_in_range(RwEst[1],-0.3,0.3);
			deltaRoll = measuredRoll * Kp_RollPitch; 
			
			motorDutyNew[0]	= throtle + deltaYaw + deltaPitch;		//FRONT
			motorDutyNew[1]	= throtle + deltaYaw - deltaPitch;		//BACK
			motorDutyNew[2]	= throtle - deltaYaw + deltaRoll;		//LEFT
			motorDutyNew[3]	= throtle - deltaYaw - deltaRoll;		//RIGHT

			if(0 == sequence % 8){
				//send debug data every few cycles
				//SerialChart file: PicQuadController_DEBUG1.scc
				hdlc_send_word(interval_us);
				hdlc_send_float(motorDutyNew[0]);
				hdlc_send_float(motorDutyNew[1]);
				hdlc_send_float(motorDutyNew[2]);
				hdlc_send_float(motorDutyNew[3]);
				hdlc_send_sep();
			}

			motorDuty[0] = float_to_int(put_in_range(motorDutyNew[0],0,100));
			motorDuty[1] = float_to_int(put_in_range(motorDutyNew[1],0,100));
			motorDuty[2] = float_to_int(put_in_range(motorDutyNew[2],0,100));
			motorDuty[3] = float_to_int(put_in_range(motorDutyNew[3],0,100));
			

		}
		

		
		motor_update_duty();
		
	};



	return 0;
}

//---------------------------------------------------------------------
// THE END
//---------------------------------------------------------------------
