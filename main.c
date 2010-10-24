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


//#define pinRX(f)  	f(B,7)		//RP7, UART RX, INPUT , IF CHANGED UPDATE uart_init()

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
#include "control.h"
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

	//IMU debug comment out for production
	/*
	while (1){
		unsigned long interval_us = adc_new_data();	
		if(interval_us > 0 ){
			//we have fresh adc samples
			getEstimatedInclination(interval_us);
			//PicQuadController_ADC_DEBUG1.scc
			printf("%lu,%.2f,%.2f,%.3f,%.3f\n",interval_us,RwAcc[0],RwEst[0],accErr,accWeight);	
			__delay_ms(10); //give computer some time to breathe
		}
	}
	*/

	//---------------------------------------------------------------------
	// MAIN LOOP
	//---------------------------------------------------------------------

	led(LED_ON);

	#define MAX_CONTROL	10.0					//maximum control in duty cycle units 
	#define MAX_THR  (100.0-MAX_CONTROL*2)		//maximum allowed throtle 
	float controlRoll,controlPitch, controlYaw;	//control for each axis
	float Kp_RollAndPitch;						//feedback coeficients (adjustable via ch5/6 on transmitter)
	float Kd_RollAndPitch,Kd_Yaw;						
	float throtle = 0;

	float RwTrg[2];								//target inclination
	float yawTrg;								//target Yaw rotation rate in  deg / ms

	Kd_Yaw = 50;

	while (1){
		unsigned long interval_us = adc_new_data();

		pulsin_process();



		if(interval_us > 0 ){
			//we have fresh adc samples
			getEstimatedInclination(interval_us);

			//development safety feature, decrease throtle if inclination is too big (imminent crash)
			if(fabs(RwEst[0]) < 0.4 && fabs(RwEst[1]) < 0.4){
				throtle = MIN(MAX_THR,cmd[THR]);
				led(PULSIN_PANIC ? BLINK_FAST : LED_ON);
			}else{
				throtle = MAX(0,throtle - 0.1);	
				led(LED_OFF);
			}


			//Max(K) ~= MAX_CONTROL / Max(measured) = 10 / 0.3 deg/ms = 33
			//To set Kd_Yaw move left stick to bottom-left , then adjust VRA, release stick, then revert VRA to previous level
			//Led will turn off while in Kd_Yaw adjustment mode
			if(0 == cmd[THR] && cmd[YAW] < -70){
				Kd_Yaw = map_to_range(cmd[VRA],0.0,100.0,	0.0,100.0);
				led(LED_OFF);
			}
			//Kp for Roll and Pitch
			Kp_RollAndPitch = map_to_range(cmd[VRA],0.0,100.0,	0.0,100.0);

			//Kd for Roll and Pitch
			Kd_RollAndPitch = map_to_range(cmd[VRB],0.0,100.0, 	0.0,200.0);
			
			
			yawTrg = map_to_range(cmd[YAW],-100.0,100.0, -0.1, 0.1);
			float dErrorYaw =  (getGyroOutput(2)- yawTrg);  		//  deg / ms	//CCW rotation <=> rateYaw >0

			RwTrg[0] = map_to_range(cmd[PTC],-100.0,100.0, -0.3, 0.3);
			float errorPitch =  RwEst[0] - RwTrg[0];
			float dErrorPitch = -getGyroOutput(0); 

			RwTrg[1] = map_to_range(cmd[ROL],-100.0,100.0, -0.3, 0.3);
			float errorRoll = RwEst[1] - RwTrg[1];
			float dErrorRoll = -getGyroOutput(1); 
			

			controlYaw = 	control_pid(0,0,dErrorYaw,				0,0,Kd_Yaw);
			controlPitch =	control_pid(errorPitch,0,dErrorPitch,	Kp_RollAndPitch,0,Kd_RollAndPitch);
			controlRoll =	control_pid(errorRoll,0,dErrorRoll,		Kp_RollAndPitch,0,Kd_RollAndPitch);
	
			motor_set_duty(0,throtle + controlYaw + controlPitch);		//FRONT
			motor_set_duty(1,throtle + controlYaw - controlPitch);		//BACK
			motor_set_duty(2,throtle - controlYaw + controlRoll);		//LEFT
			motor_set_duty(3,throtle - controlYaw - controlRoll);		//RIGHT


			/*
			if(0 == sequence % 4){
				//send debug data every few cycles
				//SerialChart file: PicQuadController_DEBUG_CONTROL.scc
				hdlc_send_byte(float_to_int(controlPitch));
				hdlc_send_byte(float_to_int(controlRoll));
				hdlc_send_byte(float_to_int(controlYaw));
				hdlc_send_sep();
			}
			*/


			if(0 == sequence % 4){
				//send debug data every few cycles
				//SerialChart file: PicQuadController_DEBUG1.scc
				//hdlc_send_word(interval_us);
				hdlc_send_byte(float_to_int(errorPitch * 100));
				hdlc_send_byte(motorDuty[0]);
				hdlc_send_byte(motorDuty[1]);
				hdlc_send_byte(motorDuty[2]);
				hdlc_send_byte(motorDuty[3]);
				hdlc_send_sep();
			}

			/*
			if(0 == sequence % 4){
				//send debug data every few cycles
				//SerialChart file: PicQuadController_TX.scc
				hdlc_send_byte(cmd[THR]);
				hdlc_send_byte(cmd[YAW]);
				hdlc_send_byte(cmd[ROL]);
				hdlc_send_byte(cmd[PTC]);
				hdlc_send_byte(cmd[VRA]);
				hdlc_send_byte(cmd[VRB]);
				hdlc_send_sep();
			}
			*/
		

		}
		

		
		motor_apply_duty();
		
	};



	return 0;
}

//---------------------------------------------------------------------
// THE END
//---------------------------------------------------------------------
