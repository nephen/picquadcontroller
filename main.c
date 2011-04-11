#include "p33FJ128MC804.h"

#include <libpic30.h>



#include "oscilator.h"

#include <pwm.h>
#include <timer.h>
#include <pwm.h>
#include <uart.h>
#include <stdio.h>
#include <adc.h>
#include <math.h>

//---------------------------------------------------------------------
// I/O PINS
//---------------------------------------------------------------------


#define pinLed1(f) 		f(B,4)		//Led, OUTPUT
#define pinLed2(f) 		f(A,8)		//Led, OUTPUT

#define pinSound(f)		f(A,10)		//Sound, OUTPUT


#define pinRX(f)  		f(C,9)		//RP25, UART RX, INPUT , IF CHANGED UPDATE uart_init()
#define pinTX(f)  		f(C,8)		//RP24, UART TX, OUTPUT, IF CHANGED UPDATE uart_init()

#define pinPulsin0(f)  	f(B,15)	
#define pinPulsin0CN	CNEN1bits.CN11IE	

#define pinPulsin1(f)  	f(B,13)		
#define pinPulsin1CN	CNEN1bits.CN13IE

#define pinPulsin2(f)  	f(B,11)		
#define pinPulsin2CN	CNEN1bits.CN15IE

#define pinPulsin3(f)  	f(C,7)
#define pinPulsin3CN	CNEN2bits.CN17IE

#define pinPulsin4(f)  	f(B,7)
#define pinPulsin4CN	CNEN2bits.CN23IE

#define pinPulsin5(f)  	f(A,4)
#define pinPulsin5CN	CNEN1bits.CN0IE


#define pinMotor0(f)  	f(B,14)	//Front Motor, PWM1H1
#define pinMotor1(f)  	f(B,12)	//Back Motor, PWM1H2
#define pinMotor2(f)  	f(B,10)	//Left Motor, PWM1H3
#define pinMotor3(f)  	f(C,6)	//Right Motor, PWM2H1

//---------------------------------------------------------------------
// LIBS
//---------------------------------------------------------------------


#include "macroutil.h"
#include "config.h"
#include "adcutil.h"
#include "pulsin.h"
#include "motor.h"
#include "uartutil.h"
#include "led.h"
#include "matrix.h"
#include "vector3d.h"
#include "calibrate.h"
#include "imu.h"
#include "control.h"



//---------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------


int main (void)
{
	//Configure all ports as inputs
	TRISA = 0xFFFF; TRISB = 0xFFFF; TRISC = 0xFFFF;

	//---------------------------------------------------------------------
	// OSCILATOR
	//---------------------------------------------------------------------
	oscilator_init();
	
	config_load();

	//---------------------------------------------------------------------
	// STATUS (LED/SOUND)
	//---------------------------------------------------------------------
	status_init();
	STATUS(_ON,_ON,_ON);
	__delay_ms(100);
	STATUS_INIT;

	//---------------------------------------------------------------------
	// ADC	
	//---------------------------------------------------------------------
	adc_init();


	//---------------------------------------------------------------------
	// UART
	//---------------------------------------------------------------------
	uart_init();
	printf("\n*** PicQuadController ***\n");

	//gyro calibration is fast and should be run on every startup, just hold device still
	calibrate_gyro();	

	//acc calibration requires user interraction and writes to Flash run only periodically
	if(/*ACC_CALIB_CONDITION*/ 0) calibrate_acc();

	config_print();

	
	STATUS_NORMAL;


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

	/*
	int i=100;
	while(1){
		_LAT(pinLed1) =! _LAT(pinLed1);
		motor_set_duty(0,i);
		motor_set_duty(1,i);
		motor_set_duty(2,i);
		motor_set_duty(3,i);
		__delay_ms(2000);
		i = i - 10;
		if(i<0) i = 100;
		motor_apply_duty();
	};
	*/


	//---------------------------------------------------------------------
	// IMU
	//---------------------------------------------------------------------
	imu_init();

	//IMU debug comment out for production
	while (1){
		unsigned long interval_us = adc_new_data();
		//adc_debug_dump1();
		if(interval_us > 0 ){
			//we have fresh adc samples
			imu_update(interval_us);

			//PicQuadController_ADC_DEBUG1.scc
			//printf("%lu,%.2f,%.2f,%.3f,%.3f\n",interval_us,RwAcc[0],RwEst[0],accErr,accWeight);	
	
			
		}
	}


	//---------------------------------------------------------------------
	// MAIN LOOP
	//---------------------------------------------------------------------

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
		unsigned char panic = 0;
		unsigned long interval_us = adc_new_data();

		pulsin_process();

		//any "panic" situation will result in gradual throtle decrease
		if(PULSIN_PANIC){ 
			STATUS_PANIC_SIGNAL;
			panic = 1;
		}

		if(BATTERY_VOLTS < 9.0){ 
			STATUS_PANIC_BATTERY;
			panic = 1;
		}



		if(interval_us > 0 ){		//loop running at 7.056ms see adcutil.h
			//we have fresh adc samples

			imu_update(interval_us);

			//development safety feature, decrease throtle if inclination is too big (imminent crash)
			if(fabs(dcmEst[0][0]) < 0.4 && fabs(dcmEst[0][1]) < 0.4){
				STATUS_PANIC_TILT;
				panic = 1;
			}
			
			if(panic){
				throtle = MAX(0,throtle - 0.1);	//this will cut throtle from 100 to 0 in 100/0.1 * 7.056ms = 7.056s
			}else{
				throtle = MIN(MAX_THR,cmd[THR]);
				STATUS_NORMAL;
			}

			
			//Max(K) ~= MAX_CONTROL / Max(measured) = 10 / 0.3 deg/ms = 33
			//To set Kd_Yaw move left stick to bottom-left , then adjust VRA, release stick, then revert VRA to previous level
			//Led will turn off while in Kd_Yaw adjustment mode
			if(0 == cmd[THR] && cmd[YAW] < -70){
				Kd_Yaw = map_to_range(cmd[VRA],0.0,100.0,	0.0,100.0);
				STATUS_ADJUST;
			}
			//Kp for Roll and Pitch
			Kp_RollAndPitch = map_to_range(cmd[VRA],0.0,100.0,	0.0,100.0);

			//Kd for Roll and Pitch
			Kd_RollAndPitch = map_to_range(cmd[VRB],0.0,100.0, 	0.0,200.0);
			
			
			yawTrg = map_to_range(cmd[YAW],-100.0,100.0, -0.1, 0.1);
			float dErrorYaw =  (getGyroOutput(2)- yawTrg);  		//  deg / ms	//CCW rotation <=> rateYaw >0

			RwTrg[0] = map_to_range(cmd[PTC],-100.0,100.0, -0.3, 0.3);
			float errorPitch =  dcmEst[0][0] - RwTrg[0];
			float dErrorPitch = -getGyroOutput(0); 

			RwTrg[1] = map_to_range(cmd[ROL],-100.0,100.0, -0.3, 0.3);
			float errorRoll = dcmEst[0][1] - RwTrg[1];
			float dErrorRoll = -getGyroOutput(1); 
			

			controlYaw = 	control_pid(0,0,dErrorYaw,				0,0,Kd_Yaw);
			controlPitch =	control_pid(errorPitch,0,dErrorPitch,	Kp_RollAndPitch,0,Kd_RollAndPitch);
			controlRoll =	control_pid(errorRoll,0,dErrorRoll,		Kp_RollAndPitch,0,Kd_RollAndPitch);
	
			motor_set_duty(0,throtle + controlYaw + controlPitch);		//FRONT
			motor_set_duty(1,throtle + controlYaw - controlPitch);		//BACK
			motor_set_duty(2,throtle - controlYaw + controlRoll);		//LEFT
			motor_set_duty(3,throtle - controlYaw - controlRoll);		//RIGHT


			if(0 == imu_sequence % 4){
				//send debug data every few cycles
				//SerialChart file: PicQuadController_DEBUG_CONTROL.scc
				hdlc_send_byte(float_to_int(controlPitch));
				hdlc_send_byte(float_to_int(controlRoll));
				hdlc_send_byte(float_to_int(controlYaw));
				hdlc_send_sep();
			}


			/*
			if(0 == imu_sequence % 4){
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
			*/

			/*
			if(0 == imu_sequence % 4){
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
