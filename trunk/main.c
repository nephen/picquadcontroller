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

//Status pins
#define pinLed1(f) 		f(B,4)		//Led, OUTPUT
#define pinLed2(f) 		f(A,8)		//Led, OUTPUT
#define pinSound(f)		f(A,10)		//Sound, OUTPUT

//Uart pins
#define pinRX(f)  		f(C,9)		//RP25, UART RX, INPUT , IF CHANGED UPDATE uart_init()
#define pinTX(f)  		f(C,8)		//RP24, UART TX, OUTPUT, IF CHANGED UPDATE uart_init()

//Receiver pins
#define pinPulsin0(f)  	f(B,15)				//CH1
#define pinPulsin0CN	CNEN1bits.CN11IE	
#define pinPulsin1(f)  	f(B,13)				//CH2
#define pinPulsin1CN	CNEN1bits.CN13IE	
#define pinPulsin2(f)  	f(B,11)				//CH3
#define pinPulsin2CN	CNEN1bits.CN15IE
#define pinPulsin3(f)  	f(C,7)				//CH4
#define pinPulsin3CN	CNEN2bits.CN17IE
#define pinPulsin4(f)  	f(C,3)				//CH5
#define pinPulsin4CN	CNEN2bits.CN28IE
#define pinPulsin5(f)  	f(C,4)				//CH6
#define pinPulsin5CN	CNEN2bits.CN25IE

//Motor PWM pins
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
	
	//---------------------------------------------------------------------
	// CONFIGURATION
	//---------------------------------------------------------------------
	config_load();

	//---------------------------------------------------------------------
	// STATUS (LED/SOUND)
	//---------------------------------------------------------------------
	status_init();
	STATUS(_ON,_ON,_ON);	//Beep and turn on all leds on startup
	__delay_ms(100);
	STATUS_INIT;			//Init status shown until calibration is done

	//---------------------------------------------------------------------
	// ADC	
	//---------------------------------------------------------------------
	adc_init();


	//---------------------------------------------------------------------
	// UART
	//---------------------------------------------------------------------
	uart_init();
	printf("\n*** PicQuadController ***\n");

	//radiomodule test 64bytes frame  at ~ 1800bps effective rate
	/*
	while(1){
		unsigned char c, k;
		uart_buffering_start();
		for(c=0;c<62;c++){
			uart_send_char('0'+c);
		}
		uart_send_char('0' + (k++%10));
		uart_send_char(10);
		uart_buffering_end();
		
		__delay_ms(350);
	}
	*/

	//---------------------------------------------------------------------
	// CALIBRATION
	//---------------------------------------------------------------------
	//gyro calibration is fast and should be run on every startup, just hold device still
	calibrate_gyro();	
	//acc calibration requires user interraction and writes to Flash run only periodically
	if(/*ACC_CALIB_CONDITION*/ 0) calibrate_acc();
	config_print();
	STATUS_NORMAL;			//Calibration is done, normal operation status


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


	//pulsin debug loop
	/*
	while(1){
		int i;
		pulsin_process();
		for(i=0;i<6;i++) printf("%d,%d, ",pulsinUs[i],cmd[i]);
		printf("0\n");
		__delay_ms(500);
	}
	*/

	//---------------------------------------------------------------------
	// MOTOR
	//---------------------------------------------------------------------
	motor_init();

	/*
	//motor debug loop	
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

	//---------------------------------------------------------------------
	// MAIN LOOP
	//---------------------------------------------------------------------
	#define MAX_CONTROL	10.0					//maximum control in duty cycle units 
	#define MAX_THR  (100.0-MAX_CONTROL*2)		//maximum allowed throtle 
	#define MAX_TILT (20.0*PI/180.0)			//deg to rad
	yaw.Kd = 15000.0;
	//Flight Note: 
	//6/21/11 : good results with yaw.kd=15k, pitch/roll kd=15-20k  , pitch/roll kp=500, but frame flimsy and broke outside
	

	unsigned char output_sequence = 0;				//output sequence to track skipped packets
	unsigned int output_skipped = 0;				//number of loops skipped (no output)
	unsigned int output_chars = 0;					//number of chars sent in each output		 
	while (1){
		unsigned char panic = 0;
		unsigned long interval_us = adc_new_data();

		pulsin_process();

		//any "panic" situation will result in gradual throtle decrease
		if(PULSIN_PANIC){ 	STATUS_PANIC_SIGNAL; panic = 1;	}
		if(BATTERY_VOLTS < 9.0){  STATUS_PANIC_BATTERY; panic = 1;}

		if(interval_us > 0 ){		//loop running at 7.056ms see adcutil.h
			//we have fresh adc samples

			imu_update(interval_us);
			
			//Roll and Pitch PID control



			unsigned char w;
			float* K = dcmEst[2];						//K(body) vector from DCM matrix	
			float pitch_roll = acos(K[2]);				//total pitch and roll, angle necessary to bring K to [0,0,1]
														//cos(K,K0) = [Kx,Ky,Kz].[0,0,1] = Kz
			//now allocate this angle proportionally between pitch and roll based on Kx, Ky
			float Kxy = sqrt(K[0]*K[0] + K[1]*K[1]);
			if(fabs(Kxy) < 0.01){						//avoid division by 0, stabilize values when Kxy is close to 0
				pitch.value = 0;						//one of the two case when Kxy is close to 0 is when device is upside down
				roll.value = pitch_roll;				//default corrective action is roll 
			}else{
				pitch.value = - pitch_roll * asin(K[0]/Kxy) / (PI/2); 
				roll.value = pitch_roll * asin(K[1]/Kxy) / (PI/2); 		
			}

			for(w=0;w<2;w++){	//ROL = 0 , PTC = 1
				pid[w].Kp = map_to_range(cmd[VRA],0.0,100.0,	0.0,2000.0);		//Kp set by VRA channel on transmitter
				pid[w].Kd = map_to_range(cmd[VRB],0.0,100.0, 	0.0,20000.0);		//Kd set by VRB channel on transmitter

				pid[w].target =  map_to_range(cmd[w],-100.0,100.0, -0.03, 0.03);		//target inclination in rad
				pid[w].error = low_pass_filter(pid[w].target  - pid[w].value,pid[w].error, 25.0 ); 	//filter
				pid[w].dError = - getGyroOutput(1-w);								//get delta error dirrectly from gyros

				pid[w].control = control_pid(pid[w].error,0,pid[w].dError ,	pid[w].Kp,0,pid[w].Kd);

			}

			//development safety feature, panic if inclination is too big (imminent crash)
			
			if(fabs(roll.value) + fabs(pitch.value) > MAX_TILT){
				STATUS_PANIC_TILT;
				panic = 1;
			}

			//Throtle control 
			if(panic){	//slowly decrease throtle on panic	
				throt.control = MAX(0,throt.control - 0.1);	//this will cut throtle from 100 to 0 in 100/0.1 * 7.056ms = 7.056s
			}else{		//normal throtle control
				//TODO: implement PID based on vertical acceleration
				throt.control = MIN(MAX_THR,cmd[THR]);
				STATUS_NORMAL;
			}
			
			//Adjust pitch/roll Ki
			/*
			if(0 == cmd[THR] && cmd[YAW] > 70){
				//To set pitch/roll.Ki move left stick to bottom-left corner , 
				//while holding YAW&THR stick in this position adjust VRA that will set pitch/roll.Ki.
				//After you release YAW&THR stick from this position 
				//remember to revert VRA to previous level wich normaly sets Roll&Pitch Kp in real time
				roll.Ki = map_to_range(cmd[VRA],0.0,100.0,	0.0,100.0);
				pitch.Ki = roll.Ki;
				STATUS_ADJUST;
			}*/

			
			//Yaw PID control
			if(0 == cmd[THR] && cmd[YAW] < -70){
				//To set yaw.Kd move left stick to bottom-right corner , 
				//while holding YAW&THR stick in this position adjust VRB that will set yaw.Kd.
				//After you release YAW&THR stick from this position 
				//remember to revert VRA/VRB to previous levels wich normaly sets Roll&Pitch Kp/Kd in real time
				yaw.Kd = map_to_range(cmd[VRB],0.0,100.0,	0.0,20000.0);
				STATUS_ADJUST;
			}
			yaw.target = map_to_range(cmd[YAW],-100.0,100.0, -0.002, 0.002);				//target Yaw rotation rate in  rad / ms
			yaw.dError = low_pass_filter(yaw.target - getGyroOutput(2),yaw.dError, 5.0 ); 	//filter			
			yaw.control = 	control_pid(0,0,yaw.dError,				0,0,yaw.Kd);

			//Quad motor control in [+] configuration
			if(0 ==	throt.control){
				for(w=0;w<4;w++) motor_set_duty(w,0);
			}else{
				motor_set_duty(0,throt.control - yaw.control - pitch.control);		//FRONT
				motor_set_duty(1,throt.control - yaw.control + pitch.control);		//BACK
				motor_set_duty(2,throt.control + yaw.control + roll.control);		//LEFT
				motor_set_duty(3,throt.control + yaw.control - roll.control);		//RIGHT
			}




			//radio serial link needs 350ms ~ 7ms *50 loops to send a 64 byte frame 
			if(!uart_buffering_busy && output_skipped>=20 ){
				uart_buffering_start();												//output data using UART with DMA
				//---------------------------------------
				
				

				//printf("%.2f,%.2f ,%.2f,%.2f,%.2f\n",pitch.value,roll.value,dcmEst[2][0],dcmEst[2][1],dcmEst[2][2]);			

		
				//Output for: FlightTrack.pde
				hdlc_send_byte(output_sequence);										//sequence 
				for(w=0;w<4;w++) hdlc_send_byte((unsigned char)motorDuty[w]);			//motor
				for(w=0;w<6;w++) hdlc_send_byte((char)cmd[w]);							//command

				hdlc_send_byte((char)throtle);											//throtle
				hdlc_send_byte((char)map_to_range(roll.value,-PI,PI,-100.0,100.0));		//roll
				hdlc_send_byte((char)map_to_range(pitch.value,-PI,PI,-100.0,100.0));	//pitch
				
				#define DERR_MAX (PI/1000.0)
				hdlc_send_byte((char)map_to_range(roll.dError,-DERR_MAX,DERR_MAX,-100.0,100.0));		//roll delta				
				hdlc_send_byte((char)map_to_range(pitch.dError,-DERR_MAX,DERR_MAX,-100.0,100.0));		//pitch delta
				hdlc_send_byte((char)map_to_range(yaw.dError,-DERR_MAX,DERR_MAX,-100.0,100.0));			//yaw delta

				hdlc_send_sep();														//separator



				
				//Output for: PICQUADCONTROLLER_LIVE1.scc
				/*
				hdlc_send_byte((char)map_to_range(pitch.value,-PI,PI,-100.0,100.0));	//pitch
				hdlc_send_byte((char)map_to_range(roll.value,-PI,PI,-100.0,100.0));		//roll

				#define DERR_MAX (PI/1000.0)
				hdlc_send_byte((char)map_to_range(pitch.dError,-DERR_MAX,DERR_MAX,-100.0,100.0));		//pitch delta
				hdlc_send_byte((char)map_to_range(roll.dError,-DERR_MAX,DERR_MAX,-100.0,100.0));		//roll delta
				hdlc_send_byte((char)map_to_range(yaw.dError,-DERR_MAX,DERR_MAX,-100.0,100.0));			//yaw delta


				for(w=0;w<4;w++) hdlc_send_byte((unsigned char)motorDuty[w]);			//motor
				for(w=0;w<6;w++) hdlc_send_byte((char)cmd[w]);							//command

				hdlc_send_byte(output_sequence);										//sequence 
				hdlc_send_sep();														//separator
				*/

				/*
				//Output for: PICQUADCONTROLLER_DEBUG1.pde
				printf("%.2f, ",(double)imu_interval_ms);
				print_float_list(3,(float*)Kacc);
				printf(", ");
				print_float_list(9,(float*)dcmEst);
				printf(" ,%.2f,%.2f\n",(double)pitch.value,(double)roll.value);
				*/
				
				/*
				//Output for PicQuadController_GYRO_DEBUG1.scc
				printf("%.5f,",(double)interval_ms);
				print_float_list(3,(float*)w);
				printf(",%.2f,%.2f,%.2f",adcAvg[3+1],adcAvg[3+0],adcAvg[3+2]);
				printf("\n ");
				*/

				/*
				//SerialChart file: PicQuadController_DEBUG_CONTROL.scc
				hdlc_send_byte(float_to_int(roll.control));
				hdlc_send_byte(float_to_int(roll.control));
				hdlc_send_byte(float_to_int(yaw.control));
				hdlc_send_sep();
				*/		


				/*
				//SerialChart file: PicQuadController_DEBUG1.scc
				//hdlc_send_word(interval_us);
				hdlc_send_byte(float_to_int(errorPitch * 100));
				hdlc_send_byte(motorDuty[0]);
				hdlc_send_byte(motorDuty[1]);
				hdlc_send_byte(motorDuty[2]);
				hdlc_send_byte(motorDuty[3]);
				hdlc_send_sep();
				*/

				/*
				//SerialChart file: PicQuadController_TX.scc
				hdlc_send_byte(cmd[THR]);
				hdlc_send_byte(cmd[YAW]);
				hdlc_send_byte(cmd[ROL]);
				hdlc_send_byte(cmd[PTC]);
				hdlc_send_byte(cmd[VRA]);
				hdlc_send_byte(cmd[VRB]);
				hdlc_send_sep();
				*/

				//---------------------------------------
				output_chars = uart_buffering_end();
				output_skipped=0;
				output_sequence++;
			}else{
				output_skipped++;
			}


		

		}
		

		
		motor_apply_duty();
		
	};



	return 0;
}

//---------------------------------------------------------------------
// THE END
//---------------------------------------------------------------------
