#ifndef MOTOR_H
#define MOTOR_H


//---------------------------------------------------------------------
// MOTOR CONTROL (USING 6 CHANNEL PWM MODULE 1)
//---------------------------------------------------------------------
#define MOTOR_PWM_FREQ_HIGH 100		//motor PWM frequency in Hz
									//determine experimentally so that motor rotates at low duty cycles

#define MOTOR_FREQ(freqHz)			(P1TPER = FCY  / 64 / freqHz -1)
#define MOTOR1_DUTY(percent) 	(P1DC1 = (2UL*P1TPER+2)*percent/100)	//(100% <=> P1TPER*2 since duty cycle resolution is Tcy/2)
#define MOTOR2_DUTY(percent) 	(P1DC2 = (2UL*P1TPER+2)*percent/100)	//see http://ww1.microchip.com/downloads/en/DeviceDoc/70187C.pdf  pg.33

void motor_init(){

	
	//setup PWM ports
	PWM1CON1 = 0;				//clear all bits (use defaults)
	PWM1CON1bits.PMOD1 = 0; 	//PWM1Ly,PWM1Hy are in independent running mode
	PWM1CON1bits.PEN1L = 0; 	//PWM1L1 NORMAL I/O
	PWM1CON1bits.PEN1H = 1; 	//PWM1H1 PWM OUTPUT
	PWM1CON1bits.PMOD1 = 0; 	//PWM2Ly,PWM2Hy are in independent running mode
	PWM1CON1bits.PEN2L = 0; 	//PWM1L2 NORMAL I/O
	PWM1CON1bits.PEN2H = 1; 	//PWM1H2 PWM OUTPUT

	//PWM mode and prescaler
	P1TCON = 0;					//clear all bits (use defaults)
	P1TCONbits.PTMOD = 0b00;	//Free-runing mode 
	P1TCONbits.PTCKPS = 0b11;	// 1:64 prescaler

	MOTOR1_DUTY(0);
	MOTOR2_DUTY(0);

	//period for 1:64 prescaler
	MOTOR_FREQ(MOTOR_PWM_FREQ_HIGH);

	//setup output pins
	_TRIS(pinEN1) = 0;
	_TRIS(pinMA1) = 0;
	_TRIS(pinMB1) = 0;
	_TRIS(pinEN2)= 0;
	_TRIS(pinMA2) = 0;
	_TRIS(pinMB2) = 0;

	_LAT(pinEN1) = 0;
	_LAT(pinEN2) = 0;

	P1TMR = 0;
	P1TCONbits.PTEN = 1;	//enable pwm 
}

void motor_drive(int speed1,int speed2){
	if(-10 < speed1 && speed1 < 10 && -10 < speed2 && speed2 < 10){
		P1TCONbits.PTEN = 0; //disable pwm	

	}else{
		P1TCONbits.PTEN = 1;
	} 

	if(speed1>=0){
		_LAT(pinMA1) = 1;
		_LAT(pinMB1) = 0;
	}else{
		_LAT(pinMA1) = 0;
		_LAT(pinMB1) = 1;
		speed1 = - speed1;
	}
	if(speed2>=0){
		_LAT(pinMA2) = 1;
		_LAT(pinMB2) = 0;
	}else{
		_LAT(pinMA2) = 0;
		_LAT(pinMB2) = 1;
		speed2 = - speed2;
	}
	MOTOR1_DUTY(speed1);
	MOTOR2_DUTY(speed2);
}
#endif
