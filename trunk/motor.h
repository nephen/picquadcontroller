#ifndef MOTOR_H
#define MOTOR_H


//---------------------------------------------------------------------
// MOTOR CONTROL (PWM MODULE 1  & PWM MODULE 3)
//---------------------------------------------------------------------
#define MOTOR_PWM_FREQ 1000		//motor PWM frequency in Hz
			

#define MOTOR_DUTY(percent) 	(P1DC1 = (2UL*P1TPER+2)*percent/100)	//(100% <=> P1TPER*2 since duty cycle resolution is Tcy/2)
#define MOTOR2_DUTY(percent) 	(P1DC2 = (2UL*P1TPER+2)*percent/100)	//see http://ww1.microchip.com/downloads/en/DeviceDoc/70187C.pdf  pg.33

int motorDuty[4] = {0,0,0,0};	//FRONT,BACK,LEFT,RIGHT
void motor_update_duty(){
	//(100% <=> P1TPER*2 since duty cycle resolution is Tcy/2)	
	//see http://ww1.microchip.com/downloads/en/DeviceDoc/70187C.pdf  pg.33
	P1DC1 = (2UL*P1TPER+2)*motorDuty[0]/100;
	P1DC2 = (2UL*P1TPER+2)*motorDuty[1]/100;
	P1DC3 = (2UL*P1TPER+2)*motorDuty[2]/100;
	P2DC1 = (2UL*P2TPER+2)*motorDuty[3]/100;
}

void motor_init(){
	//setup output pins
	_LAT(pinMotor0) = 0;
	_LAT(pinMotor1) = 0;
	_LAT(pinMotor2) = 0;
	_LAT(pinMotor3) = 0;

	_TRIS(pinMotor0) = 0;
	_TRIS(pinMotor1) = 0;
	_TRIS(pinMotor2) = 0;
	_TRIS(pinMotor3)= 0;
	
	//setup PWM ports
	//PWM1, MOTORS 0,1,2	
	PWM1CON1 = 0;				//clear all bits (use defaults)
	PWM1CON1bits.PMOD1 = 0; 	//PWM1Ly,PWM1Hy are in independent running mode
	PWM1CON1bits.PEN1L = 0; 	//PWM1L1 NORMAL I/O
	PWM1CON1bits.PEN1H = 1; 	//PWM1H1 PWM OUTPUT
	PWM1CON1bits.PMOD2 = 0; 	//PWM2Ly,PWM2Hy are in independent running mode
	PWM1CON1bits.PEN2L = 0; 	//PWM1L2 NORMAL I/O
	PWM1CON1bits.PEN2H = 1; 	//PWM1H2 PWM OUTPUT
	PWM1CON1bits.PMOD3 = 0; 	//PWM3Ly,PWM2Hy are in independent running mode
	PWM1CON1bits.PEN3L = 0; 	//PWM1L3 NORMAL I/O
	PWM1CON1bits.PEN3H = 1; 	//PWM1H3 PWM OUTPUT
	

	//PWM2, MOTOR 3
	PWM2CON1 = 0;				//clear all bits (use defaults)
	PWM2CON1bits.PMOD1 = 0; 	//PWM2Ly,PWM2Hy are in independent running mode
	PWM2CON1bits.PEN1L = 0; 	//PWM2L1 NORMAL I/O
	PWM2CON1bits.PEN1H = 1; 	//PWM2H1 PWM OUTPUT

	//PWM mode and prescaler
	//PWM1, MOTORS 0,1,2	
	P1TCON = 0;					//clear all bits (use defaults)
	P1TCONbits.PTMOD = 0b00;	//Free-runing mode 
	P1TCONbits.PTCKPS = 0b11;	// 1:64 prescaler
	//PWM2, MOTOR 3
	P2TCON = 0;					//clear all bits (use defaults)
	P2TCONbits.PTMOD = 0b00;	//Free-runing mode 
	P2TCONbits.PTCKPS = 0b11;	// 1:64 prescaler


	//setup desired frequency by setting period for 1:64 prescaler
	//PWM1, MOTORS 0,1,2	
	P1TPER = (FCY  / 64 / MOTOR_PWM_FREQ) - 1;	
	//PWM2, MOTOR 3
	P2TPER = (FCY  / 64 / MOTOR_PWM_FREQ) - 1;	

	//update duty cycle 
	motor_update_duty();	
	
	//ENABLE PWM
	//PWM1, MOTORS 0,1,2
	P1TMR = 0;
	P1TCONbits.PTEN = 1;	
	//PWM2, MOTOR 3
	P2TMR = 0;
	P2TCONbits.PTEN = 1;	

}



#endif
