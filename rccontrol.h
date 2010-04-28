#ifndef RCCONTROL_H
#define RCCONTROL_H

//---------------------------------------------------------------------
// RC CONTROL MAIN LOOP
//---------------------------------------------------------------------
void power_check(){
	//check main power level
	unsigned int adc0 = adc_read(0);
	unsigned int adc1 = adc_read(1);
	unsigned int mainPowerDV =  33UL * 8 * adc0 / 1023;	//get main power voltage in deciVolts, 1:8 resistor devider used, Vref = 3.3V
	if( mainPowerDV < 110){
		printf("adc0:%u mainPowerDV:%u adc1:%u \n",adc0,mainPowerDV,adc1);
		sound_loop(750,500,50,2*2);		//sound alarm if power below 11V
	}
}

void rc_main_loop(){

	while(1){
 		power_check();
		
		int speed = MAP_TO_RANGE(pulsinPulse[0],US_TO_TK2(1100UL),US_TO_TK2(1900UL),-100L,100UL);
		int direction = -MAP_TO_RANGE(pulsinPulse[1],US_TO_TK2(1100UL),US_TO_TK2(1900UL),-100L,100UL);

		//detect RC off
		if(
			pulsinPulse[0]< US_TO_TK2(850UL) || pulsinPulse[0]> US_TO_TK2(2150UL) ||
			pulsinPulse[1]< US_TO_TK2(850UL) || pulsinPulse[1]> US_TO_TK2(2150UL)

		){
			speed = 0;
			direction = 0;
			sound_loop(2500,100,150,2*3);		//sound alarm if receiver out of reach
		}

		
		int factor = speed > 0 ? speed : -speed;
		factor += 100;

		int speed1 = speed - direction * 100 / factor;
			speed1 = PUT_IN_RANGE( speed1  , -100,100);
		int speed2 = speed + direction * 100 / factor;
			speed2 = PUT_IN_RANGE ( speed2 , -100,100);

		printf("pulsin0 = %u pulsin1=%u speed:%d direction:%d speed1:%d speed2:%d factor:%d\n", pulsinPulse[0], pulsinPulse[1], speed, direction,speed1,speed2,factor);
	
		motor_drive(speed1,speed2);

		__delay_ms(10);

	}


}


#endif
