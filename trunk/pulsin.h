#ifndef PULSIN_H
#define PULSIN_H

//---------------------------------------------------------------------
// PULSIN (USING CN interrupts, Timer2)
//---------------------------------------------------------------------

#define PULSIN_LEN	6						// number of pulsin pins

#define US_TO_TK2(tk) ((tk)*5)				// us to Timer2 ticks /  Timer2 5 ticks = 1us (40Mhz FCY, 1:8 prescaler)
#define TK2_TO_US(tk) (((tk)+2)/5)			// Timer2 ticks to us
#define TK2_OVERFLOW_US	(65536UL * 8 / (FCY/1000000UL))	// time between Timer2 overflows in us

#define PULSIN_MAX_NOCHANGE_US	40000UL		// Maximum time of no changed allowed (should be at least 20ms for most transmiters)


#define PULSIN_VALID_MIN_US	800				// Min valid received pulse length in us
#define PULSIN_VALID_MAX_US	2200			// Max valid received pulse length in us


char cmd[PULSIN_LEN];	//Commands converted and validated by pulsin_process()
#define ROL	0			//Roll (rotation in YZ plane,around X axis) 	cmd[ROL] = -100 .. 100
#define PTC	1			//Pitch (rotation in XZ plane,around Y axis) 	cmd[PTC] = -100 .. 100
#define THR	2			//Throtle (motor speed)			cmd[THR] = 0 .. 100
#define YAW	3			//Yaw (rotation around Z axis)	cmd[YAW] = -100 .. 100
#define VRA 4			//VR(A) DIAL
#define VRB 5			//VR(B) DIAL



#define RANGE_MIN 0
#define RANGE_MAX 1

#define MID_MIN  2
#define MID_MAX  3

#define CMD_MIN	4	
#define CMD_MAX 5
#define CMD_MID 6

//pulsinUs[] are maped to cmd[] according to following map 
//Please note there's a "mid" zone, where cmd=0 if pulse in range of cmdMap[MID_MIN] .. cmdMap[MID_MAX]
// Mode 2 Transmitter, CH1 - NORM, CH2 - REV, CH3 - REV, CH4 - REV
// Tip for getting direction right, on 0 throtle: 
// When you move right stick left - right motor should turn on
// When you move right stick right - left motor should turn on
// When you move right stick forward - back motor should turn on
// When you move right stick back - forward motor should turn on
						
int cmdMap[PULSIN_LEN][7] = {
	//RANGE_MIN,RANGE_MAX, MID_MIN,MID_MAX, CMD_MIN,CMD_MAX,CMD_MID , MODE 2 6-CH TRANSMITTER
	{1100,1900,  1475,1525,  100,-100,0},		//CH1 ROL	RIGH STICK HORIZONTAL
	{1100,1900,  1475,1525,  -100,100,0},		//CH2 PTC	RIGH STICK VERTICAL
	{1100,1900,  1100,1200,  0,100,0},			//CH3 THR	LEFT STICK VERTICAL
	{1100,1900,  1475,1525,  -100,100,0},		//CH4 YAW	LEFT STICK HORIZONTAL
	{1100,1900,  1100,1200,  0,100,0},			//CH5 VR(A)	
	{1100,1900,  1100,1200,  0,100,0}			//CH6 VR(B)
};


#define FAULT_COUNT_PANIC	10 				//panic mode activated when pulsinFaultCount reaches this number
int pulsinFaultCount = FAULT_COUNT_PANIC;	//counter, invalid signal increases counter, valid siganl decreases counter


#define PULSIN_PANIC (pulsinFaultCount >= FAULT_COUNT_PANIC)

volatile unsigned int* pulsinPortPtr[PULSIN_LEN];		//Pointer to input port registers
unsigned char pulsinPortBit[PULSIN_LEN];				//Bit number in the port register
volatile unsigned int pulsinTimer[PULSIN_LEN];			//timer2 value when ON state strted
volatile unsigned char pulsinOn[PULSIN_LEN];			//signal is currently High
volatile unsigned int pulsinPulse[PULSIN_LEN];			//last measured pulse in timer2 ticks
unsigned int pulsinUs[PULSIN_LEN];						//pulse lengths in us 
volatile unsigned int pulsinOverflows[PULSIN_LEN];		//count overflows

#define PULSIN_ASSIGN_PIN(i,pin,CNIEBit)		pin(_TRIS_F) = 1; pulsinPortPtr[i] = 	pin(_PORT_PTR_F); \
												pulsinPortBit[i] = pin(_BIT_F); CNIEBit = 1;



//Setup timer and pointer to ports
void pulsin_init(){
	unsigned char i;
	for(i=0;i<PULSIN_LEN;i++){
		pulsinPulse[i] = 0;
		pulsinPortPtr[i] = 0;
		pulsinOn[i] = 1;		 //consider pulse is on, but we'll discard first pulse
		pulsinOverflows[i] = 2;	 //this will invalidate first pulse 
	}

	//Configure Timer2 5ticks = 1us (40Mhz FCY, 1:8 prescaler)
	ConfigIntTimer2(T2_INT_ON & T2_INT_PRIOR_5);
 	OpenTimer2(T2_ON & T2_GATE_OFF & T2_IDLE_STOP &
               T2_PS_1_8 & 
               T2_SOURCE_INT, 0xFFFF
	);

	IFS1bits.CNIF = 0;		//Reset CN interrupt
	IPC4bits.CNIP = 4;		//Change Notification Interrupt priority
	IEC1bits.CNIE = 1;		//Enable CN interrupts
}

//Convert and validate pulsin readings
//Set cmd[THR], cmd[YAW], cmd[PTC] , cmd[ROL]  
//returns 0 if invalid data detected, returns 1 on success 
//also updates fault counter
unsigned char pulsin_process(){
	unsigned char fault = 0;
	unsigned char i;
	for(i=0;i<PULSIN_LEN;i++){
		pulsinUs[i] = TK2_TO_US(pulsinPulse[i]);
		if(	pulsinUs[i] < PULSIN_VALID_MIN_US || pulsinUs[i] > PULSIN_VALID_MAX_US )
			fault = 1;
	}
	if(!fault){
		pulsinFaultCount = MAX(pulsinFaultCount - 2,0);	 //recover twice as fast 
		if(0 == pulsinFaultCount){
			for(i=0;i<PULSIN_LEN;i++){
				if( pulsinUs[i] < cmdMap[i][MID_MIN] ) 
					cmd[i] = float_to_int(map_to_range( pulsinUs[i] , cmdMap[i][RANGE_MIN], cmdMap[i][MID_MIN] , cmdMap[i][CMD_MIN] , cmdMap[i][CMD_MID] ));
				else if ( pulsinUs[i] > cmdMap[i][MID_MAX] )
					cmd[i] = float_to_int(map_to_range( pulsinUs[i] , cmdMap[i][MID_MAX], cmdMap[i][RANGE_MAX] , cmdMap[i][CMD_MID] , cmdMap[i][CMD_MAX] ));
				else
					cmd[i] = 0;
			}
		}
	}else{
		pulsinFaultCount = MIN(pulsinFaultCount+1,FAULT_COUNT_PANIC);	
	}

	return fault;
}


//this interrupt counts timer overflows for each channel
void __attribute__((__interrupt__, auto_psv)) _T2Interrupt(void){
	static unsigned char i;
	for(i=0;i<PULSIN_LEN;i++){
		pulsinOverflows[i]++;
		if( (unsigned long)TK2_OVERFLOW_US * pulsinOverflows[i] > PULSIN_MAX_NOCHANGE_US ){
			pulsinPulse[i] = 0;
		}
	}
	
	//SAFETY FEATURE SLOWLY SHUT DOWN THROTTLE
	static unsigned char itteration;
	if(PULSIN_PANIC){
		itteration++;
		if(0 == itteration % 4 && cmd[THR] > 0) // every 256*256 / 5 ~ 13107us * 4 ~ 52ms
			cmd[THR]--;
	}

    IFS0bits.T2IF = 0;    // Clear Timer interrupt flag
}  


//this interrupt detects changes in pin levels and sets pulsinPulse[] array
void __attribute__ ((__interrupt__, auto_psv)) _CNInterrupt(void)
{
	static unsigned char i; 

						
	IFS1bits.CNIF = 0; 	// Clear CNIF flag right away, 
						// This gives us the oportunity to catch another CN change while this interrupt executes.

	for(i=0;i<PULSIN_LEN;i++){
		if(pulsinPortPtr[i]){
			if((*pulsinPortPtr[i]) & (1U << pulsinPortBit[i])){
				if(0 == pulsinOn[i]){
					pulsinTimer[i] = ReadTimer2();
					pulsinOn[i] = 1;
					pulsinOverflows[i] = 0;
				}
			}else{
				//need a startup value for pulsinOn to avoid startup glitches !
				if(1 == pulsinOn[i]){
					//note that it's ok if ReadTimer2() < pulsinTimer[i], overflow works correctly here
					//as long as timer2 period is 0xFFFF
					if(pulsinOverflows[i]<2) pulsinPulse[i] = ReadTimer2() - pulsinTimer[i];	  
					pulsinOn[i] = 0;		
					pulsinOverflows[i] = 0;
				}
			}
		}
	}
	
}


#endif
