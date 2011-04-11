#ifndef CONFIG_H
#define CONFIG_H

/* 
****** PLEASE READ THIS *******
dsPic33 has no EEPROM so we use flash memory to store config data during production
Flash memory can be overwritten 100-1000 times only (estimated) see http://www.microchip.com/forums/m249893.aspx
IMPORTANT: during development config will be ovewritten since it is essentially part of program memory
the solution is to hard-code calibration values in config_default()
Also see:
http://www.microchip.com/forums/tm.aspx?m=532946&high=_write_flash16

*/

//reserve a page of Flash for config storage
char  __attribute__((space(prog),aligned(_FLASH_PAGE*2))) dat[_FLASH_PAGE*3];

//1 flash page = 512 instructions = 1536 bytes (8 rows)
//1 flash row = 64 instructions = 192 bytes

//From libpic30.h:
/* constants for 33F devices */
//#define _FLASH_PAGE   512
//#define _FLASH_ROW     64


#define CONFIG_SIGNATURE 0xAD01
#define CONFIG_VERSION 1

struct {
	// !!!!! sizeof(config) should be even (add padding byte if necessary)
	// !!!!! sizeof(config) <= _FLASH_ROW * 2 =  64 * 2  

	unsigned int signature;
	unsigned int writeCount;	//counts number of config writes, resets to 0 at 0xFFFF

	unsigned char accInv[3];		// invert accl input (for example due to physical mounting position)
	double accOffs[3];				// accl output at 0g in ADC units
	double accSens[3];				// accl input sensitivity in ADC/g

	unsigned char gyroInv[3];		// invert gyro input (for example due to physical mounting position)
	double gyroOffs[3];				// gyro zero rate output in ADC  @ 0 deg/s;
	double gyroSens[3];				// gyro input sensitivity ADC/(deg/ms)

} config;

void config_default(){
	config.signature = CONFIG_SIGNATURE;
	config.writeCount = 0;

	//Theoretical values entered in comments, we use actual hard-coded calibrated values for development
	//to avoid unnecessary Flash writes

	// Settings for Acc_Gyro board http://www.gadgetgangster.com/213 (update based on your sensors)
	// Accelerometer offset: 1.65V  = 1023 * 1.65/3.3 = 511.5 ADC
	// Accelerometer sensisitivity 478 mV/g = 1023 * 0.478 / 3.3 = 148.18 ADC/g

	config.accOffs[0] = 511.81;
	config.accOffs[1] = 509.10;
	config.accOffs[2] = 506.96;

	config.accSens[0] = 147.96;
	config.accSens[1] = 150.82;
	config.accSens[2] = 144.87;

	// Gyro X,Y axes (PITCH, ROLL)
	// Gyro Zero Level 1.23V @ 0 deg/s =  1023 * 1.23 / 3.3 = 381.3 ADC
	// 1 mV / (deg/s) = 1023/3300 ADC / ( (PI/180)rad / 1000ms  ) = 17761.69165 ADC /(rad/ms)
	// Gyro Sensitivity 2 mV/(deg/s) = 2 * 17761.69165 ADC /(rad/ms) = 35523.3833 ADC /(rad/ms)
	// 
	
	config.gyroOffs[0] = 383.15;		
	config.gyroOffs[1] = 385.57;		

	config.gyroSens[0] = 35523.3833;		
	config.gyroSens[1] = 35523.3833;		


	// Settinsg for LISY300AL Single-Axis Gyro  http://www.pololu.com/catalog/product/765
	// Gyro for Z axis (YAW)  
	// Gyro Zero Level 1.65V @ 0 deg/s = 1023 * 1.65/3.3 = 511.5 ADC
	// Gyro Sensitivity 3.3 mV/(deg/s) =  3.3* 17761.69165 ADC /(rad/ms) = 58613.58245 ADC /(rad/ms)
	config.gyroOffs[2] = 516.38;
	config.gyroSens[2] = 58613.58245;

	config.accInv[0] = 0;
	config.accInv[1] = 0;
	config.accInv[2] = 0;

	config.gyroInv[0] = 0;
	config.gyroInv[1] = 1;
	config.gyroInv[2] = 1;


}


void config_save(){
	config.writeCount++;
	_prog_addressT p;
	_init_prog_address(p, dat); 		//get address in program space
	_erase_flash(p);  					//erase a flash page 
    _write_flash16(p,(int*)(&config)); 	//write a row of flash data
	printf("!!! FLASH WRITTEN !!! DO NOT ABUSE THIS !!! LIMITED WRITES AVAILABLE !!!\n");
}


void config_print(){
	printf("***CONFIG SIG:%x writeCount:%x***\n",config.signature,config.writeCount);
	printf("Accl Offs: %.2f,%.2f,%.2f Sens: %.2f,%.2f,%.2f Inv:%d%d%d\n"
		,config.accOffs[0],config.accOffs[1],config.accOffs[2]
		,config.accSens[0],config.accSens[1],config.accSens[2]
		,config.accInv[0],config.accInv[1],config.accInv[2]
	);

	printf("Gyro Offs: %.2f,%.2f,%.2f Sens: %.2f,%.2f,%.2f Inv:%d%d%d\n"
		,config.gyroOffs[0],config.gyroOffs[1],config.gyroOffs[2]
		,config.gyroSens[0],config.gyroSens[1],config.gyroSens[2]
		,config.gyroInv[0],config.gyroInv[1],config.gyroInv[2]
	);
}



void config_load(){
	_prog_addressT p;
	_init_prog_address(p, dat); 	//get address in program space
	_memcpy_p2d16(&config,p,sizeof(config));

	if(config.signature != CONFIG_SIGNATURE){
		//load defaults
		config_default();
	}

}


#endif
