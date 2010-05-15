#ifndef IMU__H
#define IMU__H

// NOTE: If your gyros fail to calibrate on startup
// Make sure device is idle on startup.
// Check gyro's config.zeroLevel values (not only in datasheet but in reality ! measure with a multimeter).
// Obsrve gyro's ADC values with adc_debug_dump() and increase config.adcGyroNoise if necessary


unsigned int sequence;  			//estimation sequence
unsigned int interval;				//interval since last estimation in us 
unsigned char firstLoop = 1;		//marks first estimation

#define IMU_SMOOTHING 20			//this influences smoothing of estimates and gyro weight in accelerometer-gyro fusion

#define GYRO_CALIBRATE_LOOPS 500	//determines how long the gyro will be hold in calibration sequence


struct{
	unsigned char invert;			// invert accl input (for example due to physical mounting position)
	unsigned int sensitivity;		// accl input sensitivity in mv/g
	unsigned int zeroG_mv;			// accl output at 0g
} accl[3];					

struct{
	unsigned char invert;			// invert gyro input (for example due to physical mounting position)
	unsigned int sensitivity;		// gyro input sensitivity in uV/deg/s same as in  mV/deg/ms 
	unsigned int zeroRate_mv;		// gyro zero rate output in mV  @ 0 deg/s
	unsigned int calibAdcSpan;		// max gyro readings variation in adc units (adcMax-adcMin) while in idle position
	double calibZeroRate_mv;		// calibrated zero rate voltage
	unsigned int calibAdcMin;		// minimum Adc value during calibration
	unsigned int calibAdcMax;		// maximum Adc value during calibration
} gyro[3];


void gyro_calibrate();

void imu_init(){
	unsigned char w;
	firstLoop = 1;
	sequence = 0;

	//settings for Acc_Gyro board http://www.gadgetgangster.com/213 (update based on your sensors)
	for(w=0;w<3;w++){					// Accel X,Y,Z axes
		accl[w].zeroG_mv = 1650;		// Accelerometer zero level (mV) @ 0 G
		accl[w].sensitivity = 478; 		// Accelerometer sensisitivity mV/g
	}

	for(w=0;w<2;w++){					// Gyro X,Y axes (PITCH, ROLL)
 		gyro[w].zeroRate_mv = 1230;		// Gyro Zero Level (mV) @ 0 deg/s
		gyro[w].sensitivity = 2000;		// Gyro Sensitivity uV/deg/s (or mV/deg/ms)
		gyro[w].calibAdcSpan = 10;		// max gyro adc variation in idle position
	}

	// Gyro for Z axis (YAW),  LISY300AL Single-Axis Gyro  http://www.pololu.com/catalog/product/765
	w = 2;
	gyro[w].zeroRate_mv = 1650;			// Gyro Zero Level (mV) @ 0 deg/s
	gyro[w].sensitivity = 3300;			// Gyro Sensitivity uV/deg/s (or mV/deg/ms)
	gyro[w].calibAdcSpan = 10;			// max gyro adc variation in idle position

	gyro_calibrate();
}


void gyro_calibrate(){
	unsigned char w;
	unsigned char fault = 1;
	int counter;

	while(fault){
		counter = 0;
		for(w=0;w<3;w++){
			gyro[w].calibZeroRate_mv = 0;
			gyro[w].calibAdcMin = 1023;
			gyro[w].calibAdcMax = 0;
		}

		while(counter < GYRO_CALIBRATE_LOOPS){
			//wait for new adc data
			while(!adc_new_data());
			fault = 0;
	
			for(w=0;w<3;w++){
				gyro[w].calibZeroRate_mv += adc_to_mv(adcAvg[3+w]);
				gyro[w].calibAdcMin = MIN(adcMin[3+w],gyro[w].calibAdcMin);
 				gyro[w].calibAdcMax = MAX(adcMax[3+w],gyro[w].calibAdcMax);
				if(gyro[w].calibAdcMax  - gyro[w].calibAdcMin > gyro[w].calibAdcSpan){	
					fault = 1;
					printf("Gyro calibration FAULT\n  Loop:%d Axis:%d adcMin:%d adcMax:%d \n", 
						counter,w, gyro[w].calibAdcMin , gyro[w].calibAdcMax 
					);
					adc_debug_dump();

				}
			}
	 

			if(!fault)
				counter++;
			else
				counter = GYRO_CALIBRATE_LOOPS;
			
			//__delay_ms(10);
		}
	}	

	printf("Gyro Calibration complete\n");
	for(w=0;w<3;w++){
		gyro[w].calibZeroRate_mv /= GYRO_CALIBRATE_LOOPS;
		printf("gyro[%d].calibZeroRate_mv: %f mV adcMin: %d adcMax: %d \n",w, gyro[w].calibZeroRate_mv, gyro[w].calibAdcMin, gyro[w].calibAdcMax);
	}
	printf("***********************\n");


}



//Get accelerometer reading (accelration expressed in g)
float getAcclOutput(unsigned char w){
	static float tmpf;		//temporary variable for complex calculations
	tmpf = adc_to_mv(adcAvg[w]);		//output voltage in mV
	tmpf -= accl[w].zeroG_mv;			//voltage relative to 0g output mV
	tmpf /= accl[w].sensitivity;		//divide by input sensitivity in mV/g , to get value in g
	if( accl[w].invert) tmpf = - tmpf;	//invert axis value if needed
	return tmpf;		
}

//Get gyro reading (rate of rotation expressed in deg/ms) 
float getGyroOutput(unsigned char w){
	static float tmpf;					//temporary variable for complex calculations
	tmpf = adc_to_mv(adcAvg[3+w]);		//output voltage in mV
	tmpf -= gyro[w].calibZeroRate_mv;	//voltage relative to 0g output mV
	tmpf /= gyro[w].sensitivity;		//divide by input sensitivity in mV/deg/ms  , to get value in deg/ms
	if( gyro[w].invert) tmpf = - tmpf;	//invert axis value if needed
	return tmpf;		
}

void normalize3DVector(double* vector){
	static double R;  
	R = vector[0]*vector[0];
	R += vector[1]*vector[1];
	R += vector[2]*vector[2];
	R = sqrt(R);
	vector[0] /= R;
	vector[1] /= R;  
	vector[2] /= R;  
}


//-------------------------------------------------------------------
// Global
//-------------------------------------------------------------------

//Notation "w" stands for one of the axes, so for example RwAcc[0],RwAcc[1],RwAcc[2] means RxAcc,RyAcc,RzAcc
//Variables below must be global (their previous value is used in getEstimatedInclination)
double RwEst[3];     	//Rw estimated from combining RwAcc and RwGyro

//Variables below don't need to be global but we expose them for debug purposes
double RwAcc[3];        //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer
double RwGyro[3];       //Rw obtained from last estimated value and gyro movement
double Awz[2];          //angles between projection of R on XZ/YZ plane and Z axis (deg)


//-------------------------------------------------------------------
// getEstimatedInclination
//-------------------------------------------------------------------
void getEstimatedInclination(unsigned int interval_us){
	static int w;
	static float tmpf;  			//temp variable

	sequence++;
  
	//get accelerometer readings in g, gives us RwAcc vector
	for(w=0;w<=2;w++){ 
		RwAcc[w] = getAcclOutput(w);
		if(firstLoop) RwEst[w] = RwAcc[w];
	}
  
  	//normalize vector (convert to a vector with same direction and with length 1)
  	normalize3DVector(RwAcc);
  
	if(!firstLoop){
		//evaluate RwGyro vector
		if(fabs(RwEst[2]) < 0.1){
			//Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
			//in this case skip the gyro data and just use previous estimate
			for(w=0;w<=2;w++) RwGyro[w] = RwEst[w];
		}else{
			

			//get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst

			//info.debug[2] = FLOAT2INT(atan2_aprox(0.100,0.300)*1000);		

			for(w=0;w<=1;w++){
				Awz[w] = atan2(RwEst[w],RwEst[2]);  //get angle in radians
				tmpf = getGyroOutput(w);	        //get current gyro rate in deg/ms, default to 0 

				tmpf /= 1000.0f;					//convert to deg/us				
				tmpf *= interval_us;	            //get angle change in degrees
				tmpf *= PI / 180;					//convert fron degrees to radians

 				Awz[w] += tmpf;                     //get updated angle according to gyro movement

			}
			
			//reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
			for(w=0;w<=1;w++){
				tmpf = squared(cos(Awz[w]));
				tmpf *= squared(tan(Awz[1-w]));
				tmpf += 1;
				RwGyro[w] = sin(Awz[w]);
				RwGyro[w] /= sqrt(tmpf);
			}

			tmpf = 1.0;
			tmpf -= squared(RwGyro[0]);
			tmpf -= squared(RwGyro[1]);
			RwGyro[2] = sqrt(tmpf);

			//estimate sign of RzGyro by looking in what qudrant the angle Axz is, 
			//RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
			if ( cos(Awz[0]) < 0.0f ) RwGyro[2] = - RwGyro[2];


		}
		
		//New estimate is weighted average of (Current Accelerometer) and (Last Estimated + Current Gyro) 
		for(w=0;w<=2;w++){
			RwEst[w] =  IMU_SMOOTHING; 
			RwEst[w] *= RwGyro[w];
			RwEst[w] += RwAcc[w];
			RwEst[w] /= (float)(1 + IMU_SMOOTHING);
		}

		normalize3DVector(RwEst);   


		//IMPORTANT DEBUG SEQUENCE TO TEST GYRO INTEGRATION
		//RUN THIS TEST AND MONITOR RwAcc and RwEst
		/*
		for(w=0;w<=2;w++){
			if( info.sequence  % 255){
				RwEst[w] = RwGyro[w];
			}else{
				//reset to acc readings 
				RwEst[w] = RwAcc[w];
			}

		}
		*/
		
	}
  
	firstLoop = 0;
}




#endif
