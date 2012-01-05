#ifndef CONTROL_H
#define CONTROL_H

struct Pid {
	float Kp;
	float Ki;
	float Kd;
	float value;
	float target;
	float error;
	float iError;
	float dError;
	float control;
} pid[4] ={									//initialize Pid structures
	{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},	//rol
	{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},	//pitch
	{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},	//throtle
	{0.0,0.0,50.0,0.0,0.0,0.0,0.0,0.0,0.0}	//yaw
};

#define roll pid[ROL]
#define pitch pid[PTC]
#define throt pid[THR]
#define yaw pid[YAW]

float control_pid(float error,float iError, float dError,   float Kp, float Ki, float Kd){
	return error*Kp + iError * Ki + dError * Kd;
}

//get inclination on X/Y axes (pitch/roll)
float inclination_deg(float xy , float z){
	float incl = asin(xy)*180.0/PI;
	if(z < 0 && incl < -45) return -180.0 - incl; 
	if(z < 0 && incl > 45) return 180.0 - incl;
	return incl;
}


#endif
