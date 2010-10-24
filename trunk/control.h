#ifndef CONTROL_H
#define CONTROL_H

float control_pid(float error,float iError, float dError,   float Kp, float Ki, float Kd){
	return error*Kp + iError * Ki + dError * Kd;
}



#endif
