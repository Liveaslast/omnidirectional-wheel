#ifndef _PID_MOCTORC_SPEED_H
#define _PID_MOCTORC_SPEED_H

typedef struct 
{
	float Target;
	float Actual;
	float Out;
	float Kp;
	float Ki;
	float Kd;
	float Error0;
	float Error1;
	float ErrorInt;
	
}moctor_speed;

void MoctorC_Speed(moctor_speed* speed);
#endif