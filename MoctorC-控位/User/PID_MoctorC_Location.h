#ifndef _PID_MOCTORC_LOCATION_H
#define _PID_MOCTORC_LOCATION_H

typedef struct
{
	float Target;
	float Actual;
	float Actual1;
	float Out;
	float Kp;
	float Ki;
	float Kd;
	float Error0;
	float Error1;
	float ErrorInt;
	
}moctor_location;

void MoctorC_Location(moctor_location* location);

#endif
