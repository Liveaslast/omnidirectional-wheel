#ifndef _PID_MOCTORD_LOCATION_H
#define _PID_MOCTORD_LOCATION_H

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
	
}moctorD_location;

void MoctorD_Location(moctorD_location* location);

#endif
