#ifndef _PID_MOCTOR_H
#define _PID_MOCTOR_H

#include "math.h"

#define twopi 6.283f
#define Car_R 0.07f
#define COS30 0.866f
#define COS60 0.5f

//cosf((1 / Circle_N) * twopi)
//sinf((1 / Circle_N) * twopi) 
#define COS_Theta_theree  0.9965f //Circle_t = 3s - Circle_R = 0.25f
#define SIN_Theta_three  0.0838f  //Circle_t = 3s - Circle_R = 0.25f
#define COS_Theta_six  0.999657f  //Circle_t = 6s - Circle_R = 0.5f
#define SIN_Theta_six  0.039064f  //Circle_t = 6s - Circle_R = 0.5f
#define Square_t 2.0f
#define Triangle_t 2.0f
#define Circle_t 6.0f

#define Length 1.0f // --三角形与正方形的边
#define Circle_R 0.5f // --圆形轨迹半径

#define Square_V  Length / Square_t
#define Triangle_V Length / Triangle_t
#define Circle_V (twopi * Circle_R) / Circle_t
#define Circle_W Circle_V / Circle_R

#define Circle_N Circle_t / 0.04f //圆圈的近似多边形



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

typedef struct
{
	float Vx;
	float Vy;
	float phase;//相位
	int flag ;
}Car_Speed;

void MoctorA_Speed(moctor_speed* speed);
void MoctorC_Speed(moctor_speed* speed);
void MoctorD_Speed(moctor_speed* speed);
void PID_Speed(moctor_speed* speed);
void PID_GetSpeed(Car_Speed* car_speed, float lw, moctor_speed* MoctorSpeed);
void Square_Path(float t, Car_Speed* car_speed);
void Triangle_Path(float t, Car_Speed* car_speed);
void Circle_Path(Car_Speed* car_speed);

#endif
