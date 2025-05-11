#include "stm32f10x.h"                  // Device header
#include "PID_MoctorC_Location.h"
#include <math.h>

void MoctorC_Location(moctor_location* location)
{
		location->Actual1	= location->Actual;
//		location->Actual += Encoder_Get_TIM2();
		
		location->Error1 = location->Error0;
		location->Error0 = location->Target - location->Actual;
		
//变速积分 
//float C = 1 / (K * fabs(Error0) + 1);
//ErrorInt += C * Error0;


//积分分离
		if (fabs(location->Error0) < 20)
		{
			location->ErrorInt += location->Error0;
		}
		else
		{
			location->ErrorInt = 0;
		}
//积分限幅 -》抗积分饱和
		if(location->ErrorInt >  100 / location->Ki)
		{
			location->ErrorInt = 100 / location->Ki;
		}
		else if(location->ErrorInt < -100 / location->Ki)
		{
			location->ErrorInt = -100 / location->Ki;
		}
//		Difout = (1 - a) * Kd * (Error0 - Error1) + a * Difout;
		
		
		
//1.    location->Out = location->Kp * location->Error0
//		+ location->Ki * location->ErrorInt
//		+ location->Kd * (location->Error0 - location->Error1);普通PID
		
		location->Out = location->Kp * location->Error0
		+ location->Ki * location->ErrorInt
		- location->Kd * (location->Actual - location->Actual1);//微分先行
		
//3.    location->Out = location->Kp * location->Error0 
//		+ location->Ki * location->ErrorInt
//		+ location->Difout;//不完全微分

		//输出限幅（由Motor_SetPWM_D(Out)决定）
		if (location->Out > 100) {location->Out = 100;}
		if (location->Out < -100) {location->Out = -100;}
}
