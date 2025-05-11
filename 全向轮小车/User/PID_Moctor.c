#include "stm32f10x.h"                  // Device header
#include "PID_Moctor.h"



void PID_GetSpeed(Car_Speed* car_speed, float lw, moctor_speed* MoctorSpeed)
{
	MoctorSpeed[0].Target = (car_speed->Vx * 0 + car_speed->Vy + lw) * 0.04  * 1500 / (twopi * Car_R);
	MoctorSpeed[1].Target = (-car_speed->Vx * COS30 + car_speed->Vy * COS60 + lw) * 0.04  * 1500 / (twopi * Car_R);
	MoctorSpeed[2].Target = (car_speed->Vx * COS30 + car_speed->Vy * COS60 + lw) * 0.04  * 1500 / (twopi * Car_R);
}


//void PID_GetSpeed(Car_Speed* car_speed, float lw, moctor_speed* MoctorSpeed)
//{
//	MoctorSpeed[0].Target = (car_speed->Vx * 0 + car_speed->Vy + lw) * 0.04  * 1500 / (twopi * Car_R);
//	MoctorSpeed[1].Target = (-car_speed->Vx * cos(30.0f) + car_speed->Vy * cos(60.0f) + lw) * 0.04  * 1500 / (twopi * Car_R);
//	MoctorSpeed[2].Target = (car_speed->Vx * cos(30.0f) + car_speed->Vy * cos(60.0f) + lw) * 0.04  * 1500 / (twopi * Car_R);
//}
void Square_Path(float t, Car_Speed* car_speed)
{
	
	if (t < 0 || t > 4 * Square_t)
	{
		car_speed->Vx = 0;
		car_speed->Vy = 0;
    } 
	else if (t <= Square_t && t > 0) 
	{           
        car_speed->Vx = Square_V;
		car_speed->Vy = 0;
    } 
	else if (t <= 2 * Square_t && t > Square_t) 
	{       
        car_speed->Vx = 0;
		car_speed->Vy = Square_V;
    } 
	else if (t <= 3 * Square_t && t > 2 * Square_t) 
	{       
        car_speed->Vx = -Square_V;
		car_speed->Vy = 0;
    } 
	else 
	{                              
        car_speed->Vx = 0;
		car_speed->Vy = -Square_V;
    }
}


void Triangle_Path(float t, Car_Speed* car_speed)
{
	
	if (t < 0 || t > 3 * Triangle_t)
	{
		car_speed->Vx = 0;
		car_speed->Vy = 0;
    } 
	else if (t <= Triangle_t && t > 0) 
	{           
        car_speed->Vx = COS30 * Triangle_V;
		car_speed->Vy = COS60 * Triangle_V;

    } 
	else if (t <= 2 * Triangle_t && t > Triangle_t) 
	{       
        car_speed->Vx = -COS30 * Triangle_V;
		car_speed->Vy = COS60 * Triangle_V;

    } 
	else if (t <= 3 * Triangle_t && t > 2 * Triangle_t) 
	{       
        car_speed->Vx = 0;
		car_speed->Vy = -Triangle_V;
    } 
}

void Circle_Path(Car_Speed* car_speed)
{
	
	if(car_speed->flag <= 0 || car_speed->flag > (Circle_N + 10))
	{
		car_speed->Vx = 0;
		car_speed->Vy = 0;
	}
	else if(car_speed->flag == 1)
	{
		
		car_speed->Vx = Circle_V;
		car_speed->Vy = 0;
	}
	else
	{
		float Vx_New = car_speed->Vx * COS_Theta_six - car_speed->Vy * SIN_Theta_six;
		float Vy_New = car_speed->Vx * SIN_Theta_six + car_speed->Vy * COS_Theta_six;
		float mag = sqrtf(Vx_New* Vx_New + Vy_New * Vy_New);
	
		car_speed->Vx = Vx_New * Circle_V / mag;
		car_speed->Vy = Vy_New * Circle_V / mag;
		

	}
		
}	

void PID_Speed(moctor_speed* speed)
{

//		speed->Actual = Encoder_Get_TIM2();
			
		speed->Error1 =speed->Error0;
		speed->Error0 = speed->Target - speed->Actual;
			
		if (speed->Ki != 0)
		{
			speed->ErrorInt += speed->Error0;
		}
		else
		{
			speed->ErrorInt = 0;
		}
		
//积分限幅 -》抗积分饱和
		if(speed->ErrorInt >  100 / speed->Ki)
		{
			speed->ErrorInt = 100 / speed->Ki;
		}
		else if(speed->ErrorInt < -100 / speed->Ki)
		{
			speed->ErrorInt = -100 / speed->Ki;
		}	
		
		speed->Out = speed->Kp * speed->Error0 
		+ speed->Ki * speed->ErrorInt 
		+ speed->Kd * (speed->Error0 - speed->Error1);
			
		//输出限幅（由Motor_SetPWM_D(Out)决定）
		if (speed->Out > 100) {speed->Out = 100;}
		if (speed->Out < -100) {speed->Out = -100;}
}

//void MoctorA_Speed(moctor_speed* speed)
//{
//	
////		speed->Actual = Encoder_Get_TIM2();
//			
//		speed->Error1 =speed->Error0;
//		speed->Error0 = speed->Target - speed->Actual;
//			
//		if (speed->Ki != 0)
//		{
//			speed->ErrorInt += speed->Error0;
//		}
//		else
//		{
//			speed->ErrorInt = 0;
//		}
//		
////积分限幅 -》抗积分饱和
//		if(speed->ErrorInt >  100 / speed->Ki)
//		{
//			speed->ErrorInt = 100 / speed->Ki;
//		}
//		else if(speed->ErrorInt < -100 / speed->Ki)
//		{
//			speed->ErrorInt = -100 / speed->Ki;
//		}	
//		
//		speed->Out = speed->Kp * speed->Error0 
//		+ speed->Ki * speed->ErrorInt 
//		+ speed->Kd * (speed->Error0 - speed->Error1);
//			
//		//输出限幅（由Motor_SetPWM_D(Out)决定）
//		if (speed->Out > 100) {speed->Out = 100;}
//		if (speed->Out < -100) {speed->Out = -100;}
//			
//}

//void MoctorC_Speed(moctor_speed* speed)
//{
//	
////		speed->Actual = Encoder_Get_TIM2();
//			
//		speed->Error1 =speed->Error0;
//		speed->Error0 = speed->Target - speed->Actual;
//			
//		if (speed->Ki != 0)
//		{
//			speed->ErrorInt += speed->Error0;
//		}
//		else
//		{
//			speed->ErrorInt = 0;
//		}
//		
////积分限幅 -》抗积分饱和
//		if(speed->ErrorInt >  100 / speed->Ki)
//		{
//			speed->ErrorInt = 100 / speed->Ki;
//		}
//		else if(speed->ErrorInt < -100 / speed->Ki)
//		{
//			speed->ErrorInt = -100 / speed->Ki;
//		}	
//		
//		speed->Out = speed->Kp * speed->Error0 
//		+ speed->Ki * speed->ErrorInt 
//		+ speed->Kd * (speed->Error0 - speed->Error1);
//			
//		//输出限幅（由Motor_SetPWM_D(Out)决定）
//		if (speed->Out > 100) {speed->Out = 100;}
//		if (speed->Out < -100) {speed->Out = -100;}
//			
//}

//void MoctorD_Speed(moctor_speed* speed)
//{
//	
////		speed->Actual = Encoder_Get_TIM2();
//			
//		speed->Error1 =speed->Error0;
//		speed->Error0 = speed->Target - speed->Actual;
//			
//		if (speed->Ki != 0)
//		{
//			speed->ErrorInt += speed->Error0;
//		}
//		else
//		{
//			speed->ErrorInt = 0;
//		}
//		
////积分限幅 -》抗积分饱和
//		if(speed->ErrorInt >  100 / speed->Ki)
//		{
//			speed->ErrorInt = 100 / speed->Ki;
//		}
//		else if(speed->ErrorInt < -100 / speed->Ki)
//		{
//			speed->ErrorInt = -100 / speed->Ki;
//		}	
//		
//		speed->Out = speed->Kp * speed->Error0 
//		+ speed->Ki * speed->ErrorInt 
//		+ speed->Kd * (speed->Error0 - speed->Error1);
//			
//		//输出限幅（由Motor_SetPWM_D(Out)决定）
//		if (speed->Out > 100) {speed->Out = 100;}
//		if (speed->Out < -100) {speed->Out = -100;}
//			
//}
