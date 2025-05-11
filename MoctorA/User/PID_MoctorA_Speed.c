#include "stm32f10x.h"                  // Device header
#include  "PID_MoctorA_Speed.h"

void MoctorA_Speed(moctor_speed* speed)
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

