#include "stm32f10x.h"                  // Device header
#include "PID_Moctor.h"

//int32_t EncoderToSpeed(int16_t encoder) //为何要转为int64_t?避免encoder * Twopi数据溢出（int32_t)
//{
//    int64_t tmp = (int64_t)encoder * Twopi;   // encoder * 2π（Q15）
//    tmp = tmp * Car_R;                        // 乘以半径（Q15 * Q15 → Q30）
//    tmp = tmp / ENCODER_RESOLUTION;           // 除以编码器分辨率（Q30 → Q15）
//    tmp = tmp / DT_Q15;                       // 除以时间间隔（Q15 / Q15 → Q0，再转 Q15）
//    return (int32_t)(tmp >> Q15_SHIFT);       // 结果转为 Q15
//}
int32_t EncoderToSpeed(int16_t encoder)
{ // 正确：使用int32_t存储编码器值
    int64_t tmp = (int64_t)encoder * Twopi; // 正确：避免溢出
    tmp = (tmp * Car_R) >> Q15_SHIFT; // 调整运算顺序及移位
    tmp = tmp / ENCODER_RESOLUTION;
    tmp = (tmp << Q15_SHIFT) / DT_Q15; // 保持Q15精度
    return (int32_t)(tmp >> Q15_SHIFT);
}

void PID_GetSpeed(Car_Speed* car_speed, int32_t lw, moctor_speed* MoctorSpeed)
{
	MoctorSpeed[0].Target = car_speed->Vx * 0 + car_speed->Vy + lw;
	MoctorSpeed[1].Target = ((-car_speed->Vx * COS30) >> Q15_SHIFT) - ((car_speed->Vy * COS60) >> Q15_SHIFT) + lw;
	MoctorSpeed[2].Target = ((car_speed->Vx * COS30) >> Q15_SHIFT) - ((car_speed->Vy * COS60) >> Q15_SHIFT) + lw;
}

void Square_Path(int32_t length, int32_t t, Car_Speed* car_speed)
{
	int64_t V = ((int64_t)length << Q15_SHIFT) / Square_t;//为什么不能直接length / Square_t：整数除法会丢失小数点的精确度，
	//要把length转化为Q15,Q15的除法运算得到的任然是Q15
	if (t < 0 || t > 4 * Square_t)
	{
		car_speed->Vx = 0;
		car_speed->Vy = 0;
    } 
	else if (t <= Square_t) 
	{           
        car_speed->Vx = V;
		car_speed->Vy = 0;
    } 
	else if (t <= 2 * Square_t) 
	{       
        car_speed->Vx = 0;
		car_speed->Vy = V;
    } 
	else if (t <= 3 * Square_t) 
	{       
        car_speed->Vx = -V;
		car_speed->Vy = 0;
    } 
	else 
	{                              
        car_speed->Vx = 0;
		car_speed->Vy = -V;
    }
}


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
		if(speed->ErrorInt >   ((int64_t)FLOAT_TO_Q15(1.0f)  << Q15_SHIFT)/ speed->Ki)
		{
			speed->ErrorInt = ((int64_t)FLOAT_TO_Q15(1.0f) << Q15_SHIFT)/ speed->Ki;
		}
		else if(speed->ErrorInt < -((int64_t)FLOAT_TO_Q15(1.0f) << Q15_SHIFT)/ speed->Ki)
		{
			speed->ErrorInt = -((int64_t)FLOAT_TO_Q15(1.0f) << Q15_SHIFT)/ speed->Ki;
		}	
		
		speed->Out = speed->Kp * speed->Error0 
		+ speed->Ki * speed->ErrorInt 
		+ speed->Kd * (speed->Error0 - speed->Error1);
			
		//输出限幅（由Motor_SetPWM_D(Out)决定）
		if (speed->Out > FLOAT_TO_Q15(1.0f)) {speed->Out = FLOAT_TO_Q15(1.0f);}//为什么是1.0f
		if (speed->Out < -FLOAT_TO_Q15(1.0f)) {speed->Out = -FLOAT_TO_Q15(1.0f);}
			
}

void MoctorC_Speed(moctor_speed* speed)
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
		if(speed->ErrorInt >  ((int64_t)FLOAT_TO_Q15(1.0f) << Q15_SHIFT)/ speed->Ki)
		{
			speed->ErrorInt = ((int64_t)FLOAT_TO_Q15(1.0f) << Q15_SHIFT) / speed->Ki;
		}
		else if(speed->ErrorInt < -((int64_t)FLOAT_TO_Q15(1.0f) << Q15_SHIFT) / speed->Ki)
		{
			speed->ErrorInt = -((int64_t)FLOAT_TO_Q15(1.0f) << Q15_SHIFT) / speed->Ki;
		}	
		
		speed->Out = speed->Kp * speed->Error0 
		+ speed->Ki * speed->ErrorInt 
		+ speed->Kd * (speed->Error0 - speed->Error1);
			
		//输出限幅（由Motor_SetPWM_D(Out)决定）
		if (speed->Out > FLOAT_TO_Q15(1.0f)) {speed->Out = FLOAT_TO_Q15(1.0f);}
		if (speed->Out < -FLOAT_TO_Q15(1.0f)) {speed->Out = -FLOAT_TO_Q15(1.0f);}
			
}

void MoctorD_Speed(moctor_speed* speed)
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
		if(speed->ErrorInt >  ((int64_t)FLOAT_TO_Q15(1.0f) << Q15_SHIFT) / speed->Ki)
		{
			speed->ErrorInt = ((int64_t)FLOAT_TO_Q15(1.0f) << Q15_SHIFT) / speed->Ki;
		}
		else if(speed->ErrorInt < -((int64_t)FLOAT_TO_Q15(1.0f) << Q15_SHIFT) / speed->Ki)
		{
			speed->ErrorInt = -((int64_t)FLOAT_TO_Q15(1.0f) << Q15_SHIFT) / speed->Ki;
		}	
		
		speed->Out = speed->Kp * speed->Error0 
		+ speed->Ki * speed->ErrorInt 
		+ speed->Kd * (speed->Error0 - speed->Error1);
			
		//输出限幅（由Motor_SetPWM_D(Out)决定）
		if (speed->Out > FLOAT_TO_Q15(1.0f)) {speed->Out = FLOAT_TO_Q15(1.0f);}
		if (speed->Out < -FLOAT_TO_Q15(1.0f)) {speed->Out = -FLOAT_TO_Q15(1.0f);}
			
}
