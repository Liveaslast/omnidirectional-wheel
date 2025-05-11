#ifndef _PID_MOCTOR_H
#define _PID_MOCTOR_H

#define Q15_SHIFT 15
#define Max_Range 6.283f
#define FLOAT_TO_Q15(x) ((int32_t)((x) * (1 << Q15_SHIFT) + 0.5)) // 浮点数转Q15
#define Q15_TO_FLOAT(x) ((float)(x) / (1 << Q15_SHIFT)) // Q15转浮点数
#define ENCODER_RESOLUTION  1500 
#define Car_R FLOAT_TO_Q15(0.07f / Max_Range)//-->0.07m
#define DT_Q15 FLOAT_TO_Q15(0.04f / Max_Range)//-->0.04s


#define COS30 FLOAT_TO_Q15(0.866f / Max_Range)//-->0.866f
#define COS60 FLOAT_TO_Q15(0.5f / Max_Range)//-->0.5f
#define Square_t FLOAT_TO_Q15(2.0f / Max_Range)//--> 2s
#define Length FLOAT_TO_Q15(2.0f / Max_Range)//square -->2m
#define Twopi FLOAT_TO_Q15(6.283f / Max_Range)//-->6.283f
#define Out_Max 100

typedef struct 
{
	int32_t Target;
	int32_t Actual;
	int32_t Out;
	int32_t Kp;
	int32_t Ki;
	int32_t Kd;
	int32_t Error0;
	int32_t Error1;
	int64_t ErrorInt;//64位防止溢出？
	
}moctor_speed;

typedef struct
{
	int32_t Vx;
	int32_t Vy;
}Car_Speed;

void MoctorA_Speed(moctor_speed* speed);
void MoctorC_Speed(moctor_speed* speed);
void MoctorD_Speed(moctor_speed* speed);
void PID_GetSpeed(Car_Speed* car_speed, int32_t lw, moctor_speed* MoctorSpeed);
void Square_Path(int32_t length, int32_t t, Car_Speed* car_speed);
int32_t EncoderToSpeed(int16_t encoder);
#endif
