#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "LED.h"
#include "OLED.h"
#include "Encoder.h"
#include "Timer.h"
#include "Motor.h"
#include "PWM.h"
#include "Serial.h"
#include "PID_MoctorD_Location.h"
#include <math.h>

/*error日志
==============================================================================
1.移植函数未调整端口引脚设置，时钟设置（APB1/APB2)
2.未初始化Motor_Init,引用头文件
3.时钟配置参数TIM1、TIM2、TIM3混乱
4.计数频率降低（72MHz/7200=10kHz），手动旋转编码器的低频脉冲可能无法有效触发计数器变化，导致 Speed 始终为0

==============================================================================
*/

//****************************************************************************

/*操作指引
==============================================================================
0.TIM0提供中断
1.PWM输出由TIM2 A0口提供
2.电流驱动模块：A4,A5作为输入
3.编码器A6,A7输入信号. 每两个脉冲之间都会触发计数，得到的就可以近似看作瞬时速度speed
4.输出轴转一圈 * 减速比 = 编码器磁轴转的圈数  磁轴转一圈有11个脉冲 * 4（编码器一个脉冲计数+-4） = 44
5.PA9附用USART的TX,PA10附用USART的RX
6.MotoCr用C8T6测试，MotorD用RCT6调试



===============================================================================
*/

/*工程日志
================================================================================
1.在从未烧录程序时，连上电机驱动模块的PWM输入以及一个输入口（Cin1或Cin2),电机转动，有高低电平加上PWM输入口是默认高电平
在两个输入口（Cin1,Cin2)都接入时电机停转
2.32不和电机驱动共地，编码器会在电机停止时一直发送脉冲
3.编码器模式配置为两个上升沿有效时才能负反馈，一但改为一正一负立刻就变成正反馈（未共地时）

  其实是未共地造成的奇怪影响
  
4.32,STLINK,OLED,tb6612共地才能保证数字电平正常，否则PID失效，OLED崩溃！

5.发现PID控制D电机时只能正转不能反转，经分析后得到PB0引脚对地短路，只能输出低电平，于是用PC9替换PB1

6.中断函数名错误导致电机不转

7.PID调控实践
(1)Ki 调过大会超调，剧烈抖动
(2)增量式PID适合自动与手动控制切换的场景，即去除PID，电机不会停止。
(3)控位时11 * 4 * 35.5 = 
(4)稳态误差：目标自发偏移产生的误差
(5)控速积分饱和：防止电机断电导致积分项累加至深度饱和，进而使电机恢复后满速转动---》积分限幅：大小等于最大输出Out除以Ki
(6)控位加入Ki后积分超调：积分不断累计，当误差为0时累计到最大，积分作用大，造成超调现象---》积分分离:取稍大于稳态误差的值
(7)积分分离升级版->变速积分：设计一个函数值随误差大小增大而减小的函数
(8)微分先行：当Target突变时，微分作用反向且极大(斜率高)--》微分作用系数改为实际值的差值
(9)不完全微分：解决微分项对噪声的作用过于敏感的问题--给微分项
(10)输出值小无法抵消摩擦力等因素导致执行器无动作--》输出偏移跳过执行无动作的阶段
(11)系统在误差极小的情况下不断调控（输出偏移造成），无法稳定--》输出偏移-输入死区 ：不加入I项实现积分分离的效果



==================================================================================
*/


///*编码器测试*/
//int16_t Speed;
//int16_t Location;

//int main(void)
//{
//	OLED_Init();
//	Encoder_Init_TIM2();
//	
//	Timer_Init();
//	
//	while (1)
//	{
//		OLED_Printf(0, 0, OLED_8X16, "Speed:%+05d", Speed);
//		OLED_Printf(0, 16, OLED_8X16, "Location:%+05d", Location);
//		
//		OLED_Update();
//	}
//}

//void TIM7_IRQHandler(void)
//{
//	static uint16_t Count;
//	
//	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)
//	{
//		Count ++;
//		if (Count >= 40)
//		{
//			Count = 0;
//			
//			Speed = Encoder_Get_TIM2();
//			Location += Speed;
//		}
//		
//		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
//	}
//}

  
//int main(void)
//{
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
//	
//	GPIO_SetBits(GPIOC, GPIO_Pin_9);
//	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
//	while (1)
//	{	
//		;
//	}
//}



moctorD_location location = {
	.Target = 370,
	.Kp = 0.25,
	.Ki = 0.25,
	.Kd = 0.2
	
};



int main(void)
{
	OLED_Init();
	Timer_Init();
	Motor_Init_TIM8();
	Encoder_Init_TIM2();
	Serial_Init();


	
	
	OLED_Printf(0, 0, OLED_8X16, "Location Control");	
	OLED_Update();
	
	while (1)
	{

		
		OLED_Printf(0, 16, OLED_8X16, "Tar:%+06.0f", location.Target);
		OLED_Printf(0, 32, OLED_8X16, "Act:%+06.0f", location.Actual);
		OLED_Printf(0, 48, OLED_8X16, "Out:%+06.0f", location.Out);
		
		OLED_Update();
		
		Serial_Printf("%f,%f,%f\r\n", location.Target, location.Actual, location.Out);
	}
	
}

void TIM7_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)
	{
		//Actual += Encoder_Get_TIM2();
		
		//微分先行
		location.Actual1	= location.Actual;
		location.Actual += Encoder_Get_TIM2();
		
		MoctorD_Location(&location);
		
			
		Motor_SetPWM_D(location.Out);
			
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}

