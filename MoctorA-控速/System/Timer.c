#include "stm32f10x.h"                  // Device header

void Timer_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	
	TIM_InternalClockConfig(TIM7);//选择要配置TIM时钟
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//初始化时基单元
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//计数器模式：上升计数
	TIM_TimeBaseInitStructure.TIM_Period = 400 - 1;//
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;//设定预分频值，72M/7200 = 10000HZ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;//重复计数(0 + 1)个周期产生冲断
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);//BaseInit会自动跳到计数器的下一位，也就是1，需要复位为0
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);//开启时钟中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//配置NVIC分组
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;//选择NVIC中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//响应优先
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM7, ENABLE);//配置好NVIC后再使能TIM
}


//void TIM7_IRQHandler(void)
//{
//	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)
//	{
//		
//		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
//	}
//}


