#include "stm32f10x.h"                  // Device header

void PWM_Init_TIM8(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM8);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;		//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 36 - 1;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;		//CCR
	
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
	
	TIM_CtrlPWMOutputs(TIM8, ENABLE);//高定OC需要设置该函数
	TIM_ARRPreloadConfig(TIM8, ENABLE);//影子寄存器
	TIM_Cmd(TIM8, ENABLE);
	
}

void PWM_SetCompareC(uint16_t Compare)
{
	TIM_SetCompare1(TIM8, Compare);
}

void PWM_SetCompareD(uint16_t Compare)
{
	TIM_SetCompare2(TIM8, Compare);
}

void PWM_SetCompareA(uint16_t Compare)
{
	TIM_SetCompare3(TIM8, Compare);
}
