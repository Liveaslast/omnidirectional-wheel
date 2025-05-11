#include "stm32f10x.h"                  // Device header
#include "PWM.h"



void Motor_Init_TIM8(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	PWM_Init_TIM8();
}

void Motor_SetPWM_D(int8_t PWM)
{
	if (PWM >= 0)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		GPIO_SetBits(GPIOB, GPIO_Pin_1);
		PWM_SetCompareD(PWM);
	}
	else
	{
		GPIO_SetBits(GPIOC, GPIO_Pin_9);
		GPIO_ResetBits(GPIOB, GPIO_Pin_1);
		PWM_SetCompareD(-PWM);
	}
}

void Motor_SetPWM_C(int8_t PWM)
{
	if (PWM >= 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		GPIO_SetBits(GPIOB, GPIO_Pin_13);
		PWM_SetCompareC(PWM);
	}
	else
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);
		PWM_SetCompareC(-PWM);
	}
}

void Motor_SetPWM_A(int8_t PWM)
{
	if (PWM >= 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_14);
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
		PWM_SetCompareA(PWM);
	}
	else
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);
		PWM_SetCompareA(-PWM);
	}
}
