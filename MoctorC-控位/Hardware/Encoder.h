#ifndef __ENCODER_H
#define __ENCODER_H

void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);
void Encoder_Init_TIM4(void);

int16_t Encoder_Get_TIM2(void);
int16_t Encoder_Get_TIM3(void);
int16_t Encoder_Get_TIM4(void);

#endif
