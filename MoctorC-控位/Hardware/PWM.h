#ifndef __PWM_H
#define __PWM_H

void PWM_Init_TIM8(void);
void PWM_SetCompareC(uint16_t Compare);
void PWM_SetCompareD(uint16_t Compare);
void PWM_SetCompareA(uint16_t Compare);

#endif
