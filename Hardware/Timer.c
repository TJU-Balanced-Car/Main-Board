/*
 * Timer.c
 *
 *  Created on: 2024年11月8日
 *      Author: jacob
 */

#include "debug.h"

//==========================================================
//  函数名称：   Timer_IC_Init
//  函数功能：   定时器中断初始化函数
//  入口参数：   无
//  返回参数：   无
//==========================================================
void Timer_IC_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    TIM_InternalClockConfig(TIM5);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1; // 10ms为周期
    TIM_TimeBaseInitStructure.TIM_Prescaler = 14400 - 1; // 10kHz -> 0.1ms计一次
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); // 使能更新中断
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStruct);
    TIM_Cmd(TIM5, ENABLE); // 启动定时器
}
