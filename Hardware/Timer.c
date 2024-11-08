/*
 * Timer.c
 *
 *  Created on: 2024��11��8��
 *      Author: jacob
 */

#include "debug.h"

//==========================================================
//  �������ƣ�   Timer_IC_Init
//  �������ܣ�   ��ʱ���жϳ�ʼ������
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void Timer_IC_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    TIM_InternalClockConfig(TIM5);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1; // 10msΪ����
    TIM_TimeBaseInitStructure.TIM_Prescaler = 14400 - 1; // 10kHz -> 0.1ms��һ��
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); // ʹ�ܸ����ж�
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStruct);
    TIM_Cmd(TIM5, ENABLE); // ������ʱ��
}
