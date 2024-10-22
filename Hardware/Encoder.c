/*
 * Encoder.c
 *
 *  Created on: 2024��10��15��
 *      Author: jacob
 */

#include "debug.h"

volatile int32_t Motor1_is_there_speed = 1; // ��־λ��ָʾ�ٶ��Ƿ�Ϊ��
volatile int32_t Motor2_is_there_speed = 1; // ��־λ��ָʾ�ٶ��Ƿ�Ϊ��
volatile int8_t Motor1_lastCapture = 1; // ��־λ��ָʾ�Ƿ���²���
volatile int8_t Motor2_lastCapture = 1; // ��־λ��ָʾ�Ƿ���²���


void TIM5_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    TIM_InternalClockConfig(TIM5);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1; // 10ms
    TIM_TimeBaseInitStructure.TIM_Prescaler = 14400 - 1; // 10kHz
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



//==========================================================
//  �������ƣ�   Encoder_Init
//  �������ܣ�   ��ʼ����ȡ���ת����ת��ı�����
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void Encoder_Init(void)
{
    TIM5_Init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ��ʱ����������
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1; // ������ֵ
    TIM_TimeBaseInitStructure.TIM_Prescaler = 144 - 1; // ��׼Ƶ��Ϊ1MHz
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // ����Ƶ
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ���ģʽ
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    // �������벶��
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // ����������
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // ѡ��ֱ��ͨ��
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // ����Ƶ
    TIM_ICInitStructure.TIM_ICFilter = 0; // ��Ϊ��Դ��PWM�������˲�����С������Э�õ���ת����������Ϊ���
    TIM_ICInit(TIM1, &TIM_ICInitStructure);
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_SelectInputTrigger(TIM1, TIM_TS_TI2FP2);        // ѡ���ģʽ����ԴΪTI1FP1
    TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);        // ѡ���ģʽ����ԴΪTI1FP1
    TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);     // ѡ���ģʽΪReset
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);     // ѡ���ģʽΪReset

    // ʹ�ܶ�ʱ��
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

//==========================================================
//  �������ƣ�   Motor1_GetDir
//  �������ܣ�   ��ȡ���1�ķ���
//  ��ڲ�����   ��
//  ���ز�����   ���1�ķ�����תΪ0����תΪ-1
//==========================================================
uint32_t Motor1_GetDir(void)
{
    return GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9);
}

//==========================================================
//  �������ƣ�   Motor2_GetDir
//  �������ܣ�   ��ȡ���2�ķ���
//  ��ڲ�����   ��
//  ���ز�����   ���2�ķ�����תΪ0����תΪ-1
//==========================================================
uint32_t Motor2_GetDir(void)
{
    return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
}

//==========================================================
//  �������ƣ�   Motor1_GetFreq
//  �������ܣ�   ��ȡ���1��Ƶ��
//  ��ڲ�����   ��
//  ���ز�����   ���1��Ƶ�ʣ���תΪ������תΪ��
//==========================================================
int32_t Motor1_GetFreq(void)
{
    return (Motor1_GetDir() == 1) ? Motor1_is_there_speed * 1000000 / TIM_GetCapture2(TIM1): Motor1_is_there_speed * (-1) * 1000000 / TIM_GetCapture2(TIM1);
}

//==========================================================
//  �������ƣ�   Motor2_GetFreq
//  �������ܣ�   ��ȡ���2��Ƶ��
//  ��ڲ�����   ��
//  ���ز�����   ���2��Ƶ�ʣ���תΪ������תΪ��
//==========================================================
int32_t Motor2_GetFreq(void)
{
    return (Motor2_GetDir() == 1) ? Motor2_is_there_speed * 1000000 / TIM_GetCapture2(TIM2) : Motor2_is_there_speed * (-1) * 1000000 / TIM_GetCapture2(TIM2);
}
