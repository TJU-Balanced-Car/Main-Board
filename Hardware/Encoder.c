/*
 * Encoder.c
 *
 *  Created on: 2024��10��15��
 *      Author: jacob
 */

#include "debug.h"

//==========================================================
//  �������ƣ�   Encoder_Init
//  �������ܣ�   ��ʼ����ȡ���ת����ת��ı�����
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void Encoder_Init(void)
{
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
    TIM_PWMIConfig(TIM1, &TIM_ICInitStructure); // ʹ��PWMIģʽ��Ƶ�ʺ�ռ�ձ�
    TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

    TIM_SelectInputTrigger(TIM1, TIM_TS_TI2FP2);        // ѡ���ģʽ����ԴΪTI1FP1
    TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);        // ѡ���ģʽ����ԴΪTI1FP1
    TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);     // ѡ���ģʽΪReset
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);     // ѡ���ģʽΪReset

    // ʹ�ܶ�ʱ��
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

//==========================================================
//  �������ƣ�   PWMI1_GetDuty
//  �������ܣ�   ��ȡ���1��ռ�ձ�
//  ��ڲ�����   ��
//  ���ز�����   ���1��ռ�ձ�
//==========================================================
uint32_t PWMI1_GetDuty(void)
{
    return (100 * TIM_GetCapture2(TIM1) / TIM_GetCapture1(TIM1));
}

uint32_t P1_C1(void) {return TIM_GetCapture1(TIM1);}
uint32_t P1_C2(void) {return TIM_GetCapture2(TIM1);}

//==========================================================
//  �������ƣ�   PWMI2_GetDuty
//  �������ܣ�   ��ȡ���2��ռ�ձ�
//  ��ڲ�����   ��
//  ���ز�����   ���2��ռ�ձ�
//==========================================================
uint32_t PWMI2_GetDuty(void)
{
    return (100 * TIM_GetCapture2(TIM2) / TIM_GetCapture1(TIM2));
}
