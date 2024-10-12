/*
 * Motor.c
 *
 *  Created on: 2024��10��11��
 *      Author: mdfucker
 */

#include "debug.h"

//==========================================================
//  �������ƣ�   Motor_PWM_Init
//  �������ܣ�   ��ʼ���������
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void Motor_PWM_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE); // ����APB2����ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure; // GPIO��ʼ���ṹ��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // ��������ģʽ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; // ָ������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // �����ٶ�
    GPIO_Init(GPIOC, &GPIO_InitStructure); // GPIO��ʼ��

    TIM_InternalClockConfig(TIM8); // ѡ���ڲ�ʱ��

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; // TimeBaseInit�ṹ��
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // �˲�������Ƶ�ʷ�Ƶ
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // ����ģʽ
    TIM_TimeBaseInitStructure.TIM_Period = 200 - 1; // ���ڣ�ARR�Զ���װ��
    TIM_TimeBaseInitStructure.TIM_Prescaler = 14400 - 1; // PSCԤ��Ƶ��
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0; // �ظ�������
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure); // ʱ����Ԫ��ʼ��

    TIM_OCInitTypeDef TIM_OCInitStructure; // �ṹ��
    TIM_OCStructInit(&TIM_OCInitStructure); // �ȸ��ṹ�帳��ʼֵ����ֹ������
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // ����Ƚ�ģʽ
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High; // ����Ƚϼ���
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; // ����Ƚ�ʹ��
    TIM_OCInitStructure.TIM_Pulse = 0; // ����CCR
                                        // **����ARR��PSC��CCR�ļ���**
                                        // PWMƵ�ʣ�Freq = CK_PSC / (PSC + 1) / (ARR + 1)
                                        // *PWMռ�ձȣ�Duty = CCR / (ARR + 1)
                                        // PWM�ֱ��ʣ�Reso = 1 / (ARR + 1)
    TIM_OC1Init(TIM8, &TIM_OCInitStructure); // ͨ����ʼ��
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);

    TIM_Cmd(TIM8, ENABLE); // ������ʱ��

}

//==========================================================
//  �������ƣ�   Motor1_SetSpeed
//  �������ܣ�   ���õ��1ת��
//  ��ڲ�����   ���ת�٣�ȡֵ0~100
//  ���ز�����   ��
//==========================================================
void Motor1_SetSpeed(float Speed)
{
    TIM_SetCompare1(TIM8, Speed * 2); // ���ת��Ϊ0~100
}

//==========================================================
//  �������ƣ�   Motor2_SetSpeed
//  �������ܣ�   ���õ��2ת��
//  ��ڲ�����   ���ת�٣�ȡֵ0~100
//  ���ز�����   ��
//==========================================================
void Motor2_SetSpeed(float Speed)
{
    TIM_SetCompare3(TIM8, Speed * 2); // ���ת��Ϊ0~100
}
