/*
 * Servo.c
 *
 *  Created on: 2024��9��25��
 *      Author: jacob
 */

#include "debug.h"

//==========================================================
//  �������ƣ�   Servo_PWM_Init
//  �������ܣ�   ��ʼ���������PWM
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void Servo_PWM_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // ����APB1����ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // ����APB2����ʱ��

    GPIO_InitTypeDef GPIO_InitStructure; // GPIO��ʼ���ṹ��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // ��������ģʽ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; // ָ������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // �����ٶ�
    GPIO_Init(GPIOA, &GPIO_InitStructure); // GPIO��ʼ��

    TIM_InternalClockConfig(TIM4); // ѡ���ڲ�ʱ��

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; // TimeBaseInit�ṹ��
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // �˲�������Ƶ�ʷ�Ƶ
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // ����ģʽ
    TIM_TimeBaseInitStructure.TIM_Period = 200 - 1; // ���ڣ�ARR�Զ���װ��
    TIM_TimeBaseInitStructure.TIM_Prescaler = 14400 - 1; // PSCԤ��Ƶ��
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0; // �ظ�������
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure); // ʱ����Ԫ��ʼ��

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
    TIM_OC3Init(TIM4, &TIM_OCInitStructure); // ͨ����ʼ��

    TIM_Cmd(TIM4, ENABLE); // ������ʱ��

}

//==========================================================
//  �������ƣ�   Servo_PWM_SetCompare
//  �������ܣ�   ���ö����PWM��ռ�ձ�
//  ��ڲ�����   CCR��ֵ
//  ���ز�����   ��
//==========================================================
void Servo_PWM_SetCompare(uint16_t Compare)
{
    TIM_SetCompare3(TIM4, Compare); // ����CCR��ֵ����������ռ�ձ�
}

//==========================================================
//  �������ƣ�   Servo_SetAngle
//  �������ܣ�   ���ö��ת��Ƕ�
//  ��ڲ�����   ���ת��Ƕȣ�ȡֵ0~180
//  ���ز�����   ��
//==========================================================
void Servo_SetAngle(float Angle)
{
    Servo_PWM_SetCompare(Angle / 180 * 20 + 5); // ����Ƕ�Ϊ0~180��
}