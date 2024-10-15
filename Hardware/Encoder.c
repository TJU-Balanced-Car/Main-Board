/*
 * Encoder.c
 *
 *  Created on: 2024��10��15��
 *      Author: jacob
 */

#include "debug.h"

volatile uint32_t TIM1_capture_value_1 = 0;
volatile uint32_t TIM1_capture_value_2 = 0;
volatile uint32_t TIM1_capture_diff = 0;
volatile uint32_t TIM1_pwm_frequency = 0;
extern volatile uint32_t TIM1_rpm = 0;
extern volatile uint8_t TIM1_direction = 0;
volatile uint32_t TIM2_capture_value_1 = 0;
volatile uint32_t TIM2_capture_value_2 = 0;
volatile uint32_t TIM2_capture_diff = 0;
volatile uint32_t TIM2_pwm_frequency = 0;
extern volatile uint32_t TIM2_rpm = 0;
extern volatile uint8_t TIM2_direction = 0;

//==========================================================
//  �������ƣ�   TIM1_IRQHandler
//  �������ܣ�   TIM1��ʱ�����жϺ���
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void TIM1_IRQHandler(void)
{
    // ����Ƿ����˲����¼�
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET)
    {
        // ����жϱ�־
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);

        // �����һ��������
        if (TIM1_capture_value_1 == 0)
        {
            TIM1_capture_value_1 = TIM_GetCapture1(TIM1);
        }
        // ����ڶ��������أ���������
        else
        {
            TIM1_capture_value_2 = TIM_GetCapture1(TIM1);

            // �������ڲ�
            if (TIM1_capture_value_2 > TIM1_capture_value_1)
            {
                TIM1_capture_diff = TIM1_capture_value_2 - TIM1_capture_value_1;
            }
            else
            {
                // ����ʱ����������
                TIM1_capture_diff = (0xFFFF - TIM1_capture_value_1) + TIM1_capture_value_2;
            }

            // ����Ƶ��
            if (TIM1_capture_diff != 0)
            {
                TIM1_pwm_frequency = 144000000 / TIM1_capture_diff;
            }
            else
            {
                TIM1_pwm_frequency = 0; // �����������ʵĴ���
            }


            // ��Ƶ��ת��Ϊת�� (RPM = Ƶ�� * 60 / ������)
            TIM1_rpm = (TIM1_pwm_frequency * 60) / 6; // ���������ɣ���������6��

            // ��ȡ�������ŵ�ƽ
            TIM1_direction = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11);

            // ���ݷ������RPM�ķ��ţ�1 Ϊ��ת��0 Ϊ��ת
            if (TIM1_direction == 0)
            {
                TIM1_rpm = -TIM1_rpm;  // �������ת��Ϊ��ֵ
            }

            // ���ò���ֵ
            TIM1_capture_value_1 = 0;
        }
    }
}

//==========================================================
//  �������ƣ�   TIM2_IRQHandler
//  �������ܣ�   TIM2��ʱ�����жϺ���
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void TIM2_IRQHandler(void)
{
    // ����Ƿ����˲����¼�
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET)
    {
        // ����жϱ�־
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

        // �����һ��������
        if (TIM2_capture_value_1 == 0)
        {
            TIM2_capture_value_1 = TIM_GetCapture1(TIM2);
        }
        // ����ڶ��������أ���������
        else
        {
            TIM2_capture_value_2 = TIM_GetCapture1(TIM2);

            // �������ڲ�
            if (TIM2_capture_value_2 > TIM2_capture_value_1)
            {
                TIM2_capture_diff = TIM2_capture_value_2 - TIM2_capture_value_1;
            }
            else
            {
                // ����ʱ����������
                TIM2_capture_diff = (0xFFFF - TIM2_capture_value_1) + TIM2_capture_value_2;
            }

            // ����Ƶ��
            if (TIM2_capture_diff != 0) {
                TIM2_pwm_frequency = 144000000 / TIM2_capture_diff;
            } else {
                TIM2_pwm_frequency = 0; // �����������ʵĴ���
            }


            // ��Ƶ��ת��Ϊת�� (RPM = Ƶ�� * 60 / ������)
            TIM2_rpm = (TIM2_pwm_frequency * 60) / 6; // ���������ɣ���������6��

            // ��ȡ�������ŵ�ƽ
            TIM2_direction = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);

            // ���ݷ������RPM�ķ��ţ�1 Ϊ��ת��0 Ϊ��ת
            if (TIM2_direction == 0)
            {
                TIM2_rpm = -TIM2_rpm;  // �������ת��Ϊ��ֵ
            }

            // ���ò���ֵ
            TIM2_capture_value_1 = 0;
        }
    }
}

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

    // TIM_InternalClockConfig(TIM1);  �ƺ���PWM����Ҫ�ֶ������ڲ�ʱ��

    // ��ʱ����������
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1; // ������ֵ
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;   // ����Ƶ
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    // �������벶��
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // ����������
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0; // ��Ϊ��Դ��PWM�������˲�����С������Э�õ���ת����������Ϊ���
    TIM_ICInit(TIM1, &TIM_ICInitStructure);
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    // ������ʱ�������ж�
    TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE); // TIM_IT_CC1 or TIM_IT_Update
    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

    // ʹ�ܶ�ʱ��
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    // ���ö�ʱ���ж����ȼ�
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn; // ?����
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}
