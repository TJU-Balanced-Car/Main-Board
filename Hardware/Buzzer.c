/*
 * Buzzer.c
 *
 *  Created on: 2024��9��26��
 *      Author: mdfucker
 */

#include "debug.h"

//==========================================================
//  �������ƣ�   Buzzer_Init
//  �������ܣ�   ��ʼ��������
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void Buzzer_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

//==========================================================
//  �������ƣ�   Buzzer_Ring
//  �������ܣ�   ��������
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void Buzzer_Ring(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_11, Bit_SET);
}

//==========================================================
//  �������ƣ�   Buzzer_Stop
//  �������ܣ�   ������ͣ��
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void Buzzer_Stop(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_11, Bit_RESET);
}

