/*
 * TestLED.c
 *
 *  Created on: 2024��10��12��
 *      Author: jacob
 */

#include "debug.h"

//==========================================================
//  �������ƣ�   Test_LED_Init
//  �������ܣ�   ��ʼ������LED
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void Test_LED_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

//==========================================================
//  �������ƣ�   Test_LED_On
//  �������ܣ�  ���Ե���
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void Test_LED_On(void)
{
    GPIO_WriteBit(GPIOE, GPIO_Pin_6, Bit_RESET);
}

//==========================================================
//  �������ƣ�   Test_LED_Off
//  �������ܣ�   ���Ե���
//  ��ڲ�����   ��
//  ���ز�����   ��
//==========================================================
void Test_LED_Off(void)
{
    GPIO_WriteBit(GPIOE, GPIO_Pin_6, Bit_SET);
}
