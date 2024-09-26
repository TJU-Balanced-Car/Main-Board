/*
 * Buzzer.c
 *
 *  Created on: 2024年9月26日
 *      Author: mdfucker
 */

#include "debug.h"

//==========================================================
//  函数名称：   Buzzer_Init
//  函数功能：   初始化蜂鸣器
//  入口参数：   无
//  返回参数：   无
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
//  函数名称：   Buzzer_Ring
//  函数功能：   蜂鸣器响
//  入口参数：   无
//  返回参数：   无
//==========================================================
void Buzzer_Ring(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_11, Bit_SET);
}

//==========================================================
//  函数名称：   Buzzer_Stop
//  函数功能：   蜂鸣器停响
//  入口参数：   无
//  返回参数：   无
//==========================================================
void Buzzer_Stop(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_11, Bit_RESET);
}

