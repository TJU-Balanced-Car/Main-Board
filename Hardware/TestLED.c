/*
 * TestLED.c
 *
 *  Created on: 2024年10月12日
 *      Author: jacob
 */

#include "debug.h"

//==========================================================
//  函数名称：   Test_LED_Init
//  函数功能：   初始化测试LED
//  入口参数：   无
//  返回参数：   无
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
//  函数名称：   Test_LED_On
//  函数功能：  测试灯亮
//  入口参数：   无
//  返回参数：   无
//==========================================================
void Test_LED_On(void)
{
    GPIO_WriteBit(GPIOE, GPIO_Pin_6, Bit_RESET);
}

//==========================================================
//  函数名称：   Test_LED_Off
//  函数功能：   测试灯灭
//  入口参数：   无
//  返回参数：   无
//==========================================================
void Test_LED_Off(void)
{
    GPIO_WriteBit(GPIOE, GPIO_Pin_6, Bit_SET);
}
