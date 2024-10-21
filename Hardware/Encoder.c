/*
 * Encoder.c
 *
 *  Created on: 2024年10月15日
 *      Author: jacob
 */

#include "debug.h"

//==========================================================
//  函数名称：   Encoder_Init
//  函数功能：   初始化获取电机转速与转向的编码器
//  入口参数：   无
//  返回参数：   无
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

    // 定时器基本配置
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1; // 最大计数值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 144 - 1; // 标准频率为1MHz
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 不分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    // 配置输入捕获
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 捕获上升沿
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 选择直连通道
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // 不分频
    TIM_ICInitStructure.TIM_ICFilter = 0; // 因为来源是PWM波所以滤波器最小，江科协用的旋转编码器设置为最大
    TIM_PWMIConfig(TIM1, &TIM_ICInitStructure); // 使用PWMI模式测频率和占空比
    TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

    TIM_SelectInputTrigger(TIM1, TIM_TS_TI2FP2);        // 选择从模式触发源为TI1FP1
    TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);        // 选择从模式触发源为TI1FP1
    TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);     // 选择从模式为Reset
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);     // 选择从模式为Reset

    // 使能定时器
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

//==========================================================
//  函数名称：   PWMI1_GetDuty
//  函数功能：   获取电机1的占空比
//  入口参数：   无
//  返回参数：   电机1的占空比
//==========================================================
uint32_t PWMI1_GetDuty(void)
{
    return (100 * TIM_GetCapture2(TIM1) / TIM_GetCapture1(TIM1));
}

uint32_t P1_C1(void) {return TIM_GetCapture1(TIM1);}
uint32_t P1_C2(void) {return TIM_GetCapture2(TIM1);}

//==========================================================
//  函数名称：   PWMI2_GetDuty
//  函数功能：   获取电机2的占空比
//  入口参数：   无
//  返回参数：   电机2的占空比
//==========================================================
uint32_t PWMI2_GetDuty(void)
{
    return (100 * TIM_GetCapture2(TIM2) / TIM_GetCapture1(TIM2));
}
