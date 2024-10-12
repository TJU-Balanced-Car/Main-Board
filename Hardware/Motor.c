/*
 * Motor.c
 *
 *  Created on: 2024年10月11日
 *      Author: mdfucker
 */

#include "debug.h"

//==========================================================
//  函数名称：   Motor_PWM_Init
//  函数功能：   初始化两个电机
//  入口参数：   无
//  返回参数：   无
//==========================================================
void Motor_PWM_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE); // 开启APB2外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure; // GPIO初始化结构体
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 引脚运行模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; // 指定引脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 引脚速度
    GPIO_Init(GPIOC, &GPIO_InitStructure); // GPIO初始化

    TIM_InternalClockConfig(TIM8); // 选择内部时钟

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; // TimeBaseInit结构体
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 滤波器采样频率分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 计数模式
    TIM_TimeBaseInitStructure.TIM_Period = 200 - 1; // 周期，ARR自动重装器
    TIM_TimeBaseInitStructure.TIM_Prescaler = 14400 - 1; // PSC预分频器
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0; // 重复计数器
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure); // 时基单元初始化

    TIM_OCInitTypeDef TIM_OCInitStructure; // 结构体
    TIM_OCStructInit(&TIM_OCInitStructure); // 先给结构体赋初始值，防止出问题
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // 输出比较模式
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High; // 输出比较极性
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; // 输出比较使能
    TIM_OCInitStructure.TIM_Pulse = 0; // 设置CCR
                                        // **关于ARR，PSC，CCR的计算**
                                        // PWM频率：Freq = CK_PSC / (PSC + 1) / (ARR + 1)
                                        // *PWM占空比：Duty = CCR / (ARR + 1)
                                        // PWM分辨率：Reso = 1 / (ARR + 1)
    TIM_OC1Init(TIM8, &TIM_OCInitStructure); // 通道初始化
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);

    TIM_Cmd(TIM8, ENABLE); // 启动定时器

}

//==========================================================
//  函数名称：   Motor1_SetSpeed
//  函数功能：   设置电机1转速
//  入口参数：   电机转速，取值0~100
//  返回参数：   无
//==========================================================
void Motor1_SetSpeed(float Speed)
{
    TIM_SetCompare1(TIM8, Speed * 2); // 电机转速为0~100
}

//==========================================================
//  函数名称：   Motor2_SetSpeed
//  函数功能：   设置电机2转速
//  入口参数：   电机转速，取值0~100
//  返回参数：   无
//==========================================================
void Motor2_SetSpeed(float Speed)
{
    TIM_SetCompare3(TIM8, Speed * 2); // 电机转速为0~100
}
