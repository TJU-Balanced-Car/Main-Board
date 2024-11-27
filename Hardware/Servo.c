/*
 * Servo.c
 *
 *  Created on: 2024年9月25日
 *      Author: jacob
 */

#include "debug.h"

//==========================================================
//  函数名称：   Servo_PWM_Init
//  函数功能：   初始化舵机所用PWM
//  入口参数：   无
//  返回参数：   无
//==========================================================
void Servo_PWM_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // 开启APB1外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // 开启APB2外设时钟

    GPIO_InitTypeDef GPIO_InitStructure; // GPIO初始化结构体
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 引脚运行模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; // 指定引脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 引脚速度
    GPIO_Init(GPIOB, &GPIO_InitStructure); // GPIO初始化

    TIM_InternalClockConfig(TIM4); // 选择内部时钟

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; // TimeBaseInit结构体
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 滤波器采样频率分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 计数模式
    TIM_TimeBaseInitStructure.TIM_Period = 200 - 1; // 周期，ARR自动重装器
    TIM_TimeBaseInitStructure.TIM_Prescaler = 14400 - 1; // PSC预分频器
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0; // 重复计数器
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure); // 时基单元初始化

    TIM_OCInitTypeDef TIM_OCInitStructure; // 结构体
    TIM_OCStructInit(&TIM_OCInitStructure); // 先给结构体赋初始值，防止出问题
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // 输出比较模式
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 输出比较极性
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出比较使能
    TIM_OCInitStructure.TIM_Pulse = 0; // 设置CCR
                                        // **关于ARR，PSC，CCR的计算**
                                        // PWM频率：Freq = CK_PSC / (PSC + 1) / (ARR + 1)
                                        // *PWM占空比：Duty = CCR / (ARR + 1)
                                        // PWM分辨率：Reso = 1 / (ARR + 1)
    TIM_OC3Init(TIM4, &TIM_OCInitStructure); // 通道初始化

    TIM_OC3PreloadConfig(TIM4, ENABLE);

    TIM_Cmd(TIM4, ENABLE); // 启动定时器

}

//==========================================================
//  函数名称：   Servo_PWM_SetCompare
//  函数功能：   设置舵机的PWM波占空比
//  入口参数：   CCR的值，值取5~25（对应2.5%~12.5%）
//  返回参数：   无
//==========================================================
void Servo_PWM_SetCompare(uint16_t Compare)
{
    TIM_SetCompare3(TIM4, Compare); // 设置CCR的值，用于设置占空比
}

//==========================================================
//  函数名称：   Servo_SetAngle
//  函数功能：   设置舵机转向角度
//  入口参数：   舵机转向角度，取值0~180
//  返回参数：   无
//==========================================================
void Servo_SetAngle(float Angle)
{
    Servo_PWM_SetCompare((Angle +  110) / 180 * 20 + 5); // 舵机角度为0~180度
}
