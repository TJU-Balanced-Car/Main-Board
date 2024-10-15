/*
 * Encoder.c
 *
 *  Created on: 2024年10月15日
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
//  函数名称：   TIM1_IRQHandler
//  函数功能：   TIM1定时器的中断函数
//  入口参数：   无
//  返回参数：   无
//==========================================================
void TIM1_IRQHandler(void)
{
    // 检查是否发生了捕获事件
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET)
    {
        // 清除中断标志
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);

        // 捕获第一个上升沿
        if (TIM1_capture_value_1 == 0)
        {
            TIM1_capture_value_1 = TIM_GetCapture1(TIM1);
        }
        // 捕获第二个上升沿，计算周期
        else
        {
            TIM1_capture_value_2 = TIM_GetCapture1(TIM1);

            // 计算周期差
            if (TIM1_capture_value_2 > TIM1_capture_value_1)
            {
                TIM1_capture_diff = TIM1_capture_value_2 - TIM1_capture_value_1;
            }
            else
            {
                // 处理定时器溢出的情况
                TIM1_capture_diff = (0xFFFF - TIM1_capture_value_1) + TIM1_capture_value_2;
            }

            // 计算频率
            if (TIM1_capture_diff != 0)
            {
                TIM1_pwm_frequency = 144000000 / TIM1_capture_diff;
            }
            else
            {
                TIM1_pwm_frequency = 0; // 或者其他合适的处理
            }


            // 将频率转换为转速 (RPM = 频率 * 60 / 脉冲数)
            TIM1_rpm = (TIM1_pwm_frequency * 60) / 6; // 脉冲数存疑，三相电机是6？

            // 读取方向引脚电平
            TIM1_direction = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11);

            // 根据方向调整RPM的符号：1 为正转，0 为反转
            if (TIM1_direction == 0)
            {
                TIM1_rpm = -TIM1_rpm;  // 如果反向，转速为负值
            }

            // 重置捕获值
            TIM1_capture_value_1 = 0;
        }
    }
}

//==========================================================
//  函数名称：   TIM2_IRQHandler
//  函数功能：   TIM2定时器的中断函数
//  入口参数：   无
//  返回参数：   无
//==========================================================
void TIM2_IRQHandler(void)
{
    // 检查是否发生了捕获事件
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET)
    {
        // 清除中断标志
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

        // 捕获第一个上升沿
        if (TIM2_capture_value_1 == 0)
        {
            TIM2_capture_value_1 = TIM_GetCapture1(TIM2);
        }
        // 捕获第二个上升沿，计算周期
        else
        {
            TIM2_capture_value_2 = TIM_GetCapture1(TIM2);

            // 计算周期差
            if (TIM2_capture_value_2 > TIM2_capture_value_1)
            {
                TIM2_capture_diff = TIM2_capture_value_2 - TIM2_capture_value_1;
            }
            else
            {
                // 处理定时器溢出的情况
                TIM2_capture_diff = (0xFFFF - TIM2_capture_value_1) + TIM2_capture_value_2;
            }

            // 计算频率
            if (TIM2_capture_diff != 0) {
                TIM2_pwm_frequency = 144000000 / TIM2_capture_diff;
            } else {
                TIM2_pwm_frequency = 0; // 或者其他合适的处理
            }


            // 将频率转换为转速 (RPM = 频率 * 60 / 脉冲数)
            TIM2_rpm = (TIM2_pwm_frequency * 60) / 6; // 脉冲数存疑，三相电机是6？

            // 读取方向引脚电平
            TIM2_direction = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);

            // 根据方向调整RPM的符号：1 为正转，0 为反转
            if (TIM2_direction == 0)
            {
                TIM2_rpm = -TIM2_rpm;  // 如果反向，转速为负值
            }

            // 重置捕获值
            TIM2_capture_value_1 = 0;
        }
    }
}

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

    // TIM_InternalClockConfig(TIM1);  似乎读PWM不需要手动设置内部时钟

    // 定时器基本配置
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1; // 最大计数值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;   // 不分频
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    // 配置输入捕获
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 捕获上升沿
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0; // 因为来源是PWM波所以滤波器最小，江科协用的旋转编码器设置为最大
    TIM_ICInit(TIM1, &TIM_ICInitStructure);
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    // 开启定时器捕获中断
    TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE); // TIM_IT_CC1 or TIM_IT_Update
    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

    // 使能定时器
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    // 配置定时器中断优先级
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn; // ?对嘛
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}
