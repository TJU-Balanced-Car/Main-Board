/*
 * Encoder.c
 *
 *  Created on: 2024年10月15日
 *      Author: jacob
 */

#include "debug.h"

//volatile uint32_t TIM1_capture_value_1 = 0;
//volatile uint32_t TIM1_capture_value_2 = 0;
//volatile uint32_t TIM1_capture_diff = 0;
//volatile uint32_t TIM1_pwm_frequency = 0;
//volatile uint32_t TIM1_tmp_rpm = 0;
//volatile uint32_t TIM1_rpm = 0;
volatile uint32_t riseTime1 = 0;
volatile uint32_t lastCapture1 = 0;
volatile uint32_t Encoder1_HighTime = 0;
volatile uint8_t Encoder1_Direction = 0;
//volatile uint32_t TIM2_capture_value_1 = 0;
//volatile uint32_t TIM2_capture_value_2 = 0;
//volatile uint32_t TIM2_capture_diff = 0;
//volatile uint32_t TIM2_pwm_frequency = 0;
//volatile uint32_t TIM2_tmp_rpm = 0;
//volatile uint32_t TIM2_rpm = 0;
volatile uint32_t riseTime2 = 0;
volatile uint32_t lastCapture2 = 0;
volatile uint32_t Encoder2_HighTime = 0;
volatile uint8_t Encoder2_Direction = 0;


//==========================================================
//  函数名称：   Encoder1_Get_HighTime
//  函数功能：   获取电机1的高电平时长
//  入口参数：   无
//  返回参数：   电机1的高电平时长
//==========================================================
uint32_t Encoder1_Get_HighTime(void)
{
    return Encoder1_HighTime;
}

//==========================================================
//  函数名称：   Encoder1_Get_Dir
//  函数功能：   获取电机1的转向
//  入口参数：   无
//  返回参数：   电机1的转向
//==========================================================
uint8_t Encoder1_Get_Dir(void)
{
    return Encoder1_Direction;
}

//==========================================================
//  函数名称：   Encoder2_Get_HighTime
//  函数功能：   获取电机2的高电平时长
//  入口参数：   无
//  返回参数：   电机2的高电平时长
//==========================================================
uint32_t Encoder2_Get_HighTime(void)
{
    return Encoder2_HighTime;
}

//==========================================================
//  函数名称：   Encoder2_Get_Dir
//  函数功能：   获取电机2的转向
//  入口参数：   无
//  返回参数：   电机2的转向
//==========================================================
uint8_t Encoder2_Get_Dir(void)
{
    return Encoder2_Direction;
}

//==========================================================
//  函数名称：   TIM1_IRQHandler
//  函数功能：   TIM1定时器的中断函数
//  入口参数：   无
//  返回参数：   无
//==========================================================
void TIM1_IRQHandler(void)
{
    printf("T1!\n");
    if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)
    {
        printf("Run into T1IRQ\n");
        uint32_t currentTime = TIM_GetCapture2(TIM1);
        if (lastCapture1 == 0 || currentTime > lastCapture1)
        {
            riseTime1 = currentTime; // 记录上升沿
        }
        else
        {
            // 记录下降沿时的时间
            uint32_t fallTime1 = currentTime;
            Encoder1_HighTime = fallTime1 - riseTime1;
        }
        lastCapture1 = currentTime;

        // 读取方向引脚电平
        Encoder1_Direction = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9);
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
        printf("Run out T1IRQ\n");
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
    printf("T2!\n");
    if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
    {
        printf("Run into T2IRQ\n");
        uint32_t currentTime = TIM_GetCapture2(TIM2);
        if (lastCapture2 == 0 || currentTime > lastCapture2)
        {
            riseTime2 = currentTime; // 记录上升沿
        }
        else
        {
            // 记录下降沿时的时间
            uint32_t fallTime2 = currentTime;
            Encoder2_HighTime = fallTime2 - riseTime2;
        }
        lastCapture2 = currentTime;

        // 读取方向引脚电平
        Encoder2_Direction = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
        TIM_SetCounter(TIM2, 0);
        printf("Run out T2IRQ\n");
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

    TIM_InternalClockConfig(TIM1);  //似乎读PWM不需要手动设置内部时钟
    TIM_InternalClockConfig(TIM2);

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
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 捕获上升沿
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0; // 因为来源是PWM波所以滤波器最小，江科协用的旋转编码器设置为最大
    TIM_ICInit(TIM1, &TIM_ICInitStructure);
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    // 开启定时器捕获中断
    TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

    // 使能定时器
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    // 配置定时器中断优先级
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn; // ?对嘛
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}
