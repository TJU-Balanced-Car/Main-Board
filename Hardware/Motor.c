/*
 * Motor.c
 *
 *  Created on: 2024年10月11日
 *      Author: mdfucker
 */

#include "debug.h"
#include "Buzzer.h"
#include "Motor.h"

#define  PWM_MAX    6000
#define  PWM_MIN   -6000

extern int32_t Motor1_is_there_speed; // 标志位，指示速度是否为零
extern int32_t Motor2_is_there_speed; // 标志位，指示速度是否为零

//==========================================================
//  函数名称：   Motor_Init
//  函数功能：   初始化两个电机
//  入口参数：   无
//  返回参数：   无
//==========================================================
void Motor_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // 开启APB2外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure; // GPIO初始化结构体
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 引脚运行模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8; // 指定引脚
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 引脚速度
    GPIO_Init(GPIOC, &GPIO_InitStructure); // GPIO初始化
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // 引脚运行模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_9; // 指定引脚
    GPIO_Init(GPIOC, &GPIO_InitStructure); // GPIO初始化

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
//    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);  // 部分重映射无法输出PWM波形

    TIM_InternalClockConfig(TIM3); // 选择内部时钟

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; // TimeBaseInit结构体
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 滤波器采样频率分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 计数模式
    TIM_TimeBaseInitStructure.TIM_Period = 7200 - 1; // 周期，ARR自动重装器
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1; // PSC预分频器
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0; // 重复计数器
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure); // 时基单元初始化

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
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // 通道初始化
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM3, ENABLE);
    TIM_OC3PreloadConfig(TIM3, ENABLE);

    TIM_Cmd(TIM3, ENABLE); // 启动定时器

}

//==========================================================
//  函数名称：   Motor_Encoder_Init
//  函数功能：   初始化获取电机转速与转向的编码器
//  入口参数：   无
//  返回参数：   无
//==========================================================
void Motor_Encoder_Init(void)
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
    TIM_ICInit(TIM1, &TIM_ICInitStructure);
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_SelectInputTrigger(TIM1, TIM_TS_TI2FP2);        // 选择从模式触发源为TI1FP1
    TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);        // 选择从模式触发源为TI1FP1
    TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);     // 选择从模式为Reset
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);     // 选择从模式为Reset

    // 使能定时器
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

//==========================================================
//  函数名称：   Motor1_SetSpeed
//  函数功能：   设置电机1转速
//  入口参数：   电机转速
//  返回参数：   无
//==========================================================
void Motor1_SetSpeed(float Speed)
{
    if (Speed >= 0) Motor1_SetDir(1);
    else if (Speed < 0)
    {
        Motor1_SetDir(0);
        Speed = -Speed;
    }
    TIM_SetCompare3(TIM3, Speed); // 保持在0到100范围内
}

//==========================================================
//  函数名称：   Motor1_SetDir
//  函数功能：   设置电机1转向
//  入口参数：   转动方向，正向为1，反向为0
//  返回参数：   无
//==========================================================
void Motor1_SetDir(uint8_t Dir)
{
    BitAction Bit;
    Bit = Bit_SET;
    if (Dir == 1) Bit = Bit_SET;
    else if (Dir == 0) Bit = Bit_RESET;
    GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit);
}

//==========================================================
//  函数名称：   Motor2_SetSpeed
//  函数功能：   设置电机2转速
//  入口参数：   电机转速
//  返回参数：   无
//==========================================================
void Motor2_SetSpeed(float Speed)
{
    TIM_SetCompare1(TIM3, Speed); // 电机转速为0~100
}

//==========================================================
//  函数名称：   Motor2_SetDir
//  函数功能：   设置电机2转向
//  入口参数：   转动方向，正向为1，反向为0
//  返回参数：   无
//==========================================================
void Motor2_SetDir(uint8_t Dir)
{
    BitAction Bit;
    if (Dir == 1) Bit = Bit_SET;
    else Bit = Bit_RESET;
    GPIO_WriteBit(GPIOC, GPIO_Pin_7, Bit);
}

//==========================================================
//  函数名称：   PWM_Limit
//  函数功能：   PWM限幅
//  入口参数：   pwm变量地址
//  返回参数：   无
//==========================================================
void PWM_Limit(int *pwm)
{
    if(*pwm > PWM_MAX) *pwm = PWM_MAX;
    else if(*pwm <= PWM_MIN) *pwm = PWM_MIN;
}

//==========================================================
//  函数名称：   Motor_Stop
//  函数功能：   小车倾角过大时，电机紧急停车，保护小车
//  入口参数：   小车倾角
//  返回参数：   无
//==========================================================
void Motor_Stop(float *Mid_Angle, float *Angle)
{
    if((*Mid_Angle - *Angle) > 40 || (*Mid_Angle - *Angle) < -40)
    {
        Motor1_SetSpeed(0);
        Motor2_SetSpeed(0);
//        Buzzer_Ring();
    }
    else {
//        Buzzer_Stop();
    }
}

//==========================================================
//  函数名称：   Speed_Pid_Ctrl
//  函数功能：   速度闭环控制
//  入口参数：   目标值，实际值
//  返回参数：   占空比
//==========================================================
u16 Speed_Pid_Ctrl (int targetSpeed,int actualSpeed)
{
    float bias = 0;   // 目标值与实际值的差值
    static float lastBias = 0;   // 上次偏差
    static float lastLastBias = 0;   // 上上次偏差
    static float pwmDuty = 0;   // PWM占空比

    // 比例调节系数范围1.6
    float kp = 1.6;   // 比例调节
    float ki = 0;   // 积分调节
    float kd = 0;   // 微分调节

    printf ("target = %d actual = %d\r\n",targetSpeed,actualSpeed);

    lastLastBias = lastBias;
    lastBias = bias;

    bias = (float)targetSpeed - (float)actualSpeed;

    pwmDuty += kp * (bias - lastBias) + ki * bias + kd * (bias - 2 * lastBias + lastLastBias);

    // 限幅
    if (pwmDuty >= 999)
    {
        pwmDuty = 999;
    }

    return (u16)pwmDuty;
}

//==========================================================
//  函数名称：   Motor1_GetDir
//  函数功能：   获取电机1的方向
//  入口参数：   无
//  返回参数：   电机1的方向，正转为0，反转为-1
//==========================================================
uint32_t Motor1_GetDir(void)
{
    return GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9);
}

//==========================================================
//  函数名称：   Motor2_GetDir
//  函数功能：   获取电机2的方向
//  入口参数：   无
//  返回参数：   电机2的方向，正转为0，反转为-1
//==========================================================
uint32_t Motor2_GetDir(void)
{
    return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
}

//==========================================================
//  函数名称：   Motor1_GetFreq
//  函数功能：   获取电机1的频率
//  入口参数：   无
//  返回参数：   电机1的频率，正转为正，反转为负
//==========================================================
int32_t Motor1_GetFreq(void)
{
    return (Motor1_GetDir() == 1) ? Motor1_is_there_speed * 1000000 / TIM_GetCapture2(TIM1): Motor1_is_there_speed * (-1) * 1000000 / TIM_GetCapture2(TIM1);
}

//==========================================================
//  函数名称：   Motor2_GetFreq
//  函数功能：   获取电机2的频率
//  入口参数：   无
//  返回参数：   电机2的频率，正转为正，反转为负
//==========================================================
int32_t Motor2_GetFreq(void)
{
    return (Motor2_GetDir() == 1) ? Motor2_is_there_speed * 1000000 / TIM_GetCapture2(TIM2) : Motor2_is_there_speed * (-1) * 1000000 / TIM_GetCapture2(TIM2);
}
