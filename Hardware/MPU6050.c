/*
 * MPU6050.c
 *
 *  Created on: 2024年9月28日
 *      Author: jacob
 */

#include "debug.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS     0xD0

//==========================================================
//  函数名称：   MPU6050_WaitEvent
//  函数功能：   I2C超时检测
//  入口参数：   要检查的事件
//  返回参数：   无
//==========================================================
void MPU6050_WaitEvent(uint32_t I2C_EVENT)
{
    uint32_t Timeout;
    Timeout = 10000;
    while (I2C_CheckEvent(I2C2, I2C_EVENT) != READY)
    {
        Timeout --;
        if (Timeout == 0)
        {
            printf("ERROR: I2C Waiting Timeout\n");
            break;
        }
    }
}

//==========================================================
//  函数名称：   MPU6050_WriteReg
//  函数功能：   MPU6050写寄存器
//  入口参数：   寄存器地址，值
//  返回参数：   无
//==========================================================
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    I2C_GenerateSTART(I2C2, ENABLE);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    I2C_SendData(I2C2, RegAddress);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING);

    I2C_SendData(I2C2, Data);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);

    I2C_GenerateSTOP(I2C2, ENABLE);
}

//==========================================================
//  函数名称：   MPU6050_ReadReg
//  函数功能：   MPU6050读寄存器
//  入口参数：   寄存器地址
//  返回参数：   读取的值
//==========================================================
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    I2C_GenerateSTART(I2C2, ENABLE);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    I2C_SendData(I2C2, RegAddress);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);

    I2C_GenerateSTART(I2C2, ENABLE);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);
    MPU6050_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

    I2C_AcknowledgeConfig(I2C2, DISABLE);
    I2C_GenerateSTOP(I2C2, ENABLE);

    MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED);
    Data = I2C_ReceiveData(I2C2);

    I2C_AcknowledgeConfig(I2C2, ENABLE);

    return Data;
}

//==========================================================
//  函数名称：   MPU6050_Init
//  函数功能：   初始化MPU6050
//  入口参数：   无
//  返回参数：   无
//==========================================================
void MPU6050_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_InitTypeDef I2C_InitStructure;
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_ClockSpeed = 50000; // 时钟频率，应小于400k
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_Init(I2C2, &I2C_InitStructure);

    I2C_Cmd(I2C2, ENABLE);

    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01); // 解除睡眠，选择陀螺仪时钟
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00); // 6个轴均不待机
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09); // 采样分频为10
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06); // 滤波参数最大
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18); // 陀螺仪最大量程
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18); // 加速度计最大量程
}

//==========================================================
//  函数名称：   MPU6050_GetID
//  函数功能：   获取MPU6050的设备ID
//  入口参数：   无
//  返回参数：   设备ID的值
//==========================================================
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

//==========================================================
//  函数名称：   MPU6050_GetData
//  函数功能：   获取MPU6050的数据
//  入口参数：   6个指针，分别指向3个加速度计和3个陀螺仪的数据
//  返回参数：   无
//==========================================================
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                    int16_t * GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t DataH, DataL;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    *AccX = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    *AccY = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    *AccZ = -((DataH << 8) | DataL);

    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    *GyroX = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    *GyroY = (DataH << 8) | DataL;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    *GyroZ = (DataH << 8) | DataL;
}
