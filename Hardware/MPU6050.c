/*
 * MPU6050.c
 *
 *  Created on: 2024��9��28��
 *      Author: jacob
 */

#include "debug.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS     0xD0

//==========================================================
//  �������ƣ�   MPU6050_WaitEvent
//  �������ܣ�   I2C��ʱ���
//  ��ڲ�����   Ҫ�����¼�
//  ���ز�����   ��
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
//  �������ƣ�   MPU6050_WriteReg
//  �������ܣ�   MPU6050д�Ĵ���
//  ��ڲ�����   �Ĵ�����ַ��ֵ
//  ���ز�����   ��
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
//  �������ƣ�   MPU6050_ReadReg
//  �������ܣ�   MPU6050���Ĵ���
//  ��ڲ�����   �Ĵ�����ַ
//  ���ز�����   ��ȡ��ֵ
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
//  �������ƣ�   MPU6050_Init
//  �������ܣ�   ��ʼ��MPU6050
//  ��ڲ�����   ��
//  ���ز�����   ��
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
    I2C_InitStructure.I2C_ClockSpeed = 50000; // ʱ��Ƶ�ʣ�ӦС��400k
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_Init(I2C2, &I2C_InitStructure);

    I2C_Cmd(I2C2, ENABLE);

    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01); // ���˯�ߣ�ѡ��������ʱ��
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00); // 6�����������
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09); // ������ƵΪ10
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06); // �˲��������
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18); // �������������
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18); // ���ٶȼ��������
}

//==========================================================
//  �������ƣ�   MPU6050_GetID
//  �������ܣ�   ��ȡMPU6050���豸ID
//  ��ڲ�����   ��
//  ���ز�����   �豸ID��ֵ
//==========================================================
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

//==========================================================
//  �������ƣ�   MPU6050_GetData
//  �������ܣ�   ��ȡMPU6050������
//  ��ڲ�����   6��ָ�룬�ֱ�ָ��3�����ٶȼƺ�3�������ǵ�����
//  ���ز�����   ��
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