/*
 * MPU6050.c
 *
 *  Created on: 2024��9��28��
 *      Author: jacob
 */

/*
 *  �������˲�������ֲ��Github�⿪Դ���룺https://github.com/leech001/MPU6050
 */

#include <time.h>
#include <math.h>
#include "debug.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS     0xD0
#define RAD_TO_DEG 57.295779513082320876798154814105

const double Accel_Z_corrector = 14418.0;
uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

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

//==========================================================
//  �������ƣ�   MPU6050_Read_All
//  �������ܣ�   ��ȡ������MPU6050������
//  ��ڲ�����   MPU6050_t���ͽṹ��
//  ���ز�����   ��
//==========================================================
void MPU6050_Read_All(MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[12];

    // Read 12 BYTES of data starting from ACCEL_XOUT_H register

    Rec_Data[0] = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    Rec_Data[1] = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);

    // ��ȡ���ٶȼ� Y ������
    Rec_Data[2] = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    Rec_Data[3] = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);

    // ��ȡ���ٶȼ� Z ������
    Rec_Data[4] = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    Rec_Data[5] = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);


    // ��ȡ������ X ������
    Rec_Data[6] = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    Rec_Data[7] = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);

    // ��ȡ������ Y ������
    Rec_Data[8] = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    Rec_Data[9] = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);

    // ��ȡ������ Z ������
    Rec_Data[10] = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    Rec_Data[11] = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = -(int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    double dt = (double)(clock() - timer) / CLOCKS_PER_SEC;
    timer = clock();
    double roll;
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
}

//==========================================================
//  �������ƣ�   Kalman_getAngle
//  �������ܣ�   �������˲����㺯��
//  ��ڲ�����   Kalman - ָ�򿨶����˲����ṹ���ָ��
//          newAngle - �µĽǶ�ֵ��
//          newRate - �µĽ��ٶ�ֵ��
//          dt - ʱ������
//  ���ز�����   ���º�ĽǶ�
//==========================================================
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};
