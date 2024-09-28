/*
 * MPU6050.h
 *
 *  Created on: 2024Äê9ÔÂ28ÈÕ
 *      Author: jacob
 */

#ifndef HARDWARE_MPU6050_H_
#define HARDWARE_MPU6050_H_

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                    int16_t * GyroX, int16_t *GyroY, int16_t *GyroZ);

#endif /* HARDWARE_MPU6050_H_ */
