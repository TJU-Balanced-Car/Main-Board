/*
 * Motor.h
 *
 *  Created on: 2024Äê10ÔÂ11ÈÕ
 *      Author: mdfucker
 */

#ifndef HARDWARE_MOTOR_H_
#define HARDWARE_MOTOR_H_

void Motor_Init(void);
void Motor_Encoder_Init(void);
void Motor1_SetSpeed(float Speed);
void Motor2_SetSpeed(float Speed);
void Motor1_SetDir(uint8_t Dir);
void Motor2_SetDir(uint8_t Dir);
void PWM_Limit(int *pwm);
void Motor_Stop(float *Mid_Angle, float *Angle);
u16 Speed_Pid_Ctrl (int targetSpeed,int actualSpeed);
int32_t Motor1_GetFreq(void);
int32_t Motor2_GetFreq(void);
uint32_t Motor1_GetDir(void);
uint32_t Motor2_GetDir(void);

#endif /* HARDWARE_MOTOR_H_ */
