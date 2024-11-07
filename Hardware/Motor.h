/*
 * Motor.h
 *
 *  Created on: 2024��10��11��
 *      Author: mdfucker
 */

#ifndef HARDWARE_MOTOR_H_
#define HARDWARE_MOTOR_H_

void Motor_Init(void);
void Motor1_SetSpeed(float Speed);
void Motor2_SetSpeed(float Speed);
void Motor1_SetDir(uint8_t Dir);
void Motor2_SetDir(uint8_t Dir);
void PWM_Limit(int *pwm);
void Motor_Stop(float *Mid_Angle, float *Angle);
u16 Speed_Pid_Ctrl (int targetSpeed,int actualSpeed);

#endif /* HARDWARE_MOTOR_H_ */
