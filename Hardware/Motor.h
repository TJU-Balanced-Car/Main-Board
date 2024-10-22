/*
 * Motor.h
 *
 *  Created on: 2024Äê10ÔÂ11ÈÕ
 *      Author: mdfucker
 */

#ifndef HARDWARE_MOTOR_H_
#define HARDWARE_MOTOR_H_

void Motor_Init(void);
void Motor1_SetSpeed(float Speed);
void Motor2_SetSpeed(float Speed);
void Motor1_SetDir(uint8_t Dir);
void Motor2_SetDir(uint8_t Dir);

#endif /* HARDWARE_MOTOR_H_ */
