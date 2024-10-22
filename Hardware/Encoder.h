/*
 * Encoder.h
 *
 *  Created on: 2024Äê10ÔÂ15ÈÕ
 *      Author: jacob
 */

#ifndef HARDWARE_ENCODER_H_
#define HARDWARE_ENCODER_H_

void TIM5_Init(void);
void Encoder_Init(void);
int32_t Motor1_GetFreq(void);
int32_t Motor2_GetFreq(void);
uint32_t Motor1_GetDir(void);
uint32_t Motor2_GetDir(void);

#endif /* HARDWARE_ENCODER_H_ */
