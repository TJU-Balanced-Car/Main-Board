/*
 * Encoder.h
 *
 *  Created on: 2024Äê10ÔÂ15ÈÕ
 *      Author: jacob
 */

#ifndef HARDWARE_ENCODER_H_
#define HARDWARE_ENCODER_H_

uint32_t Encoder1_Get_HighTime(void);
uint8_t Encoder1_Get_Dir(void);
uint32_t Encoder2_Get_HighTime(void);
uint8_t Encoder2_Get_Dir(void);
void Encoder_Init(void);

#endif /* HARDWARE_ENCODER_H_ */
