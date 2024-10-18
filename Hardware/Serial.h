/*
 * Serial.h
 *
 *  Created on: 2024��9��27��
 *      Author: jacob
 */

#ifndef HARDWARE_SERIAL_H_
#define HARDWARE_SERIAL_H_

#include <stdio.h>

extern uint8_t Serial_TxPacket[];
extern uint8_t Serial_RxPacket[];

void Serial_Init(void);
void USART1_SendByte(uint8_t Byte);
void USART1_SendArray(uint8_t *Array);
void USART1_SendString(char *String);
void USART1_SendNumber(uint32_t Number);
//void USART1_Printf(char *format, ...);
void USART1_SendPacket(void);
uint8_t Serial_GetRxFlag(void);


#endif /* HARDWARE_SERIAL_H_ */