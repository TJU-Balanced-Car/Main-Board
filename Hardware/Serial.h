/*
 * Serial.h
 *
 *  Created on: 2024Äê9ÔÂ27ÈÕ
 *      Author: jacob
 */

#ifndef HARDWARE_SERIAL_H_
#define HARDWARE_SERIAL_H_

#include <stdio.h>

typedef struct {
    float Roll;
    int Motor1Speed;
    int Motor2Speed;
    int ServoAngle;
    float VerticalKp;
    float VerticalKi;
    float VerticalKd;
    int VerticalOut;
    float VelocityKp;
    float VelocityKi;
    int VelocityOut;
} DataPacket;

extern char Serial_RxPacket[];

void Serial_Init(int BaudRate);
uint8_t USART1_Init(void);
void USART2_SendByte(uint8_t Byte);
void USART2_SendArray(uint8_t *Array);
void USART2_SendString(char *String);
void USART2_SendNumber(uint32_t Number);
//void USART2_Printf(char *format, ...);
uint8_t Serial_GetRxFlag(void);
void USART_SendDataPacket(USART_TypeDef* USARTx, DataPacket* packet);
void Wait_BLE_Response(USART_TypeDef* USARTx);
void Send_AT_Command(const char *cmd);


#endif /* HARDWARE_SERIAL_H_ */
