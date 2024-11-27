#include "mpuiic.h"
#include <debug.h>

//MPU IIC 延时函数
void MPU_IIC_Delay(void)
{
	delay_us(2);
}

//初始化IIC
void MPU_IIC_Init(void)
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
    I2C_InitStructure.I2C_ClockSpeed = 400000; // 时钟频率，应小于400k
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_Init(I2C2, &I2C_InitStructure);

    I2C_Cmd(I2C2, ENABLE);
}

//void MPU6050_IIC_SDA_IO_OUT(void)
//{
////  My_GPIO_Init(MPU6050_IIC_GPIO,MPU6050_IIC_SDA_Pin,GPIO_TW_OUT,GPIO_P_NO,GPIO_50MHz);//推挽输出 不拉 50m
//    GPIO_InitTypeDef GPIO_InitStructure;
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能PB端口时钟
//    GPIO_InitStructure.GPIO_Pin = MPU6050_IIC_SDA_Pin;  //端口配置
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
//}

//void MPU6050_IIC_SDA_IO_IN(void)
//{
////  My_GPIO_Init(MPU6050_IIC_GPIO,MPU6050_IIC_SDA_Pin,GPIO_FK_IN,GPIO_P_UP,GPIO_50MHz);//浮空输入 上拉 50m
//    GPIO_InitTypeDef GPIO_InitStructure;
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能PB端口时钟
//    GPIO_InitStructure.GPIO_Pin =MPU6050_IIC_SDA_Pin;   //端口配置
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;      //推挽输出
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
//}
//产生IIC起始信号
//void MPU_IIC_Start(void)
//{
//	MPU6050_IIC_SDA_IO_OUT();     //sda线输出
//	MPU_IIC_SDA=1;
//	MPU_IIC_SCL=1;
//	MPU_IIC_Delay();
// 	MPU_IIC_SDA=0;//START:when CLK is high,DATA change form high to low
//	MPU_IIC_Delay();
//	MPU_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据
//}
//产生IIC停止信号
//void MPU_IIC_Stop(void)
//{
//	MPU6050_IIC_SDA_IO_OUT();//sda线输出
//	MPU_IIC_SCL=0;
//	MPU_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
// 	MPU_IIC_Delay();
//	MPU_IIC_SCL=1;
//	MPU_IIC_SDA=1;//发送I2C总线结束信号
//	MPU_IIC_Delay();
//}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
//u8 MPU_IIC_Wait_Ack(void)
//{
//	u8 ucErrTime=0;
//	MPU6050_IIC_SDA_IO_IN();      //SDA设置为输入
//	MPU_IIC_SDA=1;MPU_IIC_Delay();
//	MPU_IIC_SCL=1;MPU_IIC_Delay();
//	while(MPU_READ_SDA)
//	{
//		ucErrTime++;
//		if(ucErrTime>250)
//		{
//			MPU_IIC_Stop();
//			return 1;
//		}
//	}
//	MPU_IIC_SCL=0;//时钟输出0
//	return 0;
//}
//==========================================================
//  函数名称：   MPU6050_WaitEvent
//  函数功能：   I2C超时检测
//  入口参数：   要检查的事件
//  返回参数：   失败为1，成功为0
//==========================================================
u8 MPU6050_WaitEvent(uint32_t I2C_EVENT)
{
    uint32_t Timeout;
    Timeout = 10000;
    while (I2C_CheckEvent(I2C2, I2C_EVENT) != READY)
    {
        Timeout --;
        if (Timeout == 0)
        {
            printf("ERROR: I2C Waiting Timeout\n");
            return 1;
        }
    }
    return 0;
}
////产生ACK应答
//void MPU_IIC_Ack(void)
//{
//	MPU_IIC_SCL=0;
//	MPU6050_IIC_SDA_IO_OUT();
//	MPU_IIC_SDA=0;
//	MPU_IIC_Delay();
//	MPU_IIC_SCL=1;
//	MPU_IIC_Delay();
//	MPU_IIC_SCL=0;
//}
////不产生ACK应答
//void MPU_IIC_NAck(void)
//{
//	MPU_IIC_SCL=0;
//	MPU6050_IIC_SDA_IO_OUT();
//	MPU_IIC_SDA=1;
//	MPU_IIC_Delay();
//	MPU_IIC_SCL=1;
//	MPU_IIC_Delay();
//	MPU_IIC_SCL=0;
//}
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
//void MPU_IIC_Send_Byte(u8 txd)
//{
//    u8 t;
//	MPU6050_IIC_SDA_IO_OUT();
//    MPU_IIC_SCL=0;//拉低时钟开始数据传输
//    for(t=0;t<8;t++)
//    {
//        MPU_IIC_SDA=(txd&0x80)>>7;
//        txd<<=1;
//		MPU_IIC_SCL=1;
//		MPU_IIC_Delay();
//		MPU_IIC_SCL=0;
//		MPU_IIC_Delay();
//    }
//}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
//u8 MPU_IIC_Read_Byte(unsigned char ack)
//{
//	unsigned char i,receive=0;
//	MPU6050_IIC_SDA_IO_IN();//SDA设置为输入
//    for(i=0;i<8;i++ )
//	{
//        MPU_IIC_SCL=0;
//        MPU_IIC_Delay();
//		MPU_IIC_SCL=1;
//        receive<<=1;
//        if(MPU_READ_SDA)receive++;
//		MPU_IIC_Delay();
//    }
//    if (!ack)
//        MPU_IIC_NAck();//发送nACK
//    else
//        MPU_IIC_Ack(); //发送ACK
//    return receive;
//}


















