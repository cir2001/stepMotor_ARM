#ifndef __USART_H
#define __USART_H
#include "sys.h"
#include "stdio.h"	 

//如果想串口中断接收，不要注释以下宏定义
#define EN_USART1_RX //使能串口1接收
#define EN_USART2_RX //使能串口2接收

void uart_init1(u32 pclk2,u32 bound);
void uart_init2(u32 pclk2,u32 bound);

void UART1ComReply(void);
void UART2ComReply(u8 ComName);

void UART1ComReplyTest(u8 SPI_Recive[16]);

void Uart2MotorEncoderComReply(s16 MotorAEncoder,s16 MotorBEncoder,s16 MotorCEncoder);
void Uart2TotalTrans(short ax,short ay,short az,short gx,short gy,short gz,float pitch,float roll,float yaw,s16 Ma,s16 Mb,s16 Mc);


#endif	   

