////////////////////////////////////////////////////////////
//  Project Name: stepMotor stm32f103C6t6程序  自制PCB V0.1
//	File Name: stepMotor.C	 
//  芯片：STM32F103C6T6A	
//  version: V0.1
//	Author: ZJ
//	At:	xi'an China
//  Date From:202501030
//	**********************************
//	功能：
//		1.42 stepmotor control drv8825 module
//		2.CAN send receive
//		3.0.96oled display
//		4.uart2 send receive PC		
//		5.AS5600 read angle
// 	date		comment
//  2025-10-30  初始化
//	2025-11-12	VScode开发
//	2025-11-20	添加CAN收发功能
//	2025-12-05	添加AS5600读角度功能
//	2025-12-10	添加oled显示功能
//	2025-12-14	添加步进电机控制功能 串口2发送转速指令 指令格式：#S-10000 or #S-10000.
//	2025-12-15	更改can收发函数
//  2025-12-16  PlatformIO开发
//  2025-12-17  F407 F103 can联机测试
//*********** 引脚连接表 ***********************//
//	0.96OLED(SD1306)SPI1		stm32f103
//		GND						GND
//		Vcc						3.3V
//		D0	（SPI时钟线SCK）	 PA5
//		D1	（SPI数据线SDO）	 PA7
//		RES （复位）			 PC13
//		DC  （数据或命令切换）	  PC14	OLED_A0
//		CS   (片选)				 PA4   OLED_CS
//----------------
//	drv8825 modual              stm32f103
//		ENA						PA9
//		STEP					PB14
//		DIR						PB13
//		M0						PA8
//		M1						PB15
//		M2						PB14
//---------------
//	 AS5600 IIC 				stm32f103
//		GND						GND
//		Vcc						3.3V
//		SCL						PB10
//		SDA						PB11
//      DIR                     PB0
//      OUT					 	PA0
//      GPO 					PB1
//---------------
//	 CAN						stm32f103
//		CTX						PA12
//		CRX						PA11
//==================================================
//说明：
//---- UART2 ------
// UART2   115200，1start，8bit，1stop，no parity  	未使用
//******************************************************//
#include "stm32f10x.h"
#include "usart.h"		
#include "port.h"
#include "timer.h"
#include "stmflash.h"
#include "delay.h"

#include "exti.h" 
#include "SysTick.h"
#include "stdint.h"			//定义bool变量需要加入这行
#include "stdbool.h"		//定义bool变量需要加入这行
#include <stdlib.h>
#include "math.h"

#include "spi.h"
#include "oled.h"
#include "can.h"
#include "AS5600.h"
#include "stepMotor.h"
//-------------------
#define BufferSize 16
//--------------------
//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)
#define FLASH_SAVE_ADDR  0X08010000 
#define BufferSize 16
//----------------------------------------------------
// 自定义函数声明 
//-----------------------------------------------------

//----------------------------------------------------
// 外部变量声明 
//-----------------------------------------------------
u16 FlashRead[32];
extern u8 USART2_RX_BUF[128];     	//接收缓冲,最大128个字节.
extern u8 USART2_TX_BUF[128];     	//发送缓冲,最大128个字节.
extern u8  USART1_RX_BUF[128];     	//接收缓冲,最大128个字节.
extern u8  USART1_TX_BUF[128];     	//发送缓冲,最大128个字节.
extern u8 TxD1pt,TxD2pt,TxD1Num,TxD2Num;

// 闭环控制相关
extern int64_t Theory_Pos_Scaled; // 理论位置 (放大版)
extern int64_t Actual_Pos_Scaled; // 实际位置 (放大版)
extern int32_t Position_Error; // 误差 (实际编码器单位)
extern u16 Last_AS5600_Raw;           // 上次编码器读数

// 速度控制相关
extern int32_t Target_Speed_Hz;   // 用户设定的目标速度
extern int32_t Real_Output_Hz;   // 实际发给电机的速度 (包含补偿)

extern u8 u8Uart2_flag;
extern int recv_uart2_val;
extern u16 Global_AS5600_Raw;
extern volatile float Global_Actual_Speed; // 单位：deg/s (度/秒)
//----------------------------------------------------
// 变量声明 
//-----------------------------------------------------
u8 i;
u32 oled_tick;
// 调试/显示相关
volatile int32_t User_Cmd_Speed = 0;   // 串口发来的最终目标
//==========================================================
int main(void)
{
//****** 初始化 *******//
	Stm32_Clock_Init(9);     		//系统时钟设置
	delay_init(72);	  		        //延时初始化

	PORT_Init();					//IO端口初始化
	delay_ms(10);
	
	SPI1_Init();					//0.96OLED初始化
	delay_ms(10);
	OLED_Init();
	delay_ms(10);

    CAN_Mode_Init();				//CAN初始化,波特率500Kbps    
	delay_ms(10);

    StepMotor_Init();				//步进电机初始化
	delay_ms(10);
    AS5600_Init();                 //AS5600初始化
	
	uart_init2(36,115200);	 		//串口2初始化为57600 APB1/2预分频（与上位机通讯）
	delay_ms(10);

	//Timer2_Init(999,71);				//定时器2初始化 1ms
	Timer2_Init(4999,71);				//定时器2初始化 5ms
	delay_ms(10);
	
//******** 变量初始化 **********//
	// OLED 静态显示 (画表格、写固定的字)
    // 这些字只写一次，循环里不要重复刷，提高效率
    OLED_Clear();
	OLED_ShowString_Data(0, 0,  "CAN_ID:  ",CAN_ID_FILTER_START);
	OLED_ShowString_Data(0, 16, "ID_FB :  ",CAN_ID_MOTOR_FB);
    OLED_ShowString(0, 32, (unsigned char *)"Tar_Re:");
    OLED_ShowString(0, 48, (unsigned char *)"Spd:");

    OLED_Refresh_Gram(); // 第一次刷显存
	// 简单的加速启动示例 
	/*for( i = 1000; i <= 25000; i += 100) 
	{
		StepMotor_SetSpeed(i);
		delay_ms(5); // 给电机一点反应时间
	}*/
	// 加速完后，它就会一直保持 25000 转
	// 读取一次初始角度，防止上电第一帧数据跳变
    Last_AS5600_Raw = AS5600_GetRawAngle(); 

    // 4. 设定一个初始速度进行测试
    // 目标：2000Hz (约 0.6圈/秒)
    //Target_Speed_Hz = 2000;
//*********** 主循环程序 ****************//
	while(1)
	{	
		// ============================
        // 主循环：处理紧急任务
        // ============================
		// 1. 检查是否有新指令
        if (u8Uart2_flag)
        {
			// 从串口2接收新指令
            User_Cmd_Speed = recv_uart2_val;
            // 清除标志位
            u8Uart2_flag = 0;
			Target_Speed_Hz = User_Cmd_Speed;
            UART2ComReply();
        }
		// ============================
        // 主循环：处理非紧急任务
        // ============================
        // 每 100ms 刷新一次屏幕 (1ms * 100 = 100ms)
        if (oled_tick >= 20) //在timer2中断里计数
        {
            
            oled_tick = 0; // 清零计时器
            LMain = !LMain;

            char buf[20]; // 缓冲区

            // 处理 Target_Speed
            sprintf(buf, "Tar_Re: %6ld", (long int)Target_Speed_Hz);// "%-7d" 左对齐，占7位
            OLED_ShowString(0, 32, (unsigned char *)buf);

            // 获取绝对值用于计算
            float display_speed = Global_Actual_Speed;
            char sign = (display_speed >= 0) ? ' ' : '-'; // 判断符号
            if (display_speed < 0) display_speed = -display_speed; // 转为正数方便取位

            // 拆分整数和小数
            int32_t speed_int = (int32_t)display_speed;
            int32_t speed_dec = (int32_t)(display_speed * 10.0f) % 10;

            // 格式化输出：符号 + 整数 + 点 + 小数
            // %c 显示符号，%3ld 保证数字对齐
            sprintf(buf, "Spd: %c%3ld.%1ld", sign, (long int)speed_int, (long int)speed_dec);
            
            OLED_ShowString(0, 48, (unsigned char *)buf);
            OLED_Refresh_Gram();
        }
	}   // while(1) end
}		// int main(void) end




