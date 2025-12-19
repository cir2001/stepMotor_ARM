#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "oled.h"
#include "port.h"
//////////////////////////////////////////////////////////////////////////////////	 
extern int32_t Target_Speed_Hz;
extern uint32_t CAN_Timeout_Counter;

//////////////////////////////////////////////////////////////////////////////////
//CAN初始化
//位时间设置：
//tsjw:重新同步跳跃时间单元.范围:1~3;
//tbs2:时间段2的时间单元.范围:1~8;
//tbs1:时间段1的时间单元.范围:1~16;
//brp :波特率分频器.范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
//注意以上参数任何一个都不能设为0,否则会乱.
//波特率=Fpclk1/((tbs1+tbs2+1)*brp);
//Fpclk1的时钟在初始化的时候设置为36M
//因此，brp  = (36000000 / 18) / 500000; 500000将波特率设置为500kbps,18为 TSEG1+TSEG2+SJW的值
//如果设置SJW=1, BS1（TSEG1）=12, BS2（TSEG2）=5
//参数       可设置的范围    实际设置的有效范围
//SJW        0-3						1-4tq		
//BS1 (TS1)  0-15						1-16tq
//BS2 (TS2)  0-7						1-8tq     注：寄存器值是 (实际值 - 1)
//则波特率为:36M/((1+12+5)*4)=500Kbps；(1+12+5)为实际设置在寄存器的值
//返回值:0,初始化OK;
//    其他,初始化失败;
u8 CAN_Mode_Init(void)
{
	u16 i=0;
	u8 brp;
	/* 使能 CAN1 和 GPIOA 时钟 */
	RCC->APB1ENR |= (1 << 25);    	//使能 CAN1 时钟  CAN使用的是APB1的时钟(max:36M)
	RCC->APB2ENR |= (1 << 2);     	//使能PORTA时钟	 	 
	
	/* 配置 PA11 (CAN_RX)为上拉输入、PA12(CAN_TX)为复用推挽输出 */
	GPIOA->CRH &= 0XFFF00FFF; 
	GPIOA->CRH |= 0X000B8000;		//PA11 RX,PA12 TX推挽输出   	 
  	GPIOA->ODR |= 3<<11;			//PA 上拉
					    
	/* 退出 SLEEP 模式 (若处于睡眠状态) */
	CAN1->MCR = 0x00000;			//退出睡眠模式(同时设置所有位为0)
	
	/* 进入初始化模式 */
	CAN1->MCR |= 1<<0;				//请求CAN进入初始化模式
	while((CAN1->MSR & (1<<0)) == 0)
	{
		i++;
		if(i > 100) return 2;		//进入初始化模式失败
	}
	
	CAN1->MCR &= ~(1 << 4);     // NART=0: 允许自动重传
    CAN1->MCR |= (1 << 6);      // ABOM=1: 自动离线管理
    CAN1->MCR |= (1 << 5);      // AWUM=1: 自动唤醒

	/*  设置波特率  */
	/* Note: this calculations fit for PCLK1 = 36MHz */
  	brp  = (36000000 / 18) / 500000;                     // baudrate is set to 500k bit/s
	/* set BTR register so that sample point is at about 72% bit time from bit start */
	/* TSEG1 = 15, TSEG2 = 4, SJW = 2 => 1 CAN bit = 18 TQ, sample at 72%    */
  	CAN1->BTR &= ~(((        0x03) << 24) | ((        0x07) << 20) | ((         0x0F) << 16) | (          0x1FF)); 
	//--- 波特率:Fpclk1/((Tbs1+Tbs2+1)*Fdiv) ---//
	CAN1->BTR |=  ((((2-1) & 0x03) << 24) | (((4-1) & 0x07) << 20) | (((13-1) & 0x0F) << 16) | ((brp-1) & 0x1FF)); 
	
	/* 退出初始化模式 */
	CAN1->MCR &= ~(1<<0);		//请求CAN退出初始化模式
	while((CAN1->MSR & (1<<0)) == 1)
	{
		i++;
		if(i > 0XFFF0) return 3;//退出初始化模式失败
	}
	
	/*  配置过滤器0 (可配置的过滤器0-13) */
	CAN1->FMR |= 1<<0;			//过滤器组工作在初始化模式
	CAN1->FA1R &= ~(1<<0);		//过滤器0不激活

	CAN_Filter_MODE_32BIT_MASK();

 	//使用中断接收
	CAN1->IER|=1<<1;		//FIFO0消息挂号中断允许.	    
	MY_NVIC_Init(2,0,USB_LP_CAN1_RX0_IRQn,2);//组2
	return 0;
}   

//中断服务函数			    
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	if(CAN1->RF0R & CAN_RF0R_FMP0) // FIFO0 有数据
    {
		int32_t recv_val;
		LED0 = !LED0;
		// 1. 读取 ID (标准帧 ID 在 RIR 寄存器高 11 位)
        uint32_t id = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;; // 读取标准 ID
        
		// 2. 读取数据长度 DLC
        u8 len = CAN1->sFIFOMailBox[0].RDTR & 0x0F;

        // 3. 读取数据 (RDLR 低4字节, RDHR 高4字节)
        // 假设前 4 字节是 int32 速度
        recv_val = 0;
        
        // 直接从寄存器拼装数据 (假设小端发送)
        // RDLR: [Data3 | Data2 | Data1 | Data0]
        recv_val = CAN1->sFIFOMailBox[0].RDLR; // 直接读出整个32位寄存器作为int32
        // 如果需要逐字节序处理：
        // ((CAN1->sFIFOMailBox[0].RDLR >> 0) & 0xFF) ...

        // 4. 处理数据 (这里不需要再判断 ID 了，因为硬件只放行 100-10F)
        if(recv_val > 25000) recv_val = 25000;
        if(recv_val < -25000) recv_val = -25000;
        
        Target_Speed_Hz = recv_val;
		
		CAN_Timeout_Counter = 0; // 重置超时计数器 timer2

        // 释放邮箱
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    }
}

//--------------------------------------------------------------------------
//can口发送一组数据至上位机F407(固定格式:CAN_ID_MOTOR_FB,标准帧,数据帧)
//real_speed:当前实际的速度
//pos_error:位置偏差.		
//--------------------------------------------------------------------------
void CAN_Send_Feedback(int32_t real_speed, int32_t pos_error)
{
    // 检查邮箱0是否空闲
    if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) 
    {
        // 1. 配置 TIR (ID 寄存器)
        // 明确设置：标准 ID (<< 21), IDE=0, RTR=0 (数据帧)
        // 暂不设置 TXRQ
        CAN1->sTxMailBox[0].TIR = ((uint32_t)CAN_ID_MOTOR_FB << 21); 

        // 2. 配置 TDTR (数据长度和控制)
        // 数据长度为 8 字节
        // IDE 和 RTR 默认为 0 (标准ID，数据帧)
        CAN1->sTxMailBox[0].TDTR = 8; 

        // 3. 填充 TDLR (Data Low Register, 字节 0-3)
        // 直接赋值 32 位数据，无需手动拆分字节
        CAN1->sTxMailBox[0].TDLR = (uint32_t)real_speed;
        
        // 4. 填充 TDHR (Data High Register, 字节 4-7)
        // 直接赋值 32 位数据
        CAN1->sTxMailBox[0].TDHR = (uint32_t)pos_error;

        // 5. 触发发送 (设置 TXRQ 位)
        CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ; 
    }
}
//--------------------------------------------------------------------------
//16bit list mode 4个ids
//--------------------------------------------------------------------------
void CAN_Filter_MODE_16BIT_LIST(void)
{
	//--- 允许通过的id 4*16bit list mode test ---//
	/*u16 id0 = 0x0058;
	u16 id1 = 0x0158;
	u16 id2 = 0x0258;
	u16 id3 = 0x0358;*/
	//****  16bit*4 list mode 可以通过4个指定的ID ****//
	/*CAN1->FM1R |= 1<<0;		//过滤器0工作在标识符list模式
	CAN1->FS1R &= ~(1<<0); 		//过滤器0位宽为16位*4.

	//--- FR1 高 16 位 id0，低 16 位 id1 ---//
	CAN1->sFilterRegister[0].FR1 = 
			((u32)id0 << 21) | (id1 << 5);
	//--- FR2 高 16 位 id3，低 16 位 id4 ---//			
	CAN1->sFilterRegister[0].FR2 = 
			((u32)id2 << 21) | (id3 << 5);*/

	//*************************************************************
	// 过滤器配置 - 接收 0x100, 0x101, 0x102, 0x103
	// 模式设置	
	CAN1->FS1R &= ~(1 << 0);       // 设为 16位宽
	CAN1->FM1R |= (1 << 0);       // 设为 列表模式 (List Mode)

	// --- 写入 ID 和 Mask ---
	// 目标范围: 0x100 ~ 0x103
	// 标准 ID 在寄存器的高 11 位 [31:21]
	// FR1 (ID): 0x100, 0x10	, 0x102, 0x103
	CAN1->sFilterRegister[0].FR1 = (0x100 << 21) |
	                                   (0x101 << 5);	// 低 16 位
	// FR2 (ID): 0x102, 0x10		, 0x103, 0x104
	CAN1->sFilterRegister[0].FR2 = (0x102 << 21) |
	                                   (0x103 << 5);	// 低 16 位	// 关联与激活
	CAN1->FFA1R &= ~(1 << 0);     // 关联到 FIFO 0
	CAN1->FA1R |= (1 << 0);       // 激活过滤器0
	CAN1->FMR &= ~(1 << 0);       // 过滤器组进入正常模式
}
//--------------------------------------------------------------------------
//16bit mask mode 2个ids
//--------------------------------------------------------------------------
void CAN_Filter_MODE_16BIT_MASK(void)
{
	//--- 允许通过的id 2*16bit mask mode test ---//
	/*u16 id0 = 0x200;		//起始id 0x200
	//-- mask允许0x200-0x20F通过过滤器，0x0c 表示RTR=0 IDE=1
	// bit7 bit6 bit5 bit4(RTR) bit3(IDE) bit2 bit1 bit0
	//   0    0    0    0            1      1    0    0
	// bit2为1的理解：保留位 不对CAN ID起作用，实现更严格的匹配
	u16 mask0 = 0xFE0C;		
	u16 id1 = 0x0300;
	u16 mask1 = 0xFE0C;*/
	//****  16bit*2 mask mode 可以通过一个范围内id，例如：0x200-0x02F ****//
	/*CAN1->FM1R &= ~(1<<0);		//过滤器0工作在标识符mask模式
	CAN1->FS1R &= ~(1<<0); 		//过滤器0位宽为16位*4.

	//--- FR1 高 16 位 id0，低 16 位 id1 ---//
	CAN1->sFilterRegister[0].FR1 = 
			((u32)mask0 << 16) | (id0 << 5);
	//--- FR2 高 16 位 id3，低 16 位 id4 ---//			
	CAN1->sFilterRegister[0].FR2 = 
			((u32)mask1 << 16) | (id1 << 5);*/
	
	//*********************************************************************
	// 过滤器配置 - 接收 0x100, 0x101, 0x102, 0x103
	// 模式设置	
	CAN1->FS1R &= ~(1 << 0);       // 设为 16位宽
	CAN1->FM1R &= ~(1 << 0);      // 设为 掩码模式 (Mask Mode)
	// --- 写入 ID 和 Mask ---
	// 目标范围: 0x100 ~ 0x103
	// 标准 ID 在寄存器的高 11 位 [31:21]
	// FR1 (ID): 0x100, 0x101, 0x102, 0x103
	CAN1->sFilterRegister[0].FR1 = (0x100 << 21) |
	                                   (0x101 << 5);	// 低 16 位			
	// FR2 (Mask): 0x102, 0x103, 0x104, 0x105
	CAN1->sFilterRegister[0].FR2 = (0x102 << 21) |
	                                   (0x103 << 5);	// 低 16 位
	// 关联与激活
	CAN1->FFA1R &= ~(1 << 0);     // 关联到 FIFO 0
	CAN1->FA1R |= (1 << 0);       // 激活过滤器0
	CAN1->FMR &= ~(1 << 0);       // 过滤器组进入正常模式
}
//--------------------------------------------------------------------------
//32bit list mode 4个ids
//--------------------------------------------------------------------------
void CAN_Filter_MODE_32BIT_LIST(void)
{
	// 过滤器配置 - 接收 0x100, 0x101, 0x102, 0x103
	// 模式设置	
	CAN1->FS1R |= (1 << 0);       // 设为 32位宽
	CAN1->FM1R |= (1 << 0);       // 设为 列表模式 (List Mode)
	// --- 写入 ID 和 Mask ---
	// 目标范围: 0x100 ~ 0x103
	// 标准 ID 在寄存器的高 11 位 [31:21]
	// FR1 (ID): 0x100, 0x101, 0x102, 0x103
	CAN1->sFilterRegister[0].FR1 = (0x100 << 21) |
	                                   (0x101 << 5);	// 低 16 位			
	// FR2 (ID): 0x102, 0x103, 0x104, 0x105
	CAN1->sFilterRegister[0].FR2 = (0x102 << 21) |
	                                   (0x103 << 5);	// 低 16 位
	// 关联与激活
	CAN1->FFA1R &= ~(1 << 0);     // 关联到 FIFO 0
	CAN1->FA1R |= (1 << 0);       // 激活过滤器0
	CAN1->FMR &= ~(1 << 0);       // 过滤器组进入正常模式
}
//--------------------------------------------------------------------------
//32bit mask mode 
//--------------------------------------------------------------------------
void CAN_Filter_MODE_32BIT_MASK(void)
{
	//****  32bit mask mode 所有ID均通过过滤器，即没有屏蔽的设置 ****//
	/*CAN1->FM1R |= 0<<0;		//过滤器0工作在标识符屏蔽位模式
	CAN1->FS1R |= 1<<1; 		//过滤器0位宽为32位.
	
	//--- 1个32位ID id由于屏蔽寄存器FR2设置全0，因此可以设为初始值，全部id通过 ---//
	CAN1->sFilterRegister[0].FR1 = 0X00000000;
	//--- 1个32位MASK, 全0表示不关心每一位，也就是所有id通过 ---//
	CAN1->sFilterRegister[0].FR2 = 0X00000000;*/

	//*******************************************************************
	// 过滤器配置 - 接收 0x100 ~ 0x10F
	// 模式设置
	CAN1->FS1R |= (1 << 0);       // 设为 32位宽
	CAN1->FM1R &= ~(1 << 0);      // 设为 掩码模式 (Mask Mode)

	// --- 写入 ID 和 Mask ---
	// 目标范围: 0x100 ~ 0x10F
	// 标准 ID 在寄存器的高 11 位 [31:21]
	// FR1 (ID): 基准值 0x100
	CAN1->sFilterRegister[0].FR1 = ((u32)CAN_ID_FILTER_START << 21);
	// FR2 (Mask): 掩码设置。1 表示必须匹配 FR1 对应位
    // ID Mask: 0x7F0 (匹配前 7 位)
    // IDE Mask: 1 << 2 (匹配 IDE=0，即标准帧)
    // RTR Mask: 1 << 4 (匹配 RTR=0，即数据帧)
    CAN1->sFilterRegister[0].FR2 = ((u32)0x7F0 << 21) | // ID 掩码 (0x7F0)
                                   (1 << 2) |           // IDE 掩码 (匹配标准帧)
                                   (1 << 4);            // RTR 掩码 (匹配数据帧)

	//--- 1个32位ID id由于屏蔽寄存器FR2设置全0，因此可以设为初始值，全部id通过 ---//
	/*CAN1->sFilterRegister[0].FR1 = 0X00000000;
	//--- 1个32位MASK, 全0表示不关心每一位，也就是所有id通过 ---//
	CAN1->sFilterRegister[0].FR2 = 0X00000000;*/
    
    CAN1->FFA1R &= ~(1<<0); // FIFO0
    CAN1->FA1R |= 1<<0;     // 激活
    CAN1->FMR &= ~(1<<0);   // 退出初始化
}


