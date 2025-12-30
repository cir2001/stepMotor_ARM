#include "timer.h"
#include "port.h"
#include "usart.h"
#include "delay.h"
#include "AS5600.h"
#include "stepMotor.h"
#include <stdlib.h>
#include "can.h"
//==========================================

//*******************************************
extern u8 USART1_TX_BUF[128],USART2_TX_BUF[128];
extern u8 TxD1pt,TxD2pt,TxD1Num,TxD2Num;

extern u8 u8Uart2_flag_test;
extern u32 oled_tick;
//--------------------------------------------
//变量声明
//--------------------------------------------
// 闭环控制相关
volatile int64_t Theory_Pos_Scaled = 0; // 理论位置 (放大版)
volatile int64_t Actual_Pos_Scaled = 0; // 实际位置 (放大版)
volatile int32_t Position_Error = 0; // 误差 (实际编码器单位)
u16 Last_AS5600_Raw = 0;           // 上次编码器读数

// 速度控制相关
volatile int32_t Target_Speed_Hz = 0;   // 用户设定的目标速度
volatile int32_t Real_Output_Hz = 0;   // 实际发给电机的速度 (包含补偿)

volatile uint32_t CAN_Timeout_Counter = 0; // 通信超时计数器
uint8_t System_Status = 0x01;              // 系统状态位，初始设为 0x01 (Ready)

u16 Global_AS5600_Raw = 0;
volatile float Global_Actual_Speed = 0.0f; // 单位：deg/s (度/秒)
u16 TIM2_Tick1,TIM2_Tick2;
//**** Timer2 IRQHandler **********************************  	 
//===============================
//定时2中断服务程序	 未使用
//===============================  	 
void TIM2_IRQHandler(void)
{
    // 变量定义
    uint16_t curr_raw = 0;
    int32_t delta_enc = 0;
    int64_t err_scaled = 0;
    int32_t compensation = 0;
    int32_t final_speed = 0;
	int32_t temp_target = 0;
	int32_t max_change; 
    int32_t last_speed;
	int64_t total_numerator=0;
    int16_t speed_to_send = 0;

    int32_t p_term = 0;
    int32_t d_term = 0;
    
    // 静态变量 (保持状态)
    static int32_t remainder_accum = 0; // 余数累加器
    static float Actual_Speed_Filtered = 0.0f;
    static int32_t last_target_hz = 0; // 用于检测指令变化
    
    // 检查更新中断
    if(TIM2->SR & 0X0001)
    {
        TIM2->SR &= ~(1<<0); //进中断先清标志位，防止重入风险

        oled_tick++;
        // LED1 闪烁测试
        TIM2_Tick1++;
        if(TIM2_Tick1 > 100) 
		{
			 TIM2_Tick1 = 0;
			 LED1 = !LED1; 
		}
		// 测试电机旋转一周as5600旋转的角度，发送旋转一周的指令StepMotor_SetSpeed(1000);
        if(u8Uart2_flag_test == 1)
        {
            TIM2_Tick2++;
            if(TIM2_Tick2 >= 640) // 约 3.2秒超时
            {
                TIM2_Tick2 = 0;
                u8Uart2_flag_test = 0;
                //Target_Speed_Hz = 0; // 建议直接改目标速度，而不是直接操作电机
            }
        }
        // ============================================================
        // 通信保护程序 (Watchdog)
        // ============================================================
        CAN_Timeout_Counter++; 
        if(CAN_Timeout_Counter > MAX_CAN_TIMEOUT_TICKS) 
        {
            // 如果超过 100ms 没收到 F407 指令
            Target_Speed_Hz = 0;             // 强制目标速度归零
            System_Status |= (1 << 7);       // 标记状态位 Bit7 为通信丢失
            // StepMotor_Disable();          // 可选：如果是强制保护，可以直接失能电机
        }
        else
        {
            System_Status &= ~(1 << 7);      // 通信正常，清除 Bit7
        }
        // ============================================================
        // 指令切换重置逻辑：解决正反转速度不一致的核心
        // ============================================================
        // 如果目标速度的方向改变了，或者是从静止开始启动
        if ((Target_Speed_Hz > 0 && last_target_hz <= 0) || 
            (Target_Speed_Hz < 0 && last_target_hz >= 0) ||
            (Target_Speed_Hz == 0 && last_target_hz != 0)) 
        {
            Theory_Pos_Scaled = Actual_Pos_Scaled; // 同步位置
            remainder_accum = 0;                   // 清除余数
            Position_Error = 0;                    // 清除误差
        }
        last_target_hz = Target_Speed_Hz;
        // ============================================================
        // 1. 读取 AS5600 并处理位置
        // ============================================================
        curr_raw = AS5600_GetRawAngle();
        Global_AS5600_Raw = curr_raw;
        //curr_raw = Global_AS5600_Raw;
        
        // 【注意】如果Error一直是正数且增大，请解开下面这行的注释，注释掉上面那行
        delta_enc = -(int32_t)(curr_raw - Last_AS5600_Raw); 
        //delta_enc = curr_raw - Last_AS5600_Raw; 

        // 处理过零点 (0-4095)
        if (delta_enc > 2048)       delta_enc -= 4096;
        else if (delta_enc < -2048) delta_enc += 4096;
        
        Last_AS5600_Raw = curr_raw;
        // --- 【计算转速】 ---
        // 采样周期 dt = 5ms = 0.005s
        // 转速(度/秒) = (delta_enc / 4096) * 360 / 0.005
        // 简化系数：360 / (4096 * 0.005) = 17.578125
        float raw_speed_deg_s = (float)delta_enc * 17.578125f;

        // 一阶低通滤波 (防止转速信号抖动，0.3是滤波系数)
        Actual_Speed_Filtered = Actual_Speed_Filtered * 0.7f + raw_speed_deg_s * 0.3f;

        Global_Actual_Speed = Actual_Speed_Filtered;

        // 转换为整数发送，放大10倍保留一位小数 (例如 123.4 deg/s 发送 1234)
        speed_to_send = (int16_t)(Actual_Speed_Filtered * 10.0f);

        // 更新实际位置
        Actual_Pos_Scaled += (int64_t)delta_enc * SCALING_FACTOR;
        // ============================================================
        // 2. 更新理论位置 
        // ============================================================
        if (Target_Speed_Hz != 0) 
        {
            // 这里必须用 Target_Speed_Hz (纯净的目标速度)
            // -----------------------------------------------------------
            //total_numerator = ((int64_t)Target_Speed_Hz * RATIO_SCALED) + remainder_accum;
			total_numerator = ((int64_t)Target_Speed_Hz * RATIO_SCALED) + remainder_accum;
            Theory_Pos_Scaled += total_numerator / 200;  
            remainder_accum = total_numerator % 200;
        }
        else 
        {
            Theory_Pos_Scaled = Actual_Pos_Scaled;
            remainder_accum = 0; 
        }
        // ============================================================
        // 3. 计算误差 & 限制
        // ============================================================
        err_scaled = Theory_Pos_Scaled - Actual_Pos_Scaled;
        Position_Error = (int32_t)(err_scaled / SCALING_FACTOR);
        // ============================================================
        // 4. 计算补偿 (P控制)
        // ============================================================
        compensation = 0;
        if (Target_Speed_Hz != 0 && abs(Position_Error) > COMP_DEADZONE)
        {
            // P 项：比例项，驱动电机走向目标
            p_term = (int32_t)(Position_Error * COMP_KP);
            
            // D 项：微分项，使用已经计算好的滤波速度 Global_Actual_Speed
            // 注意：D 项的方向通常与速度相反，起到阻尼作用
            d_term = (int32_t)(Global_Actual_Speed * COMP_KD); 

            // 最终补偿 = P项 - D项
            compensation = p_term - d_term;

            // 安全限幅：补偿量不应超过目标速度的 50%，防止低速不换向
            int32_t limit = abs(Target_Speed_Hz) / 2;
            if (compensation > limit)  compensation = limit;
            if (compensation < -limit) compensation = -limit;
        }else
        {
            compensation = 0; // 误差极小时不抖动
        }
        // ============================================================
        // 5. 计算最终速度 (含加速度限制 Slew Rate Limit)
        // ============================================================
        // 1. 算出理想的目标值 (可能发生剧烈跳变)
		temp_target = Target_Speed_Hz + compensation;

		// 2. 加速度限制 (防止电机堵转的核心！)
		// 限制每 5ms 最多改变 100Hz (相当于 20000Hz/s 的加速度)
		max_change = 50; 
		last_speed = Real_Output_Hz;

		if (temp_target > last_speed + max_change)
			final_speed = last_speed + max_change;
		else if (temp_target < last_speed - max_change)
			final_speed = last_speed - max_change;
		else
			final_speed = temp_target;
        
        // 更新历史值
        Real_Output_Hz = final_speed; 
        // ============================================================
        // 6. 执行驱动
        // ============================================================
        StepMotor_SetSpeed(final_speed);

        // CAN发送参数：发送原始位置、滤波后的转速、系统状态
        CAN_Send_Feedback(Global_AS5600_Raw, speed_to_send, System_Status);     
    }   
}
//**** Timer2 IRQHandler **********************************  	 
//===============================
//定时3中断服务程序	 未使用
//===============================  	 
void TIM3_IRQHandler(void)
{
	if(TIM3->SR&0X0001)//溢出中断
	{
        TIM3->SR&=~(1<<0);//清除中断标志位 	

	}
}
//**** Timer2 IRQHandler **********************************  	 
//===============================
//定时4中断服务程序	 未使用
//===============================  	 
void TIM4_IRQHandler(void)
{
	if(TIM4->SR&0X0001)//溢出中断
	{
	
	}
	TIM4->SR&=~(1<<0);//清除中断标志位 	    
}
//=======Timer2 Init ============
//通用定时器初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//===============================
void Timer2_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<0;//TIM2时钟使能    
 	TIM2->ARR=arr;     //设定计数器自动重装值   
	TIM2->PSC=psc;     //预分频器7200,得到10Khz的计数时钟
	//这两个东东要同时设置才可以使用中断
	TIM2->DIER|=1<<0;   //允许更新中断				
	TIM2->DIER|=1<<6;   //允许触发中断	   
 	MY_NVIC_Init(1,0,TIM2_IRQn,2);//抢占1，子优先级1，组2									 
	TIM2->CR1|=0x01;    //使能定时器2
	//TIM2->CR1&=0xfffe;    //禁止定时器2
}
//=======Timer3 Init ============
//通用定时器初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//===============================
void Timer3_Init(u16 arr, u16 psc)
{
    // 1. 使能 TIM3 时钟
    RCC->APB1ENR |= 1 << 1; 

    // 2. 设置重装载值和预分频
    TIM3->ARR = arr;      
    TIM3->PSC = psc;      

    // 3. 中断设置
    TIM3->SR &= ~(1 << 0);   // 【重要】清空状态寄存器，防止初始化完立刻跳入中断
    TIM3->DIER |= 1 << 0;    // 允许更新中断 (UIE)
    // TIM3->DIER |= 1 << 6; // 如果不需要定时器级联，这一行可以删掉

    // 4. NVIC 优先级配置
    // 注意：抢占优先级 1 应该高于你 TIM2 的优先级（比如 TIM2 设为 2）
    MY_NVIC_Init(0, 0, TIM3_IRQn, 2); 

    // 5. 使能计数器
    TIM3->CR1 |= 1 << 0;     // 使能 TIM3
}




