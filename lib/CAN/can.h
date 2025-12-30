#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//
//////////////////////////////////////////////////////////////////////////////////
// 定义 CAN ID
#define CAN_ID_MOTOR_FB    0x101  // F103 发回给 F407 的反馈
#define CAN_ID_FILTER_START   0x200  // 32位mask模式下允许0x100-0x10F 的ID通过 F407 发给 F103 

/*typedef struct {
    int32_t RealSpeed;   // 4字节，实际速度
    int32_t Position;    // 4字节，累计位置 (部分)
} Motor_Feedback_t;*/

// CAN 初始化函数
u8 CAN_Mode_Init(void);         //CAN初始化

// pos: 0-4095, speed: 放大10倍的度/秒, status: 系统状态位
void CAN_Send_Feedback(uint16_t pos, int16_t speed, uint8_t status);

// 过滤器模式
void CAN_Filter_MODE_16BIT_LIST(void);
void CAN_Filter_MODE_16BIT_MASK(void);
void CAN_Filter_MODE_32BIT_LIST(void);
void CAN_Filter_MODE_32BIT_MASK(void);

#endif


