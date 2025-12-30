#ifndef __TIMER_H
#define __TIMER_H

#include "sys.h"
//==============================================
void Timer2_Init(u16 arr,u16 psc);
void Timer3_Init(u16 arr,u16 psc);

// 机械参数
#define MICRO_STEP        16    // 细分
#define STEPPER_FULL_STEP 200   // 电机物理步数
// 一圈需要的脉冲数 = 200 * 16 = 3200
#define PULSES_PER_REV    (STEPPER_FULL_STEP * MICRO_STEP) 
// 编码器一圈数值
#define ENCODER_RES       4096 

// 定点数运算参数 (使用 int64_t 避免 double 性能问题)
#define SCALING_FACTOR    1000  // 放大1000倍
// 比例系数: 1个脉冲 = 多少个编码器单位 (放大后)
// (4096 / 3200) * 1000 = 1280
#define RATIO_SCALED      ((ENCODER_RES * SCALING_FACTOR) / PULSES_PER_REV)

#define MAX_CAN_TIMEOUT_TICKS  20          // 5ms * 20 = 100ms 未收到指令则保护
#define COMP_KD  0.02f   // 阻尼系数，需根据实验调整
#define COMP_KP  0.001f // 补偿力度 (P参数)
// 设定死区，误差在 3 个编码器单位以内不进行补偿
#define COMP_DEADZONE 3

#endif























