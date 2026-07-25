#ifndef __APP_PLL_H
#define __APP_PLL_H

#include "Algorithm.h"

// 定义常量
#define PI_VALUE 3.14159265f
#define GRID_BASE_FREQ 50.0f    // 电网基准频率 50Hz

/* 单相锁相环 (Single-Phase PLL) 系统结构体 */
typedef struct {
    // 1. 挂载算法模块对象
    SOGI_t          sogi;
    Park_t          park;
    PI_Controller_t pi;
    
    // 2. 系统运行参数
    float dt;               // 系统的采样周期 (例如 10kHz 下为 0.0001)
    
    // 3. 运行过程中的关键状态 (保留下来方便用串口打印 Debug)
    float delta_omega;      // PI 算出来的角速度微调量
    float current_omega;    // 当前实际的运行角速度
    
    // 4. 最终输出结果
    float theta;            // 当前电网相位 (0 ~ 2π)
    float real_freq_hz;     // 实时推算出的电网真实频率 (约 50.0Hz)
} APP_PLL_t;

/* 外部接口函数声明 */
void APP_PLL_Init(APP_PLL_t *pll, float sample_time);
void APP_PLL_Update(APP_PLL_t *pll, float v_grid_real);

#endif