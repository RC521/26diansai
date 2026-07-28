#ifndef __SOGI_H
#define __SOGI_H

/* SOGI 结构体定义 */
typedef struct {
    // 1. 配置参数
    float k;           // 阻尼系数：决定滤波器的响应速度和超调量，通常取 1.414 (根号2)
    float Ts;          // 系统的采样周期 dt (例如 10kHz 中断就是 0.0001)
    
    // 2. 核心输出结果 (状态变量)
    float alpha;       // 输出 1：与电网同相位的滤波电压 (代替原始的 ADC 波形)
    float beta;        // 输出 2：滞后 90 度的正交电压 (凭空造出来的波)
    
} SOGI_t;

/* 函数声明 */
void SOGI_Init(SOGI_t *sogi, float k, float ts);
void SOGI_Update(SOGI_t *sogi, float v_grid, float omega);

#endif