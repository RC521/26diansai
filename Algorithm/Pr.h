#ifndef __PR_H
#define __PR_H

#include "stdint.h"

/* QPR (准比例谐振) 控制器结构体 */
typedef struct {
    // 1. 基础控制参数
    float Kp;            // 比例系数 (处理瞬态响应)
    float Kr;            // 谐振增益
    float target_freq_hz;// 谐振目标频率，例如 50 Hz
    float bandwidth_hz;  // 谐振带宽，例如 5 Hz
    float sample_time;   // 控制器调用周期，例如 0.0001 s
    
    // 2. 谐振积分项系数 (由 Kr, wc, w0 和离散时间 Ts 提前算好)
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    
    // 3. 历史状态缓存 (差分方程的核心，必须保存前两次的值)
    float error_prev1;   // e[k-1]
    float error_prev2;   // e[k-2]
    float output_prev1;  // y[k-1] (仅谐振部分的输出)
    float output_prev2;  // y[k-2] (仅谐振部分的输出)
    
    // 4. 物理防线 (限幅)
    float Max_Output;    // 输出上限 (例如 SPWM 调制比上限 1.0)
    float Min_Output;    // 输出下限 (例如 0.0 或 -1.0)
    
} PR_Controller_t;

/* 接口函数声明 */
void PR_Init(PR_Controller_t *pr, float kp, float kr,
             float target_freq_hz, float bandwidth_hz, float sample_time,
             float max_out, float min_out);
void PR_Reset(PR_Controller_t *pr);
void PR_Set_Coeffs(PR_Controller_t *pr, float b0, float b1, float b2, float a1, float a2);
float PR_Update(PR_Controller_t *pr, float target, float actual);

#endif
