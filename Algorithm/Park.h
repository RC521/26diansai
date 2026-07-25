#ifndef __PARK_H
#define __PARK_H

#include "arm_math.h" // 必须包含 DSP 库以使用极速查表法

/* Park 变换结构体定义 */
typedef struct {
    // 1. 输入变量 (由 SOGI 或 Clark 变换提供)
    float Alpha;      // 静态坐标系下的同相分量
    float Beta;       // 静态坐标系下的正交分量 (滞后 90 度)
    float Theta;      // 当前的角度 (弧度制 0 ~ 2π)
    
    // 2. 输出变量 (计算结果，准备喂给 PI 控制器)
    float D;          // 旋转坐标系下的 d 轴分量 (通常代表无功/激磁)
    float Q;          // 旋转坐标系下的 q 轴分量 (通常代表有功/转矩，PLL用它算误差)
} Park_t;

/* 函数声明 */
void Park_Init(Park_t *park);
void Park_Update(Park_t *park, float alpha, float beta, float theta);

#endif