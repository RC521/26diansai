#ifndef __SPWM_H
#define __SPWM_H

#include "stdint.h"

/* SPWM 结构体定义 */
typedef struct {
    float    Period;      // 定时器的周期值 (例如 ARR 配置为 8400，这里填 8400.0f)
    uint32_t CCR1_Value;  // 算出来的左桥臂比较值 (准备推给 TIM1->CCR1)
    uint32_t CCR2_Value;  // 算出来的右桥臂比较值 (准备推给 TIM1->CCR2)
} SPWM_t;

/* 接口函数声明 */
void SPWM_Init(SPWM_t *spwm, float arr_period);
void SPWM_Update(SPWM_t *spwm, float theta, float amplitude);

#endif