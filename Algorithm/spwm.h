#ifndef __SPWM_H
#define __SPWM_H

#include "arm_math.h"
#include <stdint.h>

/* SPWM 结构体定义 */
typedef struct {
    float    Period;      // 定时器的周期值 (例如 ARR 配置为 8400，这里就填 8400.0f)
    uint32_t CCR_Value;   // 算法最终算出来的比较值，准备写入定时器寄存器
} SPWM_t;

/* 接口函数声明 */
void SPWM_Init(SPWM_t *spwm, float arr_period);
void SPWM_Update(SPWM_t *spwm, float theta, float amplitude);

#endif