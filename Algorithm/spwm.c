#include "spwm.h"

/**
 * @brief  初始化 SPWM 模块
 * @param  spwm:       结构体指针
 * @param  arr_period: 硬件定时器的 ARR 值 (如 8400)
 */
void SPWM_Init(SPWM_t *spwm, float arr_period) {
    spwm->Period = arr_period;
    spwm->CCR_Value = 0;
}

/**
 * @brief  生成 SPWM 占空比
 * @param  spwm:      结构体指针
 * @param  theta:     当前电网的相位角 (由 PLL 提供，范围 0 ~ 2π)
 * @param  amplitude: 调制幅度 (由外部 PI 控制器提供，范围 0.0 ~ 1.0)
 */
void SPWM_Update(SPWM_t *spwm, float theta, float amplitude) {
    // 1. 软件限幅，防止占空比超调爆炸
    if (amplitude > 1.0f) amplitude = 1.0f;
    if (amplitude < 0.0f) amplitude = 0.0f;
    
    // 2. 查表算出当前相位的标准正弦值 (-1.0 到 1.0)
    float sin_value = arm_sin_f32(theta);
    
    // 3. 核心计算：引入幅度，并归一化到 0 ~ 1.0 之间
    // 原理: 先把振幅缩小，然后再整体向上平移，保证波形始终在横坐标上方
    float duty_cycle = ((sin_value * amplitude) + 1.0f) / 2.0f;
    
    // 4. 将小数占空比映射为定时器寄存器的真实数值
    spwm->CCR_Value = (uint32_t)(duty_cycle * spwm->Period);
    
    // 5. 最后一道硬件保护防线 (防止写入的值大于等于 ARR 导致常高电平)
    if (spwm->CCR_Value >= (uint32_t)spwm->Period) {
        spwm->CCR_Value = (uint32_t)spwm->Period - 1;
    }
}