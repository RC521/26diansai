#include "spwm.h"
#include "arm_math.h"
/**
 * @brief  初始化 SPWM 模块
 * @param  spwm:       结构体指针
 * @param  arr_period: 硬件定时器的 ARR 值 (如 8400)
 */
void SPWM_Init(SPWM_t *spwm, float arr_period) {
    spwm->Period = arr_period;
    spwm->CCR1_Value = 0;
	  spwm->CCR2_Value = 0;
}

/**
 * @brief  生成单相全桥 SPWM 占空比 (双极性调制)
 * @param  spwm:      结构体指针
 * @param  theta:     当前电网的相位角 (由 PLL 提供，范围 0 ~ 2π)
 * @param  amplitude: 调制幅度 (由外部 PI 控制器提供，范围 0.0 ~ 1.0)
 */
void SPWM_Update(SPWM_t *spwm, float theta, float amplitude) {
    // 1. 软件限幅，防止占空比超调爆炸（给死区留一点点极限余量）
    if (amplitude > 0.98f) amplitude = 0.98f; 
    if (amplitude < 0.0f)  amplitude = 0.0f;
    
    // 2. 查表算出当前相位的标准正弦值 (-1.0 到 1.0)
    float sin_value = arm_sin_f32(theta);
    
    // 3. 核心计算：引入幅度，并计算左桥臂占空比 (0.0 ~ 1.0)
    float duty_cycle_left = ((sin_value * amplitude) + 1.0f) / 2.0f;
    
    // 4. 倒相逻辑：右桥臂的占空比直接用 1 减去左边即可
    float duty_cycle_right = 1.0f - duty_cycle_left;
    
    // 5. 将小数占空比映射为定时器寄存器的真实数值
    spwm->CCR1_Value = (uint32_t)(duty_cycle_left * spwm->Period);
    spwm->CCR2_Value = (uint32_t)(duty_cycle_right * spwm->Period);
    
    // 6. 最后一道硬件保护防线 (防止算出极其危险的满占空比)
    // 留下 1 的余量，配合硬件死区，绝对防止上下管直通
    if (spwm->CCR1_Value >= (uint32_t)spwm->Period) spwm->CCR1_Value = (uint32_t)spwm->Period - 1;
    if (spwm->CCR2_Value >= (uint32_t)spwm->Period) spwm->CCR2_Value = (uint32_t)spwm->Period - 1;
}