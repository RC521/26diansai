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
 * RMS / 幅值闭环
 * 控制器输出的是“正弦幅度”
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
    float duty_cycle_right =duty_cycle_left;
    
    // 5. 将小数占空比映射为定时器寄存器的真实数值
    spwm->CCR1_Value = (uint32_t)(duty_cycle_left * spwm->Period);
    spwm->CCR2_Value = (uint32_t)(duty_cycle_right * spwm->Period);
    
    // 6. 最后一道硬件保护防线 (防止算出极其危险的满占空比)
    // 留下 1 的余量，配合硬件死区，绝对防止上下管直通
    if (spwm->CCR1_Value >= (uint32_t)spwm->Period) spwm->CCR1_Value = (uint32_t)spwm->Period - 1;
    if (spwm->CCR2_Value >= (uint32_t)spwm->Period) spwm->CCR2_Value = (uint32_t)spwm->Period - 1;
}


/**
 * @brief  瞬时电压闭环 / PR 控制
 * 适合：瞬时电压闭环 / PR 控制
 * 控制器输出的是“此刻瞬时调制量”
 * @param  spwm:   结构体指针
 * @param  control: 控制量 (范围 -0.98 到 0.98)
 */
void SPWM_Update_ByControl(SPWM_t *spwm, float control)
{
    if (control > 0.98f) control = 0.98f;
    if (control < -0.98f) control = -0.98f;

    float duty = (control + 1.0f) / 2.0f;

    spwm->CCR1_Value = (uint32_t)(duty * spwm->Period);
    spwm->CCR2_Value = spwm->CCR1_Value;

    if (spwm->CCR1_Value >= (uint32_t)spwm->Period) {
        spwm->CCR1_Value = (uint32_t)spwm->Period - 1;
    }

    if (spwm->CCR2_Value >= (uint32_t)spwm->Period) {
        spwm->CCR2_Value = (uint32_t)spwm->Period - 1;
    }
}

void SPWM_ThreePhase_Update(SPWM_t *spwm, float theta, float amplitude)
{
    float phase_a;
    float phase_b;
    float phase_c;
    float duty_a;
    float duty_b;
    float duty_c;
    const float phase_shift = 2.094395102f;  // 2*pi/3

    if (amplitude > 0.95f) amplitude = 0.95f;
    if (amplitude < 0.0f)  amplitude = 0.0f;

    phase_a = arm_sin_f32(theta);
    phase_b = arm_sin_f32(theta - phase_shift);
    phase_c = arm_sin_f32(theta + phase_shift);

    duty_a = 0.5f + 0.5f * amplitude * phase_a;
    duty_b = 0.5f + 0.5f * amplitude * phase_b;
    duty_c = 0.5f + 0.5f * amplitude * phase_c;

    spwm->CCR1_Value = (uint32_t)(duty_a * spwm->Period);
    spwm->CCR2_Value = (uint32_t)(duty_b * spwm->Period);
    spwm->CCR3_Value = (uint32_t)(duty_c * spwm->Period);
}

static float SVPWM_Clamp(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static uint32_t SVPWM_DutyToCCR(float duty, float period)
{
    duty = SVPWM_Clamp(duty, 0.0f, 1.0f);

    if (duty >= 1.0f) {
        return (uint32_t)period - 1U;
    }

    return (uint32_t)(duty * period);
}

void SVPWM_Update(SPWM_t *spwm,
                  float v_alpha, float v_beta, float v_dc)
{
    float va, vb, vc;
    float vmax, vmin;
    float v_offset;
    float half_span;
    float scale;
    float duty_a, duty_b, duty_c;
    const float SQRT3_OVER_2 = 0.8660254f;

    if (v_dc <= 1.0f) {
        spwm->CCR1_Value = (uint32_t)(spwm->Period / 2.0f);
        spwm->CCR2_Value = (uint32_t)(spwm->Period / 2.0f);
        spwm->CCR3_Value = (uint32_t)(spwm->Period / 2.0f);
        return;
    }

    /* Inverse Clarke: alpha/beta voltage command -> three phase command. */
    va = v_alpha;
    vb = -0.5f * v_alpha + SQRT3_OVER_2 * v_beta;
    vc = -0.5f * v_alpha - SQRT3_OVER_2 * v_beta;

    vmax = va;
    if (vb > vmax) vmax = vb;
    if (vc > vmax) vmax = vc;

    vmin = va;
    if (vb < vmin) vmin = vb;
    if (vc < vmin) vmin = vc;

    /* Keep the requested voltage vector inside the SVPWM linear region. */
    half_span = 0.5f * (vmax - vmin);
    if (half_span > 0.5f * v_dc) {
        scale = (0.5f * v_dc) / half_span;
        va *= scale;
        vb *= scale;
        vc *= scale;

        vmax *= scale;
        vmin *= scale;
    }

    /* This common-mode voltage is the SVPWM zero-vector distribution. */
    v_offset = -0.5f * (vmax + vmin);

    duty_a = 0.5f + (va + v_offset) / v_dc;
    duty_b = 0.5f + (vb + v_offset) / v_dc;
    duty_c = 0.5f + (vc + v_offset) / v_dc;

    spwm->CCR1_Value = SVPWM_DutyToCCR(duty_a, spwm->Period);
    spwm->CCR2_Value = SVPWM_DutyToCCR(duty_b, spwm->Period);
    spwm->CCR3_Value = SVPWM_DutyToCCR(duty_c, spwm->Period);
}