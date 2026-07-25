#include "dual_loop.h"

/*
 * 函数功能：双环流水线更新（在 DMA/定时器中断里高频调用）
 * 输入参数：预期的目标电压、ADC 采回的真实电压、ADC 采回的真实电流
 * 返回值：  最新的调制比幅度（用来直接喂给 SPWM 公式）
 */
float DualLoop_Update(DualLoop_t *dual_loop, float target_voltage, float actual_voltage, float actual_current) {
    
    // ==========================================
    // 步骤 1：外环工作（大局观调控）
    // ==========================================
    // 直接调用基类方法，传入外环的“目标（Target）”和“现实（Actual）”
    // PI 内部会自动计算电压误差，并输出系统现在需要多少安培的电流
    float target_current = PI_Update(&(dual_loop->Voltage_PI), target_voltage, actual_voltage);

    // ==========================================
    // 步骤 2：内环工作（极速响应）
    // ==========================================
    // 继续调用基类方法，把外环刚刚算出的 target_current 作为内环的目标！
    // 传入内环的“目标（Target）”和“现实（Actual）”
    // PI 内部会自动计算电流误差，并算出 SPWM 需要多大的幅度才能挤出这些电流
    float modulation_amplitude = PI_Update(&(dual_loop->Current_PI), target_current, actual_current);

    // ==========================================
    // 步骤 3：输出结果
    // ==========================================
    // 将算好的幅度扔回给系统，准备去乘以 sin(theta)
    return modulation_amplitude;
}