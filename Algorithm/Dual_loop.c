
#include "dual_loop.h"


/*
 * 函数功能：初始化双环控制器
 * 纯软件思维：在这里给两个 PI 实例赋予不同的“限幅属性”，决定了系统的物理边界。
 */
void DualLoop_Init(DualLoop_t *dual_loop, float sample_time) {
    
    // 1. 初始化电压外环
    // 输入是电压误差，输出是“目标电流”
    //限幅意义：保护硬件！最高输出 5.0A 电流。如果电压跌得再惨，系统也绝对不会向硬件索取超过 5A 的电流。
    // (Kp, Ki 参数需后续根据真实硬件板子整定，这里填入示例值)
    PI_Init(&(dual_loop->Voltage_PI), 0.5f, 0.01f, sample_time, 5.0f, -5.0f); 

    // 2. 初始化电流内环
    // 输入是电流误差，输出是“调制比幅度 A”
    // 限幅意义：SPWM 的调制比物理极限就是 0.0 到 1.0。为了给硬件死区留点余量，通常上限锁死在 0.95。
    PI_Init(&(dual_loop->Current_PI), 0.2f, 0.05f, sample_time, 0.95f, 0.0f); 
}
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
