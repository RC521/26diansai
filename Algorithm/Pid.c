#include "pid.h"

/**
 * @brief  初始化 PI 控制器参数
 * @param  pi:   你要初始化的那个结构体指针
 * @param  kp:   比例系数
 * @param  ki:   积分系数
 * @param  max:  输出最大值 (例如如果是占空比，最大可能是 100 或 8000)
 * @param  min:  输出最小值 (如果是纯交流，可能有负数；如果是直流，通常是 0)
 */
void PI_Init(PI_Controller_t *pi, float kp, float ki, float max, float min) {
    pi->Kp = kp;
    pi->Ki = ki;
    pi->Integral = 0.0f; // 初始状态下，历史累加误差为 0
    pi->Out_Max = max;
    pi->Out_Min = min;
    pi->Output = 0.0f;
}

/**
 * @brief  执行一次 PI 闭环计算 (放在定时器中断里循环调用)
 * @param  pi:     结构体指针
 * @param  target: 你的目标值 (比如你想输出 5V)
 * @param  actual: 实际的反馈值 (比如 ADC 当前读回来的电压)
 * @retval float:  计算后的控制输出量
 */
float PI_Update(PI_Controller_t *pi, float target, float actual) {
    // 1. 计算当前的瞬时误差
    float error = target - actual;
    
    // 2. 累加历史误差 (I 项的核心)
    pi->Integral += error;
    
    // 3. 积分限幅防饱和 (极其重要！)
    // 如果累加得太大，强制限制住，防止下次想减小的时候减不下来
    // 注意：这里的限幅范围通常和输出限幅保持一致，或者稍微小一点
    if (pi->Integral > pi->Out_Max) {
        pi->Integral = pi->Out_Max;
    } else if (pi->Integral < pi->Out_Min) {
        pi->Integral = pi->Out_Min;
    }
    
    // 4. 核心计算公式：比例 + 积分
    pi->Output = (pi->Kp * error) + (pi->Ki * pi->Integral);
    
    // 5. 整体输出限幅保护
    if (pi->Output > pi->Out_Max) {
        pi->Output = pi->Out_Max;
    } else if (pi->Output < pi->Out_Min) {
        pi->Output = pi->Out_Min;
    }
    
    return pi->Output;
}


/**
注册示例：
PI_Controller_t Voltage_PI_1; 
PI_Init(&Voltage_PI_1, 2.5f, 0.1f, 100.0f, 0.0f);
 */
