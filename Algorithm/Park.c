#include "park.h"

/**
 * @brief  初始化 Park 结构体
 * @param  park: 结构体指针
 */
void Park_Init(Park_t *park) {
    park->Alpha = 0.0f;
    park->Beta  = 0.0f;
    park->Theta = 0.0f;
    park->D     = 0.0f;
    park->Q     = 0.0f;
}

/**
 * @brief  执行 Park 变换 (放在定时器中断里，SOGI 算完之后调用)
 * @param  park:  结构体指针
 * @param  alpha: 输入的 Alpha 轴电压/电流
 * @param  beta:  输入的 Beta 轴电压/电流
 * @param  theta: 当前电网的相位角度 (由 PLL 提供)
 */
void Park_Update(Park_t *park, float alpha, float beta, float theta) {
    float sin_val, cos_val;
    
    // 1. 更新结构体内部的输入状态
    park->Alpha = alpha;
    park->Beta  = beta;
    park->Theta = theta;
    
    // 2. 调用 DSP 库极速算出当前 Theta 的正弦和余弦
    // 这一步比用 math.h 的 sin() 和 cos() 快几十倍
    arm_sin_cos_f32(park->Theta, &sin_val, &cos_val);
    
    // 3. 核心数学计算公式
    // 计算 D 轴分量
    park->D = (park->Alpha * cos_val) + (park->Beta * sin_val);
    
    // 计算 Q 轴分量 (如果是做单相 PLL 锁相，这个 Q 就是我们要消灭的误差)
    park->Q = -(park->Alpha * sin_val) + (park->Beta * cos_val);
}

/**
注册示例:
Park_t my_park;
Park_Init(&my_park);
调用示例：
在定时器中断里，我们将SOGI算出来的V_alpha,V_beta,以及我们预测的现在的θ一并传入，
Park_Update(&my_park, V_alpha, V_beta, current_theta);
算出误差Q
float speed_compensate = PI_Update(&my_pi, 0.0f, my_park.Q);
speed_compensate:角速度补偿量
 */
