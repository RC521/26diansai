#include "sogi.h"

/**
 * @brief  初始化 SOGI 参数
 * @param  sogi: 结构体指针
 * @param  k:    阻尼系数 (推荐 1.414)
 * @param  ts:   中断控制周期 (如 0.0001)
 */
void SOGI_Init(SOGI_t *sogi, float k, float ts) {
    sogi->k = k;
    sogi->Ts = ts;
    
    // 初始状态下，电压都为 0
    sogi->alpha = 0.0f;
    sogi->beta  = 0.0f;
}

/**
 * @brief  执行一次 SOGI 迭代 (放在定时器中断里循环调用)
 * @param  sogi:   结构体指针
 * @param  v_grid: 当前 ADC 采到的真实电网电压 (需减去 2048 偏置后的交流值)
 * @param  omega:  当前的角速度 (由 PI 控制器加 50Hz 基准算出来的那个值)
 */
void SOGI_Update(SOGI_t *sogi, float v_grid, float omega) {
    // 1. 计算输入误差：真实电压与我们生成的 alpha 之间的差值
    float error = v_grid - sogi->alpha;
    
    // 2. 计算 Alpha 通道的积分变化率 (导数)
    // 公式: (k * omega * error) - (omega * beta)
    float alpha_dot = (sogi->k * omega * error) - (omega * sogi->beta);
    
    // 3. 计算 Beta 通道的积分变化率 (导数)
    // 公式: omega * alpha
    float beta_dot = omega * sogi->alpha;
    
    // 4. 纯软件的核心魔法：对变化率进行积分 (当前值 = 历史值 + 变化率 * 时间)
    sogi->alpha += (alpha_dot * sogi->Ts);
    sogi->beta  += (beta_dot  * sogi->Ts);
}

/**
注册示例:
SOGI_t my_sogi;
// 参数1: 结构体地址
// 参数2: 阻尼系数 k (通常雷打不动写 1.414f，即根号2)
// 参数3: 中断周期 ts (10ms = 0.01秒)//太慢了，一般写10KHz
SOGI_Init(&my_sogi, 1.414f, 0.01f);
调用示例：
在定时器中断里，我们将SOGI算出来的V_alpha,V_beta,以及我们预测的现在的θ一并传入，
Park_Update(&my_park, V_alpha, V_beta, current_theta);
算出误差Q
float speed_compensate = PI_Update(&my_pi, 0.0f, my_park.Q);
speed_compensate:角速度补偿量
 */

