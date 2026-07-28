#include "Algorithm.h"
#include "app_pll.h"
#include "main.h"


/**
 * @brief  初始化整个 PLL 系统
 * @param  pll:         系统结构体指针
 * @param  sample_time: 中断周期 (如 10kHz 就是 0.0001)
 */
void APP_PLL_Init(APP_PLL_t *pll, float sample_time) {
    pll->dt = sample_time;
    pll->theta = 0.0f;
    
    // 1. 初始化 SOGI (阻尼系数 1.414, 传入周期)
    SOGI_Init(&(pll->sogi), 1.414f, pll->dt);
    
    // 2. 初始化 Park 变换
    Park_Init(&(pll->park));
    
    // 3. 初始化 PI 控制器 
    // 参数 (Kp=0.5, Ki=10.0) 需要你在实车上根据 50Hz 的追踪效果微调
    // 限幅设为 50.0，意思是允许电网频率最多偏移这么多的角速度
    PI_Init(&(pll->pi), 0.5f, 10.0f, pll->dt, 50.0f, -50.0f); 
}

/**
 * @brief  PLL 核心流转函数 (必须在定时器中断里严格按周期调用)
 * @param  pll:         系统结构体指针
 * @param  v_grid_real: 当前 ADC 采到的真实交流电压 (去掉了 2048 直流偏置的值)
 */
void APP_PLL_Update(APP_PLL_t *pll, float v_grid_real) {
    // 1. 基准角速度
    float base_omega = 2.0f * PI_VALUE * GRID_BASE_FREQ;  //2pif
    
    // 2. SOGI 造波：传入真实电压和上一轮的角速度
    // 如果是第一次运行，current_omega 还没算出来，可以默认用 base_omega
    float temp_omega = (pll->current_omega == 0.0f) ? base_omega : pll->current_omega;  //这段话的逻辑是一个判断语句，如果current_omega=0，那么temp_omega=base_omega
    SOGI_Update(&(pll->sogi), v_grid_real, temp_omega);
    
    // 3. Park 变换：用 SOGI 算出的 Alpha/Beta 和上一轮的 Theta 算误差
    Park_Update(&(pll->park), pll->sogi.alpha, pll->sogi.beta, pll->theta);
    
    // 4. PI 控制器消除误差：目标是让 Park 出来的 Q 轴电压为 0
    // Q 轴就是误差，输入给 PI 的 error 就是 0 - Q
    pll->delta_omega = PI_Update(&(pll->pi), 0.0f, pll->park.Q);
    
    // 5. VCO 压控振荡 (更新速度并积分成角度)
    pll->current_omega = base_omega + pll->delta_omega;
    pll->theta += pll->current_omega * pll->dt;
    
    // 6. 角度限幅 (保证在 0 到 2π 之间一直转圈)
    if (pll->theta > (2.0f * PI_VALUE)) {
        pll->theta -= (2.0f * PI_VALUE);
    } else if (pll->theta < 0.0f) {
        pll->theta += (2.0f * PI_VALUE);
    }
    
    // 顺手算一下真实的物理频率，方便 Debug
    pll->real_freq_hz = pll->current_omega / (2.0f * PI_VALUE);
}


//瞬时电压闭环
void Control_Update_A(void)
{

}


//有效值/幅值闭环
void Control_Update_B(void)
{

}
