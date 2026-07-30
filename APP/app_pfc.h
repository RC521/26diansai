#ifndef APP_PFC_H
#define APP_PFC_H

#include "app_pll.h"

/**
 * @brief 单相桥式 PFC 控制器结构体
 *
 * 控制架构：
 *   直流母线电压 PI 外环 → 输入电流峰值参考 → QPR 电流内环
 *
 * 硬件拓扑：单相全桥 (H 桥)，单极性倍频调制
 *   - 腿 A (①②): TIM4 CH1/CH2 互补对，PD12/PD13，中心对齐 10kHz
 *   - 腿 B (③④): TIM5 CH2/CH3 互补对，PA1/PA2，  中心对齐 10kHz
 *   - 两路调制波互为反相 (m 和 −m)，等效输出纹波频率 20kHz
 *   - 导通序列 (一个工频周期)：
 *     正半周: ②④ → ①④ → ②④ (V_AB = 0 → +Vdc → 0)
 *     负半周: ①③ → ②③ → ①③ (V_AB = 0 → −Vdc → 0)
 */
typedef struct {
    APP_PLL_t       Pll;                /**< 锁相环，跟踪电网电压相位 */
    PI_Controller_t Vdc_PI;             /**< 电压外环 PI 控制器，调节直流母线电压 */
    PR_Controller_t Current_PR;         /**< QPR 电流内环控制器，跟踪正弦电流参考 */
    uint32_t        Outer_Loop_Counter; /**< 外环分频计数器 (每 N 个周期执行一次电压环) */
    uint8_t         Pwm_Enabled;        /**< PWM 使能标志: 0 = 禁止, 1 = 使能 */
    uint8_t         OverVoltage_Fault;
    uint8_t         Pwm_Output_Active;
    uint8_t         Pll_Locked;
    uint32_t        Pll_Lock_Counter;
    uint8_t         Input_Voltage_Valid;
    uint32_t        Input_Valid_Counter;
    uint8_t         Startup_Vdc_Ready;
    float           Ui_Peak;

    /* 调试/监测变量 */
    volatile float  Debug_Ui;               /**< 电网电压瞬时值 */
    volatile float  Debug_Ui_Peak;
    volatile float  Debug_Ii;               /**< 电网电流瞬时值 */
    volatile float  Debug_Vdc;              /**< 直流母线电压 */
    volatile float  Debug_Ipeak_Ref;        /**< 电流峰值参考 (电压环 PI 输出) */
    volatile float  Debug_Iref;             /**< 瞬时电流参考 = Ipeak_Ref * cos(theta) */
    volatile float  Debug_Current_Control;  /**< QPR 电流环控制输出 */
    volatile float  Debug_Bridge_Command;   /**< 桥臂电压指令 (前馈解耦后) */
    volatile float  Debug_Modulation;       /**< 调制波 (归一化到 [-1, 1]) */
    volatile float  Debug_DutyA;            /**< 腿 A (TIM4) 上管占空比 */
    volatile float  Debug_DutyB;            /**< 腿 B (TIM5) 上管占空比 */
} APP_PFC_t;

/**
 * @brief 初始化 PFC 控制器
 *
 * 初始化 PLL、电压外环 PI 和 QPR 电流内环，
 * PWM 初始为禁止状态，两路桥臂均输出中性点 (50% 占空比)。
 *
 * @param pfc PFC 控制器指针
 */
void APP_PFC_Init(APP_PFC_t *pfc);

/**
 * @brief 设置 PWM 使能/禁止
 *
 * 禁止时自动复位 PI/PR 积分器，两路桥臂回到中性点，
 * 防止重新使能时产生电流冲击。
 *
 * @param pfc    PFC 控制器指针
 * @param enable 0 = 禁止 PWM, 非 0 = 使能 PWM
 */
void APP_PFC_SetPwmEnable(APP_PFC_t *pfc, uint8_t enable);
void APP_PFC_CheckOverVoltage(APP_PFC_t *pfc, float vdc_real);

/**
 * @brief PFC 控制主更新函数 (每个控制周期调用一次)
 *
 * 控制流程 (单极性倍频)：
 *   1. PLL 锁相 → theta
 *   2. 电压外环 PI (分频) → 电流峰值参考 Ipeak_Ref
 *   3. 瞬时电流参考 Iref = Ipeak_Ref * cos(theta)
 *   4. QPR 电流内环 → 控制输出
 *   5. 前馈解耦 + 归一化 → 调制波 m (范围 [-1, 1])
 *   6. 两路桥臂同时写入:
 *        duty_A = 0.5 + 0.5 * m  (TIM4)
 *        duty_B = 0.5 − 0.5 * m  (TIM5)
 *      两者互为反相，中心对齐下等效开关频率 20kHz
 *
 * @param pfc      PFC 控制器指针
 * @param ui_real  电网电压瞬时采样值
 * @param ii_real  电网电流瞬时采样值
 * @param vdc_real 直流母线电压采样值
 */
void APP_PFC_Update(APP_PFC_t *pfc,
                    float ui_real, float ii_real, float vdc_real);

#endif
