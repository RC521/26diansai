/**
 * @file    app_pfc.c
 * @brief   单相桥式 PFC 控制：电压 PI 外环 + QPR 电流内环
 *          调制方式：单极性倍频 (Unipolar Double-Frequency PWM)
 *
 * 控制架构：
 *   Vdc PI 外环 → 电流峰值参考 Ipeak_Ref
 *   Iref = Ipeak_Ref * cos(theta) (与电网电压同相的正弦波)
 *   QPR 电流内环跟踪 Iref
 *   前馈解耦: Ubridge = Ui - Control → 归一化调制波 → PWM
 *
 * 硬件拓扑：单相全桥 (H 桥)
 *   - 腿 A (①②): TIM4 CH1/CH2 互补对, PD12/PD13, 中心对齐 10kHz
 *   - 腿 B (③④): TIM5 CH2/CH3 互补对, PA1/PA2,   中心对齐 10kHz
 *   - 导通序列 (一个工频周期内，以占空比变化循环)：
 *     正半周: ②④ → ①④ → ②④  (V_AB: 0 → +Vdc → 0)
 *     负半周: ①③ → ②③ → ①③  (V_AB: 0 → −Vdc → 0)
 *
 * 单极性倍频原理：
 *   两条桥臂均以 10kHz 中心对齐载波斩波，调制波互为反相：
 *     duty_A = 0.5 + 0.5 * m   (TIM4 CH1/CH2, 腿 ①②)
 *     duty_B = 0.5 − 0.5 * m   (TIM5 CH2/CH3, 腿 ③④)
 *   中心对齐下每个载波周期有两次比较匹配，
 *   V_AB = V_A − V_B 的等效开关纹波频率为 20kHz (倍频)。
 */

#include "app_pfc.h"
#include "app_config.h"
#include "tim.h"
#include "arm_math.h"

/**
 * @brief 数值限幅
 *
 * @param value 输入值
 * @param min   下限
 * @param max   上限
 * @return 限幅后的值，保证在 [min, max] 范围内
 */
static float APP_PFC_Clamp(float value, float min, float max)
{
    if (value > max) {
        return max;
    }
    if (value < min) {
        return min;
    }
    return value;
}

/**
 * @brief 两路桥臂 PWM 均置于中性点 (50% 占空比)
 *
 * 此时每个桥臂上下管各导通 50%，V_AB = 0。
 * 用于 PWM 禁止或故障保护时的安全状态。
 * TIM4 和 TIM5 同时写入。
 */
static void APP_PFC_WriteNeutral(void)
{
    uint32_t period = TIM4->ARR + 1U;
    uint32_t upper_ccr = period / 2U;
    uint32_t lower_ccr = upper_ccr + PFC_SOFTWARE_DEADTIME_COUNTS;

    if (lower_ccr >= period) {
        lower_ccr = period - 1U;
    }

    /* 腿 A: TIM4 CH1/CH2 */
    TIM4->CCR1 = lower_ccr;
    TIM4->CCR2 = upper_ccr;

    /* 腿 B: TIM5 CH2/CH3 */
    TIM5->CCR2 = lower_ccr;
    TIM5->CCR3 = upper_ccr;
}

static void APP_PFC_StopBridgePwm(APP_PFC_t *pfc)
{
    if (pfc->Pwm_Output_Active != 0U) {
        (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
        (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
        (void)HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_2);
        (void)HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
        pfc->Pwm_Output_Active = 0U;
    }
}

static void APP_PFC_StartBridgePwm(APP_PFC_t *pfc)
{
    if (pfc->Pwm_Output_Active == 0U) {
        pfc->Pwm_Output_Active = 1U;
        if ((HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) == HAL_OK) &&
            (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) == HAL_OK) &&
            (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) == HAL_OK) &&
            (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3) == HAL_OK)) {
        } else {
            APP_PFC_StopBridgePwm(pfc);
        }
    }
}

static void APP_PFC_UpdateInputVoltage(APP_PFC_t *pfc, float ui_real)
{
    float absolute_ui = ui_real;

    if (absolute_ui < 0.0f) {
        absolute_ui = -absolute_ui;
    }

    if (absolute_ui > pfc->Ui_Peak) {
        pfc->Ui_Peak = absolute_ui;
    } else {
        pfc->Ui_Peak -= pfc->Ui_Peak * PFC_UI_PEAK_DECAY;
    }

    if (pfc->Ui_Peak >= PFC_PLL_MIN_INPUT_VOLTAGE) {
        if (pfc->Input_Valid_Counter < PFC_INPUT_VALID_SAMPLES) {
            pfc->Input_Valid_Counter++;
        }
        if (pfc->Input_Valid_Counter >= PFC_INPUT_VALID_SAMPLES) {
            pfc->Input_Voltage_Valid = 1U;
        }
    } else {
        pfc->Input_Voltage_Valid = 0U;
        pfc->Input_Valid_Counter = 0U;
    }

    pfc->Debug_Ui_Peak = pfc->Ui_Peak;
}

/**
 * @brief 单极性倍频调制 PWM 输出
 *
 * 两条桥臂均以高频斩波，调制波互为反相：
 *   - 腿 A (TIM4, ①②): duty_upper = 0.5 + 0.5 * m
 *   - 腿 B (TIM5, ③④): duty_upper = 0.5 − 0.5 * m
 *
 * 等效效果：
 *   m =  0.0 → duty_A = 50%, duty_B = 50% → V_AB = 0
 *   m = +0.5 → duty_A = 75%, duty_B = 25% → V_AB = +0.5*Vdc
 *   m = +1.0 → duty_A = 100%, duty_B = 0%  → V_AB = +Vdc
 *   m = −0.5 → duty_A = 25%, duty_B = 75% → V_AB = −0.5*Vdc
 *   m = −1.0 → duty_A = 0%, duty_B = 100%  → V_AB = −Vdc
 *
 * 各定时器的两个通道为 PWM1/PWM2 互补对，
 * 等值 CCR 产生一相桥臂的上下管驱动。
 *
 * @param modulation 调制波，范围 [-PFC_MAX_MODULATION, PFC_MAX_MODULATION]
 * @param duty_a     输出: 腿 A 上管占空比 (调试用)
 * @param duty_b     输出: 腿 B 上管占空比 (调试用)
 */
static void APP_PFC_WriteBridge(float  modulation,
                                float *duty_a,
                                float *duty_b)
{
    uint32_t period = TIM4->ARR + 1U;
    uint32_t ccr_a;
    uint32_t ccr_b;
    uint32_t lower_ccr_a;
    uint32_t lower_ccr_b;
    uint32_t deadtime = PFC_SOFTWARE_DEADTIME_COUNTS;

    if (deadtime >= period) {
        deadtime = period - 1U;
    }

    /* 限幅防过调制 */
    modulation = APP_PFC_Clamp(modulation,
                               -PFC_MAX_MODULATION,
                                PFC_MAX_MODULATION);

    /* 调制波 → 两路占空比 (0~1)，互为反相 */
    *duty_a = 0.5f + 0.5f * modulation;
    *duty_b = 0.5f - 0.5f * modulation;

    /* 占空比 → CCR 值 */
    ccr_a = (uint32_t)(*duty_a * (float)period);
    ccr_b = (uint32_t)(*duty_b * (float)period);

    /* Keep space for the PWM2 compare point after each PWM1 transition. */
    if (ccr_a >= (period - deadtime)) {
        ccr_a = period - deadtime - 1U;
    }
    if (ccr_b >= (period - deadtime)) {
        ccr_b = period - deadtime - 1U;
    }

    lower_ccr_a = ccr_a + deadtime;
    lower_ccr_b = ccr_b + deadtime;

    /* TIM4 CH1/CH2 互补对 → 腿 A (①②)
     * TIM5 CH2/CH3 互补对 → 腿 B (③④) */
    TIM4->CCR1 = lower_ccr_a;
    TIM4->CCR2 = ccr_a;
    TIM5->CCR2 = lower_ccr_b;
    TIM5->CCR3 = ccr_b;
}

/**
 * @brief 复位 PFC 控制器状态
 *
 * 清零外环计数器、PI 积分器/输出、PR 谐振状态，
 * 以及所有调试变量。在 PWM 禁止或母线欠压时调用，
 * 防止重新使能时积分器累积导致冲击。
 *
 * @param pfc PFC 控制器指针
 */
static void APP_PFC_Reset(APP_PFC_t *pfc)
{
    pfc->Outer_Loop_Counter = 0U;
    pfc->Vdc_PI.Integral    = 0.0f;
    pfc->Vdc_PI.Output      = 0.0f;
    PR_Reset(&pfc->Current_PR);

    pfc->Debug_Ipeak_Ref       = 0.0f;
    pfc->Debug_Iref            = 0.0f;
    pfc->Debug_Current_Control = 0.0f;
    pfc->Debug_Bridge_Command  = 0.0f;
    pfc->Debug_Modulation      = 0.0f;
    pfc->Debug_DutyA           = 0.5f;
    pfc->Debug_DutyB           = 0.5f;
}

/**
 * @brief 初始化 PFC 控制器
 *
 * 依次初始化 PLL、电压外环 PI 和 QPR 电流内环。
 * PWM 初始为禁止状态，两路桥臂均输出中性点。
 *
 * @param pfc PFC 控制器指针
 */
void APP_PFC_Init(APP_PFC_t *pfc)
{
    /* 初始化锁相环 */
    /* 电压外环 PI: 分频执行，输出为电流峰值参考，
     * 限幅 ±PFC_ID_REF_LIMIT */
    PI_Init(&pfc->Vdc_PI, PFC_VDC_KP, PFC_VDC_KI,
            CONTROL_SAMPLE_TIME * PFC_OUTER_LOOP_DIVIDER,
            PFC_ID_REF_LIMIT, 0.0f);

    /* QPR 电流内环: 准比例谐振控制器，
     * 在电网基频附近提供高增益，实现无静差跟踪正弦参考 */
    PR_Init(&pfc->Current_PR,
            PFC_PR_KP, PFC_PR_KR,
            PFC_GRID_NOMINAL_FREQ_HZ, PFC_PR_BANDWIDTH_HZ,
            CONTROL_SAMPLE_TIME,
            1.0f, -1.0f);

    /* 初始状态: PWM 禁止，安全输出 */
    pfc->Pwm_Enabled = 0U;
    pfc->OverVoltage_Fault = 0U;
    pfc->Pwm_Output_Active = 0U;
    pfc->Pll_Locked = 0U;
    pfc->Pll_Lock_Counter = 0U;
    pfc->Input_Voltage_Valid = 0U;
    pfc->Input_Valid_Counter = 0U;
    pfc->Startup_Vdc_Ready = 0U;
    pfc->Ui_Peak = 0.0f;
    APP_PFC_Reset(pfc);
    APP_PFC_WriteNeutral();
}

/**
 * @brief 设置 PWM 使能/禁止
 *
 * - 使能: 置标志位，下一控制周期开始输出调制波
 * - 禁止: 立即复位控制器状态并将两路桥臂置于中性点
 *
 * @param pfc    PFC 控制器指针
 * @param enable 0 = 禁止, 非 0 = 使能
 */
void APP_PFC_SetPwmEnable(APP_PFC_t *pfc, uint8_t enable)
{
    if (pfc->OverVoltage_Fault != 0U) {
        pfc->Pwm_Enabled = 0U;
        return;
    }

    pfc->Pwm_Enabled = (enable != 0U) ? 1U : 0U;

    if (pfc->Pwm_Enabled == 0U) {
        pfc->Startup_Vdc_Ready = 0U;
        /* 禁止时复位，防止下次使能时积分器累积冲击 */
        APP_PFC_Reset(pfc);
        APP_PFC_WriteNeutral();
        APP_PFC_StopBridgePwm(pfc);
    }
}

void APP_PFC_CheckOverVoltage(APP_PFC_t *pfc, float vdc_real)
{
    if ((pfc->OverVoltage_Fault == 0U) && (vdc_real >= PFC_VDC_OVP)) {
        pfc->OverVoltage_Fault = 1U;
        pfc->Pwm_Enabled = 0U;
        APP_PFC_Reset(pfc);
        APP_PFC_StopBridgePwm(pfc);
    }
}

/**
 * @brief PFC 控制主更新函数 (每个控制周期调用一次)
 *
 * 控制流程 (单极性倍频):
 *   1. PLL 锁相 → theta
 *   2. 欠压保护检查
 *   3. 电压外环 PI (分频) → Ipeak_Ref (电流峰值参考)
 *   4. 瞬时电流参考: Iref = Ipeak_Ref * cos(theta)
 *      注意: PLL 锁相后 theta 超前电网电压约 +90°，
 *      因此 -cos(theta) 与电网电压同相。
 *   5. QPR 电流内环 → Current_Control
 *   6. 前馈解耦: Bridge_Command = Ui - Current_Control
 *      (由 L*di/dt = Ui - Ubridge 推导)
 *   7. 归一化调制波 m = Bridge_Command / Vdc
 *   8. 倍频 PWM:
 *        duty_A = 0.5 + 0.5*m → TIM4 CH2/CH1 (腿 ①②)
 *        duty_B = 0.5 − 0.5*m → TIM5 CH3/CH2 (腿 ③④)
 *
 * @param pfc      PFC 控制器指针
 * @param ui_real  电网电压瞬时采样值
 * @param ii_real  电网电流瞬时采样值
 * @param vdc_real 直流母线电压采样值
 */
void APP_PFC_Update(APP_PFC_t *pfc,
                    float ui_real, float ii_real, float vdc_real)
{
#if PFC_PWM_TEST_ENABLE == 1
    (void)pfc;
    (void)ui_real;
    (void)ii_real;
    (void)vdc_real;
    return;
#endif
    float qpr_limit;

    /* ---- 第 1 步: PLL 锁相 ---- */
    /* 更新调试变量 */
    pfc->Debug_Ui  = ui_real;
    pfc->Debug_Ii  = ii_real;
    pfc->Debug_Vdc = vdc_real;
    APP_PFC_UpdateInputVoltage(pfc, ui_real);

    APP_PFC_CheckOverVoltage(pfc, vdc_real);
    if (pfc->OverVoltage_Fault != 0U) {
        return;
    }

    /* TODO: 调试完成后恢复 PLL 锁检查 */
    if (pfc->Input_Voltage_Valid == 0U) {
    //if (pfc->Pll_Locked == 0U) {
        APP_PFC_Reset(pfc);
        APP_PFC_WriteNeutral();
        APP_PFC_StopBridgePwm(pfc);
        return;
    }

    /* ---- 第 2 步: 欠压保护 ---- */
    if (pfc->Startup_Vdc_Ready == 0U) {
        if (vdc_real >= PFC_MIN_VDC) {
            pfc->Startup_Vdc_Ready = 1U;
        }
    } else if (vdc_real < PFC_MIN_VDC) {
        APP_PFC_Reset(pfc);
        APP_PFC_WriteNeutral();
        APP_PFC_StopBridgePwm(pfc);
        return;
    }

    /* ---- 第 3 步: 电压外环 PI (分频执行) ---- */
#if PFC_SIMULATION_ENABLE == 1
    pfc->Debug_Ipeak_Ref = PFC_SIM_IREF_PEAK;
#else
    if (pfc->Outer_Loop_Counter == 0U) {
        /* 外环产生电流峰值参考，正值 = 整流 (功率从交流侧流向直流侧) */
        pfc->Debug_Ipeak_Ref = PI_Update(&pfc->Vdc_PI,
                                          PFC_VDC_REF,
                                          vdc_real);
    }
    pfc->Outer_Loop_Counter++;
    if (pfc->Outer_Loop_Counter >= PFC_OUTER_LOOP_DIVIDER) {
        pfc->Outer_Loop_Counter = 0U;
    }
#endif

    /* ---- 第 4 步: 瞬时电流参考生成 ----
     * PLL 的 theta 锁定后超前电网电压约 +90° (取决于 PLL 实现)，
     * 因此用 -cos(theta) 得到与 Ui 同相的正弦波。
     * 如果 ADC 传感器物理极性相反，只需修改 PFC_CURRENT_REFERENCE_SIGN。 */
    pfc->Debug_Iref = PFC_CURRENT_REFERENCE_SIGN *
                       pfc->Debug_Ipeak_Ref *
                       (ui_real / pfc->Ui_Peak);

    /* ---- 第 5 步: QPR 电流内环 ----
     * 动态限幅: PR 输出限幅与母线电压成正比 (前馈补偿) */
    qpr_limit = PFC_MAX_MODULATION * vdc_real;
    pfc->Current_PR.Max_Output =  qpr_limit;
    pfc->Current_PR.Min_Output = -qpr_limit;
    pfc->Debug_Current_Control = PR_Update(&pfc->Current_PR,
                                            pfc->Debug_Iref,
                                            ii_real);

    /* ---- 第 6 步: 前馈解耦 ----
     * 电感方程: L * di/dt = Ui - Ubridge
     * 因此: Ubridge = Ui - (PR 控制输出)
     * PFC_CURRENT_CONTROL_SIGN 处理电流方向符号 */
    pfc->Debug_Bridge_Command = ui_real -
                                PFC_CURRENT_CONTROL_SIGN *
                                pfc->Debug_Current_Control;

    /* ---- 第 7 步: 归一化调制波 m ----
     * m = Bridge_Command / Vdc，经极性修正和限幅 */
    pfc->Debug_Modulation = PFC_BRIDGE_POLARITY *
                            (pfc->Debug_Bridge_Command / vdc_real);
    pfc->Debug_Modulation = APP_PFC_Clamp(pfc->Debug_Modulation,
                                          -PFC_MAX_MODULATION,
                                           PFC_MAX_MODULATION);

    /* ---- 第 8 步: 单极性倍频 PWM 输出 ----
     * duty_A = 0.5 + 0.5*m → TIM4 CH1/CH2 (腿 ①②)
     * duty_B = 0.5 − 0.5*m → TIM5 CH2/CH3 (腿 ③④)
     * 两路调制波互为反相，中心对齐下等效开关频率 20kHz */
    if (pfc->Pwm_Enabled != 0U) {
        APP_PFC_StartBridgePwm(pfc);
        APP_PFC_WriteBridge(pfc->Debug_Modulation,
                            &pfc->Debug_DutyA,
                            &pfc->Debug_DutyB);
    } else {
        pfc->Debug_DutyA = 0.5f;
        pfc->Debug_DutyB = 0.5f;
        APP_PFC_WriteNeutral();
        APP_PFC_StopBridgePwm(pfc);
    }
}
