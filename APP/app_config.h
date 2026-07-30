#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/* 当前使用模式: 1=模式A, 2=模式B */
#define USE_MODE 2
/* 模式A: 三相闭环控制 */
#define USE_MODE_A 1
/* 模式B: 单相PFC控制 */
#define USE_MODE_B 2

/* PLL锁相环使能 (0=禁用, 1=使能) */
#define PLL_USE 0
/* SPWM调制使能 (0=禁用, 1=使能) */
#define SPWM_USE 1
/* SVPWM调制使能 (0=禁用, 1=使能) */
#define SVPWM_USE 0
/* 三相开环控制使能 (0=禁用, 1=使能) */
#define KAI_HUAN_SAN_XIANG 0
/* 三相闭环控制使能 (0=禁用, 1=使能) */
#define BI_HUAN_SAN_XIANG 0
/* PFC控制使能 (0=禁用, 1=使能) */
#define PFC_USE 1
/* PFC PWM独立输出使能 (0=禁用, 1=使能) */
#define PFC_PWM_ENABLE 1
#define PFC_PWM_TEST_ENABLE 0
/* Software-only PFC input test. Keep PFC_PWM_ENABLE at 0 while enabled. */
#define PFC_SIMULATION_ENABLE 0
#define PFC_SIM_GRID_FREQ_HZ 50.0f
#define PFC_SIM_UI_PEAK 18.0f
#define PFC_SIM_IREF_PEAK 0.9f
#define PFC_SIM_VDC 60.0f
#define PFC_SIM_PRINT_DIVIDER 100U
/* TIM2/TIM6 control update period: 20 kHz. */
#define CONTROL_SAMPLE_TIME 0.00005f

/* 单相有源整流器 PFC 初始参数。 */
/* PFC目标直流母线电压参考值 */
#define PFC_VDC_REF              30.0f
/* PFC允许的最低直流母线电压 */
#define PFC_MIN_VDC              15.0f
#define PFC_VDC_OVP              35.0f
#define PFC_PLL_MIN_INPUT_VOLTAGE 5.0f
#define PFC_PLL_FREQ_TOLERANCE_HZ 2.0f
#define PFC_PLL_Q_ERROR_RATIO     0.15f
#define PFC_PLL_LOCK_SAMPLES      400U
#define PFC_INPUT_VALID_SAMPLES    400U
#define PFC_UI_PEAK_DECAY          0.00002f
/* 外环控制分频系数 */
#define PFC_OUTER_LOOP_DIVIDER   20U
/* PFC电流参考限幅 */
#define PFC_ID_REF_LIMIT         0.5f  //峰值
/* 直流母线电压环比例系数 */
#define PFC_VDC_KP               0.10f
/* 直流母线电压环积分系数 */
#define PFC_VDC_KI               2.0f
/* 比例谐振(PR)控制器比例系数 */
#define PFC_PR_KP                 2.0f
/* 比例谐振(PR)控制器谐振系数 */
#define PFC_PR_KR                 50.0f
/* 比例谐振(PR)控制器带宽(Hz) */
#define PFC_PR_BANDWIDTH_HZ       5.0f
/* 电网标称频率(Hz) */
#define PFC_GRID_NOMINAL_FREQ_HZ  50.0f
/* 电流参考方向符号 (+1.0或-1.0) */
#define PFC_CURRENT_REFERENCE_SIGN (1.0f)
/* PWM最大调制比 */
#define PFC_MAX_MODULATION       0.90f
/* TIM4/TIM5 timer tick is 2 MHz, so one count is 0.5 us. */
#define PFC_SOFTWARE_DEADTIME_COUNTS 1U
/* 电流控制符号方向 ，电流内环的负反馈方向开关*/
#define PFC_CURRENT_CONTROL_SIGN 1.0f
/* 桥臂极性设置 ，正调制比 m，到底让全桥输出正电压还是负电压*/
#define PFC_BRIDGE_POLARITY      1.0f
/* 电流偏置自动校准使能 (0=禁用, 1=使能) */
#define PFC_AUTO_OFFSET_CAL_ENABLE 1
/* 偏置校准采样点数 */
#define PFC_OFFSET_CAL_SAMPLES   4000U
/* 电压电流低通滤波器截止频率(Hz) */
#define PFC_UI_LPF_CUTOFF_HZ     2000.0f
/* 输入电流低通滤波器截止频率(Hz) */
#define PFC_II_LPF_CUTOFF_HZ     2000.0f
/* 直流母线电压低通滤波器截止频率(Hz) */
#define PFC_VDC_LPF_CUTOFF_HZ    20.0f




/* 三相 dq 闭环控制初始参数，建议在实机调试时再微调。 */
/* 电压环控制分频系数 */
#define DQ_VOLTAGE_LOOP_DIVIDER 20U
/* d轴电压参考值 */
#define DQ_VD_REF               3.0f
/* q轴电压参考值 */
#define DQ_VQ_REF               0.0f
/* 电流参考限幅 */
#define DQ_CURRENT_REF_LIMIT    0.50f
/* 电压环比例系数 */
#define DQ_VOLTAGE_KP           0.05f
/* 电压环积分系数 */
#define DQ_VOLTAGE_KI           2.0f
/* 电流环比例系数 */
#define DQ_CURRENT_KP           2.0f
/* 电流环积分系数 */
#define DQ_CURRENT_KI           200.0f
/* 直流母线最低电压保护阈值 */
#define DQ_MIN_VDC              2.0f
/* 最大电压比限制 */
#define DQ_MAX_VOLTAGE_RATIO    0.45f

#endif
