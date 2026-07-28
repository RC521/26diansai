#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#define USE_MODE 2
#define USE_MODE_A 1
#define USE_MODE_B 2

#define PLL_USE 0
#define SPWM_USE 0
#define SVPWM_USE 1
#define KAI_HUAN_SAN_XIANG 0
#define BI_HUAN_SAN_XIANG 0
/* TIM2/TIM6 control update period: 20 kHz. */
#define CONTROL_SAMPLE_TIME 0.00005f

/* Three-phase dq closed-loop initial settings. Tune these on the real plant. */
#define DQ_VOLTAGE_LOOP_DIVIDER 20U
#define DQ_VD_REF               3.0f
#define DQ_VQ_REF               0.0f
#define DQ_CURRENT_REF_LIMIT    0.50f
#define DQ_VOLTAGE_KP           0.05f
#define DQ_VOLTAGE_KI           2.0f
#define DQ_CURRENT_KP           2.0f
#define DQ_CURRENT_KI           200.0f
#define DQ_MIN_VDC              2.0f
#define DQ_MAX_VOLTAGE_RATIO    0.45f

#endif
