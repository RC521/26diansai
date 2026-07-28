#include "app_ThreePhase.h"
#include "app_config.h"
#include "tim.h"
#include "arm_math.h"

#define THREE_PHASE_FREQ_HZ 50.0f
#define DQ_THETA_OFFSET (-0.5f * PI_VALUE)
#define THREE_PHASE_STEP \
    (2.0f * PI_VALUE * THREE_PHASE_FREQ_HZ * CONTROL_SAMPLE_TIME)

static void APP_ThreePhase_WritePwm(const APP_ThreePhase_t *control)
{
    TIM3->CCR1 = control->Pwm->CCR1_Value;
    TIM3->CCR2 = control->Pwm->CCR1_Value;
    TIM3->CCR3 = control->Pwm->CCR2_Value;
    TIM3->CCR4 = control->Pwm->CCR2_Value;
    TIM9->CCR1 = control->Pwm->CCR3_Value;
    TIM9->CCR2 = control->Pwm->CCR3_Value;
}

static void APP_ThreePhase_Reset(APP_ThreePhase_t *control)
{
    control->Voltage_Loop_Counter = 0U;
    control->Voltage_D_PI.Integral = 0.0f;
    control->Voltage_Q_PI.Integral = 0.0f;
    control->Current_D_PI.Integral = 0.0f;
    control->Current_Q_PI.Integral = 0.0f;
    control->Voltage_D_PI.Output = 0.0f;
    control->Voltage_Q_PI.Output = 0.0f;
    control->Current_D_PI.Output = 0.0f;
    control->Current_Q_PI.Output = 0.0f;
    control->Debug_Id_Ref = 0.0f;
    control->Debug_Iq_Ref = 0.0f;
    control->Debug_Vd_Cmd = 0.0f;
    control->Debug_Vq_Cmd = 0.0f;
}

void APP_ThreePhase_Init(APP_ThreePhase_t *control, SPWM_t *pwm)
{
    control->Pwm = pwm;
    control->Theta = 0.0f;
    control->Voltage_Loop_Counter = 0U;

    Clarke_Init(&control->Current_Clarke);
    Park_Init(&control->Current_Park);
    Clarke_Init(&control->Voltage_Clarke);
    Park_Init(&control->Voltage_Park);

    PI_Init(&control->Voltage_D_PI, DQ_VOLTAGE_KP, DQ_VOLTAGE_KI,
            CONTROL_SAMPLE_TIME * DQ_VOLTAGE_LOOP_DIVIDER,
            DQ_CURRENT_REF_LIMIT, -DQ_CURRENT_REF_LIMIT);
    PI_Init(&control->Voltage_Q_PI, DQ_VOLTAGE_KP, DQ_VOLTAGE_KI,
            CONTROL_SAMPLE_TIME * DQ_VOLTAGE_LOOP_DIVIDER,
            DQ_CURRENT_REF_LIMIT, -DQ_CURRENT_REF_LIMIT);
    PI_Init(&control->Current_D_PI, DQ_CURRENT_KP, DQ_CURRENT_KI,
            CONTROL_SAMPLE_TIME, 1.0f, -1.0f);
    PI_Init(&control->Current_Q_PI, DQ_CURRENT_KP, DQ_CURRENT_KI,
            CONTROL_SAMPLE_TIME, 1.0f, -1.0f);

    APP_ThreePhase_Reset(control);
}

void APP_ThreePhase_TimerTick(APP_ThreePhase_t *control)
{
    control->Theta += THREE_PHASE_STEP;
    if (control->Theta >= 2.0f * PI_VALUE) {
        control->Theta -= 2.0f * PI_VALUE;
    }
}

void APP_ThreePhase_Update(APP_ThreePhase_t *control,
                           float vdc_real,
                           float ia_real, float ib_real,
                           float va_real, float vb_real)
{
    float ic_real;
    float vc_real;
    float theta_dq;

    ic_real = -ia_real - ib_real;
    vc_real = -va_real - vb_real;
    theta_dq = control->Theta + DQ_THETA_OFFSET;
    if (theta_dq < 0.0f) {
        theta_dq += 2.0f * PI_VALUE;
    }

    Clarke_Update(&control->Current_Clarke, ia_real, ib_real, ic_real);
    Park_Update(&control->Current_Park,
                control->Current_Clarke.alpha,
                control->Current_Clarke.beta,
                theta_dq);
    Clarke_Update(&control->Voltage_Clarke, va_real, vb_real, vc_real);
    Park_Update(&control->Voltage_Park,
                control->Voltage_Clarke.alpha,
                control->Voltage_Clarke.beta,
                theta_dq);

    control->Debug_Vdc = vdc_real;
    control->Debug_Ia = ia_real;
    control->Debug_Ib = ib_real;
    control->Debug_Ic = ic_real;
    control->Debug_Id = control->Current_Park.D;
    control->Debug_Iq = control->Current_Park.Q;
    control->Debug_Va = va_real;
    control->Debug_Vb = vb_real;
    control->Debug_Vc = vc_real;
    control->Debug_Vd = control->Voltage_Park.D;
    control->Debug_Vq = control->Voltage_Park.Q;

    if (vdc_real < DQ_MIN_VDC) {
        APP_ThreePhase_Reset(control);
        control->Pwm->CCR1_Value = (uint32_t)(control->Pwm->Period * 0.5f);
        control->Pwm->CCR2_Value = (uint32_t)(control->Pwm->Period * 0.5f);
        control->Pwm->CCR3_Value = (uint32_t)(control->Pwm->Period * 0.5f);
        APP_ThreePhase_WritePwm(control);
        return;
    }

    if (control->Voltage_Loop_Counter == 0U) {
        control->Debug_Id_Ref = PI_Update(&control->Voltage_D_PI,
                                          DQ_VD_REF,
                                          control->Voltage_Park.D);
        control->Debug_Iq_Ref = PI_Update(&control->Voltage_Q_PI,
                                          DQ_VQ_REF,
                                          control->Voltage_Park.Q);
    }
    control->Voltage_Loop_Counter++;
    if (control->Voltage_Loop_Counter >= DQ_VOLTAGE_LOOP_DIVIDER) {
        control->Voltage_Loop_Counter = 0U;
    }

    control->Current_D_PI.Out_Max = DQ_MAX_VOLTAGE_RATIO * vdc_real;
    control->Current_D_PI.Out_Min = -control->Current_D_PI.Out_Max;
    control->Current_Q_PI.Out_Max = control->Current_D_PI.Out_Max;
    control->Current_Q_PI.Out_Min = control->Current_D_PI.Out_Min;

    control->Debug_Vd_Cmd = control->Voltage_Park.D +
                            PI_Update(&control->Current_D_PI,
                                      control->Debug_Id_Ref,
                                      control->Current_Park.D);
    control->Debug_Vq_Cmd = control->Voltage_Park.Q +
                            PI_Update(&control->Current_Q_PI,
                                      control->Debug_Iq_Ref,
                                      control->Current_Park.Q);

    {
        float sin_theta;
        float cos_theta;
        float v_alpha_cmd;
        float v_beta_cmd;

        arm_sin_cos_f32(theta_dq, &sin_theta, &cos_theta);
        v_alpha_cmd = control->Debug_Vd_Cmd * cos_theta -
                      control->Debug_Vq_Cmd * sin_theta;
        v_beta_cmd = control->Debug_Vd_Cmd * sin_theta +
                     control->Debug_Vq_Cmd * cos_theta;
        SVPWM_Update(control->Pwm, v_alpha_cmd, v_beta_cmd, vdc_real);
        APP_ThreePhase_WritePwm(control);
    }
}
