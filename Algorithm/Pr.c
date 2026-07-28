#include "Pr.h"

#define PR_PI 3.14159265358979323846f

void PR_Reset(PR_Controller_t *pr)
{
    pr->error_prev1 = 0.0f;
    pr->error_prev2 = 0.0f;
    pr->output_prev1 = 0.0f;
    pr->output_prev2 = 0.0f;
}

/*
 * Resonant term in continuous time:
 * G_r(s) = 2 * Kr * wc * s / (s^2 + 2 * wc * s + w0^2)
 *
 * Tustin substitution converts it to the discrete coefficients used by
 * PR_Update(). This runs once during initialization, never in the ISR.
 */
void PR_Init(PR_Controller_t *pr, float kp, float kr,
             float target_freq_hz, float bandwidth_hz, float sample_time,
             float max_out, float min_out)
{
    float tustin_k;
    float omega_0;
    float omega_c;
    float d0;
    float d1;
    float d2;
    float temp;

    pr->Kp = kp;
    pr->Kr = kr;
    pr->target_freq_hz = target_freq_hz;
    pr->bandwidth_hz = bandwidth_hz;
    pr->sample_time = sample_time;
    pr->Max_Output = max_out;
    pr->Min_Output = min_out;

    if (pr->Max_Output < pr->Min_Output) {
        temp = pr->Max_Output;
        pr->Max_Output = pr->Min_Output;
        pr->Min_Output = temp;
    }

    if ((sample_time <= 0.0f) || (target_freq_hz <= 0.0f) ||
        (bandwidth_hz <= 0.0f)) {
        PR_Set_Coeffs(pr, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    tustin_k = 2.0f / sample_time;
    omega_0 = 2.0f * PR_PI * target_freq_hz;
    omega_c = 2.0f * PR_PI * bandwidth_hz;

    d0 = tustin_k * tustin_k + 2.0f * omega_c * tustin_k + omega_0 * omega_0;
    d1 = -2.0f * tustin_k * tustin_k + 2.0f * omega_0 * omega_0;
    d2 = tustin_k * tustin_k - 2.0f * omega_c * tustin_k + omega_0 * omega_0;

    pr->b0 = (2.0f * kr * omega_c * tustin_k) / d0;
    pr->b1 = 0.0f;
    pr->b2 = -pr->b0;
    pr->a1 = -d1 / d0;
    pr->a2 = -d2 / d0;

    PR_Reset(pr);
}

/*
 * Optional low-level override for verification against MATLAB or another
 * design tool. Normal application code should use PR_Init().
 */
void PR_Set_Coeffs(PR_Controller_t *pr, float b0, float b1, float b2,
                   float a1, float a2)
{
    pr->b0 = b0;
    pr->b1 = b1;
    pr->b2 = b2;
    pr->a1 = a1;
    pr->a2 = a2;
    PR_Reset(pr);
}

float PR_Update(PR_Controller_t *pr, float target, float actual)
{
    float error;
    float p_term;
    float r_term;
    float total_output;

    error = target - actual;
    p_term = pr->Kp * error;

    r_term = pr->a1 * pr->output_prev1
           + pr->a2 * pr->output_prev2
           + pr->b0 * error
           + pr->b1 * pr->error_prev1
           + pr->b2 * pr->error_prev2;

    /*
     * Limit the resonant state before it is stored. This prevents the
     * resonant term from continuing to build up while PWM is saturated.
     */
    if (r_term > (pr->Max_Output - p_term)) {
        r_term = pr->Max_Output - p_term;
    } else if (r_term < (pr->Min_Output - p_term)) {
        r_term = pr->Min_Output - p_term;
    }
    total_output = p_term + r_term;

    pr->output_prev2 = pr->output_prev1;
    pr->output_prev1 = r_term;
    pr->error_prev2 = pr->error_prev1;
    pr->error_prev1 = error;

    return total_output;
}
