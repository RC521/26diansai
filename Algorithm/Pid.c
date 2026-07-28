#include "pid.h"

void PI_Init(PI_Controller_t *pi, float kp, float ki,
             float sample_time, float max, float min)
{
    float temp;

    pi->Kp = kp;
    pi->Ki = ki;
    pi->SampleTime = sample_time;
    pi->Integral = 0.0f;
    pi->Out_Max = max;
    pi->Out_Min = min;
    pi->Output = 0.0f;

    if (pi->Out_Max < pi->Out_Min) {
        temp = pi->Out_Max;
        pi->Out_Max = pi->Out_Min;
        pi->Out_Min = temp;
    }
}

float PI_Update(PI_Controller_t *pi, float target, float actual)
{
    float error;
    float p_term;
    float integral_next;
    float output_unsaturated;

    error = target - actual;
    p_term = pi->Kp * error;

    integral_next = pi->Integral + error * pi->SampleTime;
    output_unsaturated = p_term + pi->Ki * integral_next;

    if (output_unsaturated > pi->Out_Max) {
        pi->Output = pi->Out_Max;

        /* Let the integrator move only in the direction that removes saturation. */
        if (error < 0.0f) {
            pi->Integral = integral_next;
        }
    } else if (output_unsaturated < pi->Out_Min) {
        pi->Output = pi->Out_Min;

        if (error > 0.0f) {
            pi->Integral = integral_next;
        }
    } else {
        pi->Integral = integral_next;
        pi->Output = output_unsaturated;
    }

    return pi->Output;
}
