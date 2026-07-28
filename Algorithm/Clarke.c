#include "Clarke.h"

#define CLARKE_TWO_THIRDS 0.666666667f  // 2/3
#define CLARKE_INV_SQRT3  0.577350269f  // 1/sqrt(3)

void Clarke_Init(Clarke_t *clarke)
{
    clarke->phase_a = 0.0f;
    clarke->phase_b = 0.0f;
    clarke->phase_c = 0.0f;
    clarke->alpha = 0.0f;
    clarke->beta = 0.0f;
    clarke->zero = 0.0f;
}

/*
 * Amplitude-invariant Clarke transform.
 * For a balanced three-wire system, phase_a + phase_b + phase_c is zero,
 * so the zero component should stay close to zero.
 */
void Clarke_Update(Clarke_t *clarke,
                   float phase_a, float phase_b, float phase_c)
{
    clarke->phase_a = phase_a;
    clarke->phase_b = phase_b;
    clarke->phase_c = phase_c;

    clarke->alpha = CLARKE_TWO_THIRDS *
                     (phase_a - 0.5f * phase_b - 0.5f * phase_c);
    clarke->beta = CLARKE_INV_SQRT3 * (phase_b - phase_c);
    clarke->zero = (phase_a + phase_b + phase_c) / 3.0f;
}
