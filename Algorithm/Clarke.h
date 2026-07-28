#ifndef __CLARKE_H
#define __CLARKE_H

typedef struct {
    float phase_a;
    float phase_b;
    float phase_c;

    float alpha;
    float beta;
    float zero;
} Clarke_t;

void Clarke_Init(Clarke_t *clarke);
void Clarke_Update(Clarke_t *clarke,
                   float phase_a, float phase_b, float phase_c);

#endif
