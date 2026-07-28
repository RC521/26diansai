#include "RMS.h"
#include "math.h"

void RMS_Init(RMS_t *rms)
{
    rms->sum_square = 0.0f;
    rms->index = 0;
    rms->count = 0;
    rms->output = 0.0f;

    for (uint16_t i = 0; i < RMS_BUF_SIZE; i++) {
        rms->buffer[i] = 0.0f;
    }
}

float RMS_Update(RMS_t *rms, float input)
{
    float new_square = input * input;

    rms->sum_square -= rms->buffer[rms->index];
    rms->buffer[rms->index] = new_square;
    rms->sum_square += new_square;

    rms->index++;

    if (rms->index >= RMS_BUF_SIZE) {
        rms->index = 0;
    }

    if (rms->count < RMS_BUF_SIZE) {
        rms->count++;
    }

    if (rms->sum_square < 0.0f) {
        rms->sum_square = 0.0f;
    }

    rms->output = sqrtf(rms->sum_square / rms->count);

    return rms->output;
}