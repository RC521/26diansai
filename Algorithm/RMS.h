#ifndef __RMS_H
#define __RMS_H

#include <stdint.h>

#define RMS_BUF_SIZE 200

typedef struct {
    float buffer[RMS_BUF_SIZE];
    float sum_square;
    uint16_t index;
    uint16_t count;
    float output;
} RMS_t;

void RMS_Init(RMS_t *rms);
float RMS_Update(RMS_t *rms, float input);

#endif