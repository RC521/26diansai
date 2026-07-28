#ifndef APP_THREE_PHASE_H
#define APP_THREE_PHASE_H

#include "Algorithm.h"

typedef struct {
    SPWM_t *Pwm;
    Clarke_t Current_Clarke;
    Park_t Current_Park;
    Clarke_t Voltage_Clarke;
    Park_t Voltage_Park;
    PI_Controller_t Voltage_D_PI;
    PI_Controller_t Voltage_Q_PI;
    PI_Controller_t Current_D_PI;
    PI_Controller_t Current_Q_PI;
    float Theta;
    uint32_t Voltage_Loop_Counter;

    volatile float Debug_Vdc;
    volatile float Debug_Ia;
    volatile float Debug_Ib;
    volatile float Debug_Ic;
    volatile float Debug_Id;
    volatile float Debug_Iq;
    volatile float Debug_Va;
    volatile float Debug_Vb;
    volatile float Debug_Vc;
    volatile float Debug_Vd;
    volatile float Debug_Vq;
    volatile float Debug_Id_Ref;
    volatile float Debug_Iq_Ref;
    volatile float Debug_Vd_Cmd;
    volatile float Debug_Vq_Cmd;
} APP_ThreePhase_t;

void APP_ThreePhase_Init(APP_ThreePhase_t *control, SPWM_t *pwm);
void APP_ThreePhase_TimerTick(APP_ThreePhase_t *control);
void APP_ThreePhase_Update(APP_ThreePhase_t *control,
                           float vdc_real,
                           float ia_real, float ib_real,
                           float va_real, float vb_real);

#endif
