#ifndef PID_H
#define PID_H

#include "stm32f1xx_hal.h"

/* Public Structure -------------------------------------------------------------------------------*/\
typedef struct {
    float kp;
    float ki;
    float kd;

    float out;
    
    float tSample;

    float inputDiff;
    float inputPrev;


    float error;

    float errorDiff;
    float errorPrev;
    
    float errorSum;
} PID_TypeDef;


/* Public Function Prototypes ---------------------------------------------------------------------*/\
void PID_Init(PID_TypeDef *pid, float p_gain, float i_gain, float d_gain, float t_sample);
void PID_Controller(PID_TypeDef *pid, float *input, float *setpoint);
void PID_Reset_Parameter(PID_TypeDef *pid);

#endif