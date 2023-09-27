#include "pid.h"

#define ALPHA_D          0.4f

#define PID_UPPER_LIMIT      400.0f
#define PID_LOWER_LIMIT     -400.0f

#define I_UPPER_LIMIT       400.0f
#define I_LOWER_LIMIT      -400.0f
/* Public Functions -------------------------------------------------------------------------------*/\

/**
 * @name PID_Init.
 * @brief This function used to initialize for PID controller.
 * @param pid Pointer to a PID_TypeDef structure.
 * @param p_gain Proportional gain.
 * @param i_gain Integral gain.
 * @param d_gain Differential gain.
 * @param t_sample Sampling time (it must be larger 0). 
 *                 The unit is in second. 
 * @retval None.
*/
void PID_Init(PID_TypeDef *pid, float p_gain, float i_gain, float d_gain, float t_sample) {
    pid->tSample = t_sample;

    pid->kp = p_gain;
    pid->ki = i_gain * t_sample;
    pid->kd = d_gain / t_sample;
}


/**
 * @name PID_Controller.
 * @brief This function used to calculate out for PID controller.
 * @param pid Pointer to a PID_TypeDef structure.
 * @param input Pointer to input.
 * @param setpoint Pointer to setpoint.
 * @retval None.
*/
void PID_Controller(PID_TypeDef *pid, float *input, float *setpoint) {
        
    pid->error      = *setpoint - *input;  
    pid->inputDiff  = *input - pid->inputPrev;
    pid->errorSum  += pid->error * pid->ki; 


    /* Upper and lower limit for error sum */   
    if (pid->errorSum > I_UPPER_LIMIT) pid->errorSum = I_UPPER_LIMIT;
    else if (pid->errorSum < I_LOWER_LIMIT) pid->errorSum = I_LOWER_LIMIT;

    /* Calculate value for PID output */
    pid->out = (pid->kp * pid->error) + pid->errorSum - (pid->kd * pid->inputDiff);

    /* Upper and lower limit for PID output */
    if (pid->out > PID_UPPER_LIMIT) pid->out = PID_UPPER_LIMIT;
    else if (pid->out < PID_LOWER_LIMIT) pid->out = PID_LOWER_LIMIT;

    /* Update value for previous input */
    pid->inputPrev = *input;
}

/**
 * @name PID_Reset_Parameter.
 * @brief This function used to reset parameter for PID controller.
 * @param pid Pointer to a PID_TypeDef structure.
 * @retval None.
*/
void PID_Reset_Parameter(PID_TypeDef *pid) {
    pid->inputPrev = 0.0f;
    pid->errorPrev = 0.0f;
    pid->errorSum = 0.0f;
}

