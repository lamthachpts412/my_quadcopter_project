#ifndef PWM_H
#define PWM_H

#include "stm32f1xx_hal.h"


/* Public Function Prototypes ---------------------------------------------------------------------*/\
void PWM_Init(TIM_HandleTypeDef *htim);
void PWM_Control_ESC(TIM_HandleTypeDef *htim, uint8_t tim_channel, uint16_t pulse_width);

#endif