#ifndef RECEIVER_H
#define RECEIVER_H

#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"

/* Define names for 4 channels of receiver. */
#define ROLL        0u
#define PITCH       1u
#define THROTTLE    2u
#define YAW         3u

/* Public Structure -------------------------------------------------------------------------------*/\
typedef struct {
    TIM_HandleTypeDef *htim;
    GPIO_TypeDef *gpio;

    int16_t tStart[4];
    int16_t tEnd[4];
    int16_t out[4];              /* Pulse width of receiver. */

} Receiver_TypeDef;


/* Public Function Prototypes ---------------------------------------------------------------------*/\
void Receiver_Get_Value_IT(Receiver_TypeDef *receiver, TIM_HandleTypeDef *htim, GPIO_TypeDef *gpio);
void Receiver_Get_Value_IT_Complete(Receiver_TypeDef *receiver);

#endif