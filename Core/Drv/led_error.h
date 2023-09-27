#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

#include "stm32f1xx_hal.h"

/* Public TypeDef ---------------------------------------------------------------------------------*/\
typedef enum {
    ERROR_NONE = 0u,                        /* No error. */             
    ERROR_LOW_BATTERY,                      /* Error 1: Flash LED 1 time. */
    ERROR_NOT_DETECT_MPU6050,               /* Error 2: Flash LED 2 times. */
    ERROR_NOT_DETECT_RECEIVER,              /* Error 3: Flash LED 3 times. */
    ERROR_THROTTLE_NOT_LOWEST_POSITION,     /* Error 4: Flash LED 4 times. */
} LED_ErrorState;

/* Public Macros ----------------------------------------------------------------------------------*/\
#define Turn_On_Green_Led()         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_GREEN, GPIO_PIN_SET) 
#define Turn_On_Red_Led()           HAL_GPIO_WritePin(GPIOA, GPIO_PIN_RED, GPIO_PIN_SET)

#define Turn_Off_Green_Led()        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_GREEN, GPIO_PIN_RESET)  
#define Turn_Off_Red_Led()          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_RED, GPIO_PIN_RESET)  

#define GPIO_PIN_RED                GPIO_PIN_11
#define GPIO_PIN_GREEN              GPIO_PIN_12

/* Public Function Prototype ----------------------------------------------------------------------*/\
void LED_Error_Handler(LED_ErrorState error);


#endif