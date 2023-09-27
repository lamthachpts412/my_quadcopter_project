#ifndef DRONE_H
#define DRONE_H

#include "led_error.h"
#include "battery.h"
#include "mpu6050.h"
#include "receiver.h"
#include "pid.h"

/* Public structures ------------------------------------------------------------------------------*/\
typedef enum {
    STOP = 0u,
    PREPARE,
    START,
} Motor_State;

typedef struct drone {
    ADC_HandleTypeDef *hadc;
    MPU6050_TypeDef *mpu;
    Receiver_TypeDef *receiver;

    PID_TypeDef *pid[3];

    float setpoint[3];

    int16_t esc[4];
    
    Motor_State state;
} Drone_TypeDef;





/* Public Function Prototypes ---------------------------------------------------------------------*/\
void Drone_Init(Drone_TypeDef *drone,
                ADC_HandleTypeDef *hadc,
                MPU6050_TypeDef *mpu,
                Receiver_TypeDef *receiver,
                PID_TypeDef *pid);

void Drone_Warn_Low_Battery(Drone_TypeDef *drone, float warn_level);
void Drone_Warn_Not_Detect_MPU6050(Drone_TypeDef *drone);
void Drone_Warn_Not_Detect_Receiver(Drone_TypeDef *drone);
void Drone_Warn_Throttle_Not_In_Lowest_Position(Drone_TypeDef *drone);

void Drone_Get_Angle_Setpoint (Drone_TypeDef *drone, 
                               uint8_t angle_axis,
                               uint8_t rx_channel, 
                               float angle_limit);
void Drone_Get_Dps_Setpoint (Drone_TypeDef *drone, 
                             uint8_t dps_axis, 
                             uint8_t rx_channel,
                             float dps_limit);

void Drone_Esc_Reset_Value(Drone_TypeDef *drone);
void Drone_Esc_Calc_Value(Drone_TypeDef *drone);

#endif