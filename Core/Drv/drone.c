#include "drone.h"

/* Private Macros ---------------------------------------------------------------------------------*/\
#define Battery_Is_Low()                (Battery_Get_Value(drone->hadc) < warn_level)  

#define MPU6050_Is_Not_Detected()       (MPU6050_Is_Ready(drone->mpu) != HAL_OK)  

#define Receiver_Is_Not_Detected()      (drone->receiver->out[THROTTLE] < 990 || \
                                         drone->receiver->out[ROLL] < 990 || \
                                         drone->receiver->out[PITCH] < 990 || \
                                         drone->receiver->out[YAW] < 990)   

#define Throttle_Is_Not_In_Lowest_Position()    (drone->receiver->out[THROTTLE] < 990 || \
                                                 drone->receiver->out[THROTTLE] > 1050)            



/* Private Function Prototypes --------------------------------------------------------------------*/\
static void Esc_Reset_Value(int16_t *esc);
static void Esc_Calc_Value(int16_t *esc, 
                           int16_t throttle, 
                           float pid_out_x, 
                           float pid_out_y, 
                           float pid_out_z);

static void Get_Setpoint(float *setpoint, 
                         uint8_t axis, 
                         int16_t *receiver_out, 
                         uint8_t channel, 
                         float limit);



/* Private Functions ------------------------------------------------------------------------------*/\
static void Esc_Reset_Value(int16_t *esc) {
    esc[0] = 1000; 
    esc[1] = 1000;
    esc[2] = 1000;
    esc[3] = 1000;
}


static void Esc_Calc_Value(int16_t *esc, 
                           int16_t throttle, 
                           float pid_out_x, 
                           float pid_out_y, 
                           float pid_out_z) {
    if (throttle > 1800) throttle = 1800;

    esc[0] = throttle - pid_out_y - pid_out_x - pid_out_z;
    esc[1] = throttle + pid_out_y - pid_out_x + pid_out_z;
    esc[2] = throttle + pid_out_y + pid_out_x - pid_out_z;
    esc[3] = throttle - pid_out_y + pid_out_x + pid_out_z;

    /* Set low limit for esc - Keep motor is running */
    if (esc[0] < 1100) esc[0] = 1100;
    if (esc[1] < 1100) esc[1] = 1100;
    if (esc[2] < 1100) esc[2] = 1100;
    if (esc[3] < 1100) esc[3] = 1100;

    /* Set high limit for esc. */
    if (esc[0] > 2000) esc[0] = 2000;
    if (esc[1] > 2000) esc[1] = 2000;
    if (esc[2] > 2000) esc[2] = 2000;
    if (esc[3] > 2000) esc[3] = 2000;
}

static void Get_Setpoint(float *setpoint, 
                         uint8_t axis, 
                         int16_t *receiver_out, 
                         uint8_t channel, 
                         float limit) {
    setpoint[axis] = 0.0f;

    if(receiver_out[THROTTLE] > 1050) {
        if (receiver_out[channel] > 1508) {
            setpoint[axis] = receiver_out[channel] - 1508;
        }
        else if (receiver_out[channel] < 1492) {
            setpoint[axis] = receiver_out[channel] - 1492;
        }
    }

    /* Convert receiver value to setpoint value */
    setpoint[axis] *= (limit / 492.0f);
}



/* Public Functions -------------------------------------------------------------------------------*/\

/**
 * @name Drone_Init.
 * @brief This function used to initialize for drone.
 * @param drone Pointer to a Drone_TypeDef structure.
 * @param hadc Pointer to a ADC_HandleTypeDef structure.
 * @param mpu Pointer to a MPU6050_TypeDef structure.
 * @param receiver Pointer to a Receiver_TypeDef structure.
 * @param pid Pointer to a PID_TypeDef structure.
 * @retval None.
*/
void Drone_Init(Drone_TypeDef *drone,
                ADC_HandleTypeDef *hadc,
                MPU6050_TypeDef *mpu,
                Receiver_TypeDef *receiver,
                PID_TypeDef *pid) {
    drone->hadc = hadc;
    drone->mpu = mpu;
    drone->receiver = receiver;

    drone->pid[X] = &pid[X];
    drone->pid[Y] = &pid[Y];
    drone->pid[Z] = &pid[Z];

    drone->esc[0] = 0;
    drone->esc[1] = 0;
    drone->esc[2] = 0;
    drone->esc[3] = 0;

    drone->setpoint[X] = 0.0f;
    drone->setpoint[Y] = 0.0f;
    drone->setpoint[Z] = 0.0f;

    drone->state = STOP;
}



/**
 * @name Drone_Warn_Low_Battery.
 * @brief Error 1: Low battery.
 * @param drone Pointer to a Drone_TypeDef structure.
 * @param warn_level Warning battery level.
 * @retval None.
 */
void Drone_Warn_Low_Battery(Drone_TypeDef *drone, float warn_level) {
    LED_ErrorState error;

    while (Battery_Is_Low()) {
        error = ERROR_LOW_BATTERY;
        LED_Error_Handler(error);
    }

    error = ERROR_NONE;
    LED_Error_Handler(error);
}



/**
 * @name Drone_Warn_Not_Detect_MPU6050.
 * @brief Error 2: Do not detect the mpu6050. 
 * @param drone Pointer to a Drone_TypeDef structure.
 * @retval None.
 */
void Drone_Warn_Not_Detect_MPU6050(Drone_TypeDef *drone) {
    LED_ErrorState error;

    while (MPU6050_Is_Not_Detected()) {
        error = ERROR_NOT_DETECT_MPU6050;
        LED_Error_Handler(error);
    }

    error = ERROR_NONE;
    LED_Error_Handler(error);
}



/**
 * @name Drone_Warn_Not_Detect_Receiver.
 * @brief Error 3:  Do not detect receiver.
 * @param drone Pointer to a Drone_TypeDef structure.
 * @retval None.
 */
void Drone_Warn_Not_Detect_Receiver(Drone_TypeDef *drone) {
    LED_ErrorState error;

    while (Receiver_Is_Not_Detected()) {
        error = ERROR_NOT_DETECT_RECEIVER;
        LED_Error_Handler(error);
    }

    error = ERROR_NONE;
    LED_Error_Handler(error);
}



/**
 * @name Drone_Warn_Throttle_Not_In_Lowest_Position.
 * @brief Error 4: Throttle is not in lowest position.
 * @param drone Pointer to a Drone_TypeDef structure.
 * @retval None.
 */
void Drone_Warn_Throttle_Not_In_Lowest_Position(Drone_TypeDef *drone) {
    LED_ErrorState error;

    while (Throttle_Is_Not_In_Lowest_Position()) {
        error = ERROR_THROTTLE_NOT_LOWEST_POSITION;
        LED_Error_Handler(error);
    }

    error = ERROR_NONE;
    LED_Error_Handler(error);
}


/**
 * @name Drone_Get_Angle_Setpoint.
 * @brief This function used to get angle setpoint for PID controller from receiver.
 * @param drone Pointer to a Drone_TypeDef structure.
 * @param angle_axis The angle axis can be one of the following values:
 *                   @arg X,
 *                   @arg Y.
 * @param rx_channel The receiver channel can be one of the following values:
 *                   @arg ROLL,
 *                   @arg PITCH.
 * @param angle_limit Limit angle for drone.
 *                    The unit is in degree.
 * @retval None.
*/
void Drone_Get_Angle_Setpoint(Drone_TypeDef *drone, 
                              uint8_t angle_axis,
                              uint8_t rx_channel, 
                              float angle_limit) {
    Get_Setpoint(drone->setpoint,
                 angle_axis, 
                 drone->receiver->out,
                 rx_channel,
                 angle_limit);
}



/**
 * @name Drone_Get_Dps_Setpoint.
 * @brief This function used to get DPS setpoint for PID controller from receiver.
 * @param drone Pointer to a Drone_TypeDef structure.
 * @param dps_axis Axis of DPS. It must be Z-axis in this program.
 * @param rx_channel Channel of receiver. It must be YAW-channel in this program.
 * @param dps_limit Limit DPS for drone.
 *                  The unit is in degree per second.
 * @retval None.
*/
void Drone_Get_Dps_Setpoint(Drone_TypeDef *drone, 
                            uint8_t dps_axis, 
                            uint8_t rx_channel,
                            float dps_limit) {
    Get_Setpoint(drone->setpoint,
                 dps_axis, 
                 drone->receiver->out,
                 rx_channel,
                 dps_limit);
}


/**
 * @name Drone_Esc_Reset_Value.
 * @brief This function used to reset value for ESC.
 * @param drone Pointer to a Drone_TypeDef structure.
 * @retval None.
*/
void Drone_Esc_Reset_Value(Drone_TypeDef *drone) {
    Esc_Reset_Value(drone->esc);
}


/**
 * @name Drone_Esc_Calc_Value.
 * @brief This function used to calculate value for ESC.
 * @param drone Pointer to a Drone_TypeDef structure.
 * @retval None.
*/
void Drone_Esc_Calc_Value(Drone_TypeDef *drone) {
    Esc_Calc_Value(drone->esc,
                   drone->receiver->out[THROTTLE],
                   drone->pid[X]->out,
                   drone->pid[Y]->out,
                   drone->pid[Z]->out);
}




/* Proportional part */
// printf("error = %.2f\r\n", pidAngleY.error * pidAngleY.kp);
// printf("error = %.2f\r\n", pidDpsZ.error * pidDpsZ.kp);

/* Integral part */
// printf("errorSum = %.2f\r\n", pidAngleY.errorSum);
// printf("errorSum = %.2f\r\n", pidDpsZ.errorSum);

/* Differential part */
// printf("inputDiff = %.2f\r\n", pidAngleY.inputDiff * pidAngleY.kd);
// printf("errorDiff = %.2f\r\n", pidDpsZ.errorDiff * pidDpsZ.kd);


/* Check value from receiver. */
// printf("roll = %-10d, pitch = %-10d, throttle = %-10d, yaw = %d\n", 
//         receiver.out[angle_axis], receiver.out[PITCH], receiver.out[THROTTLE], receiver.out[YAW]);