#ifndef MPU6050_H
#define MPU6050_H


#include "stm32f1xx_hal.h"


/* Public Structure -------------------------------------------------------------------------------*/\
/**
 * @brief Define Acc_TypeDef structure.
*/\
typedef struct acc {
    float raw[3];           /* Store raw value for acc */


    float offsetRaw[3];     /* Store filtered raw offset value for gyro */

    float angle[3];         /* Store angle value for acc */

} Acc_TypeDef;


/**
 * @brief Define Temp_TypeDef structure.
*/\
typedef struct temp {
    float raw;           /*Store raw value for temp */
} Temp_TypeDef;


/**
 * @brief Define Gyro_TypeDef structure.
*/\
typedef struct gyro {
    float raw[3];           /* Store raw value for gyro */

    float dps[3];


    float offsetRaw[3];     /* Store filtered raw offset value for gyro */


    float angle[3];         /* Store angle value for gyro */

} Gyro_TypeDef;


/**
 * @brief Define MPU6050_TypeDef structure.
*/\
typedef struct mpu6050{
    I2C_HandleTypeDef *hi2c;

    Acc_TypeDef acc;
    Temp_TypeDef temp;
    Gyro_TypeDef gyro;

    float angle[3];
    float anglePrev[3];
    
} MPU6050_TypeDef;



/* Public Defines ---------------------------------------------------------------------------------*/\
/**
 * @brief Define axis-es for mpu6050.
*/\
#define X      0u
#define Y      1u
#define Z      2u
     

/**
 * @brief Define range for accelerometer. 
*/\
#define ACC_RANGE_2G        (uint8_t) (0<<3)
#define ACC_RANGE_4G        (uint8_t) (1<<3)
#define ACC_RANGE_8G        (uint8_t) (2<<3)
#define ACC_RANGE_16G       (uint8_t) (3<<3)


/**
 * @brief Define range for gyroscope. 
*/\
#define GYRO_RANGE_250      (uint8_t) (0<<3)
#define GYRO_RANGE_500      (uint8_t) (1<<3)
#define GYRO_RANGE_1000     (uint8_t) (2<<3)
#define GYRO_RANGE_2000     (uint8_t) (3<<3)

/**
 * @brief Define for using interrupt.
*/\
#define USE_IT       1u
#define USE_NO_IT    0u

/* Public Function Prototypes ---------------------------------------------------------------------*/\
/**
 * @brief Declare some public function prototypes 
*/\
void MPU6050_Init(MPU6050_TypeDef *mpu, 
                  I2C_HandleTypeDef *hi2c, 
                  uint8_t acc_range, 
                  uint8_t gyro_range,
                  uint8_t use_interrupt,
                  uint32_t timeout_main);
                  
HAL_StatusTypeDef MPU6050_Is_Ready(MPU6050_TypeDef *mpu);

void MPU6050_Get_Raw(MPU6050_TypeDef *mpu);
void MPU6050_Get_Dps(MPU6050_TypeDef *mpu);

void MPU6050_Get_Raw_IT(MPU6050_TypeDef *mpu);
void MPU6050_Get_Dps_IT(MPU6050_TypeDef *mpu);
void MPU6050_Get_Angle_IT(MPU6050_TypeDef *mpu);

void MPU6050_Get_Raw_IT_Complete(MPU6050_TypeDef *mpu);
void MPU6050_Get_Dps_IT_Complete(MPU6050_TypeDef *mpu);
void MPU6050_Get_Angle_IT_Complete(MPU6050_TypeDef *mpu);

void MPU6050_Acc_Get_Offset_Raw(MPU6050_TypeDef *mpu, uint16_t num_of_sample);
void MPU6050_Gyro_Get_Offset_Raw(MPU6050_TypeDef *mpu, uint16_t num_of_sample);

void MPU6050_Get_Angle(MPU6050_TypeDef *mpu);





#endif