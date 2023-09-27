#include "mpu6050.h"
#include <math.h>
#include <stdio.h>

#define ALPHA       0.001f             /* The alpha should be in range [0.0004 : 0.001]*/

/* Global Variables -------------------------------------------------------------------------------*/\
float tSample = 0.0f;                           /* The unit in second. */

float rawToDps = 0.0f;
uint8_t bufferIT[14] = {0};


/* Private Defines --------------------------------------------------------------------------------*/\
#define I2C_TIMEOUT         4u                  /* The unit in millisecond. */

/**
 * @brief Define sample time for 100kHz I2C interrupt mode. 
*/\
#define SAMPLE_TIME_IT      0.0015617f          /* The unit in second. */

/**
 * @brief Define some basic conversion values. 
*/\
#define PI                      3.141592f
#define DEG_TO_RAD              (PI / 180.0f)   /* Convert degree to radian */
#define RAD_TO_DEG              (180.0f / PI)   /* Convert radian to degree */



/**
 * @brief Define addresses for mpu6050 and some registers. 
*/\
#define ADD_MPU6050       (0x68<<1)
#define ADD_GYRO_CONFIG   0x1B          /* Register 27 – Gyroscope Configuration */
#define ADD_ACC_CONFIG    0x1C          /* Register 28 – Accelerometer Configuration */
#define ADD_ALL_OUT       0x3B      
#define ADD_ACC_OUT       0x3B          /* Registers 59 to 64 – Accelerometer Measurements */
#define ADD_GYRO_OUT      0x43          /* Registers 67 to 72 – Gyroscope Measurements */
#define ADD_PWR_MGMT_1    0x6B          /* Register 107 – Power Management 1 */
#define ADD_WHO_AM_I      0x75          /* Register 117 – Who Am I */
/**
 * @brief Define position of data registers in mpu6050.
*/\
#define GYRO_XOUT_H     0u
#define GYRO_XOUT_L     1u

#define GYRO_YOUT_H     2u      
#define GYRO_YOUT_L     3u      

#define GYRO_ZOUT_H     4u      
#define GYRO_ZOUT_L     5u      

#define TEMP_OUT_H      6u      
#define TEMP_OUT_L      7u      

#define ACC_XOUT_H      8u      
#define ACC_XOUT_L      9u      

#define ACC_YOUT_H      10u      
#define ACC_YOUT_L      11u      

#define ACC_ZOUT_H      12u      
#define ACC_ZOUT_L      13u 


/**
 * @brief Define modes for Power Management 1 register. 
*/\
#define EXIT_SLEEP_MODE     0u
#define ENTER_RESET_MODE    8u



/**
 * @brief Define some conversion values to convert a RAW value to a DPS value. 
 * @note DPS (degree per second).
*/\
#define RAW_TO_DPS_250        (1.0f / 131.0f)
#define RAW_TO_DPS_500        (1.0f / 65.5f)
#define RAW_TO_DPS_1000       (1.0f / 32.8f)
#define RAW_TO_DPS_2000       (1.0f / 16.4f)

#define RAW_TO_DPS(GYRO_RANGE)        ((GYRO_RANGE == GYRO_RANGE_250)? RAW_TO_DPS_250 : \
                                       ((GYRO_RANGE == GYRO_RANGE_500)? RAW_TO_DPS_500 : \
                                        ((GYRO_RANGE == GYRO_RANGE_1000)? RAW_TO_DPS_1000 : \
                                           RAW_TO_DPS_2000)))



/**
 * @brief Define a conversion value to convert a RAW value to a DPxS value.
 * @note DPxS (degree per X second).
*/\
#define RAW_TO_DPxS             (rawToDps * tSample)


/**
 * @brief Define a conversion value to convert a RAW value to a DPxR value.
 * @note DPxR (degree per X radian).
*/\
#define RAW_TO_DPxR             (rawToDps * tSample * DEG_TO_RAD)


/**
 * @brief Define a conversion value to convert a DPS value to a DPxS value.
 * @note DPxS (degree per X second).
*/\
#define DPS_TO_DPxS             tSample

/**
 * @brief Define a conversion value to convert a DPS value to a DPxR value.
 * @note DPxS (degree per X second).
*/\
#define DPS_TO_DPxR             (tSample * DEG_TO_RAD)

                                                                  
/* Private Function Prototypes --------------------------------------------------------------------*/\
static void Acc_Get_Sum_Raw(float *sum_raw, float *raw);
static void Gyro_Get_Sum_Raw(float *sum_raw, float *raw);

static void Acc_Get_Offset_Raw(float *offset_raw, float *sum_raw,  uint16_t num_of_sample);
static void Gyro_Get_Offset_Raw(float *offset_raw, float *sum_raw,  uint16_t num_of_sample);

static void Acc_Get_Angle(float *angle, float *raw, float *offset_raw);
static void Gyro_Get_Angle(float *angle, float *dps, float *mpu_angle_prev);

static void Gyro_Get_Dps(float *dps, float *raw, float *offset_raw);

static void Mpu_Get_Raw(float *acc_raw, float *temp_raw, float *gyro_raw, uint8_t *buffer);
static void Mpu_Get_Angle(float *mpu_angle, 
                          float *mpu_angle_prev, 
                          float *acc_angle, 
                          float *gyro_angle, 
                          float alpha);



/* Public Functions -------------------------------------------------------------------------------*/\
/**
 * @name MPU6050_Init.
 * @brief This function used to initialize for the MPU6050.
 * 
 * @param mpu Pointer to a MPU6050_TypeDef structure.
 * @param hi2c Pointer to a I2C_HandleTypeDef structure.
 * @param acc_range This parameter can be one of the following values:
 *                   @arg ACC_RANGE_2G,  
 *                   @arg ACC_RANGE_4G,
 *                   @arg ACC_RANGE_8G,
 *                   @arg ACC_RANGE_16G.
 * 
 * @param gyro_range This parameter can be one of the following values:
 *                    @arg GYRO_RANGE_250,
 *                    @arg GYRO_RANGE_500,
 *                    @arg GYRO_RANGE_1000,
 *                    @arg GYRO_RANGE_2000. 
 * 
 * @param use_interrupt This parameter can be one of the following values:
 *                      @arg USE_IT,
 *                      @arg USE_NO_IT.
 * 
 * @param timeout_main This is timeout of the loop in main function.
 *                     The unit is in micro second (us).
 * @retval None.
*/
void MPU6050_Init(MPU6050_TypeDef *mpu, 
                  I2C_HandleTypeDef *hi2c, 
                  uint8_t acc_range, 
                  uint8_t gyro_range,
                  uint8_t use_interrupt,
                  uint32_t timeout_main) {

    mpu->hi2c = hi2c;

    if(use_interrupt == USE_IT) {
        tSample = SAMPLE_TIME_IT;
    }
    else {
        tSample = (float)timeout_main * (1e-6f);        /* Convert micro second to second. */
    }

    rawToDps = RAW_TO_DPS(gyro_range);                  /* Convert the raw value to the dps value. */


    uint8_t buffer = 0u;
    
    buffer = ENTER_RESET_MODE;
    HAL_I2C_Mem_Write(hi2c, ADD_MPU6050, ADD_PWR_MGMT_1, 1, &buffer, 1, HAL_MAX_DELAY);

    buffer = EXIT_SLEEP_MODE;
    HAL_I2C_Mem_Write(hi2c, ADD_MPU6050, ADD_PWR_MGMT_1, 1, &buffer, 1, HAL_MAX_DELAY);

    buffer = acc_range;
    HAL_I2C_Mem_Write(hi2c, ADD_MPU6050, ADD_ACC_CONFIG, 1, &buffer, 1, HAL_MAX_DELAY);

    buffer = gyro_range;
    HAL_I2C_Mem_Write(hi2c, ADD_MPU6050, ADD_GYRO_CONFIG, 1, &buffer, 1, HAL_MAX_DELAY);

    // mpu->acc.offsetRaw[X] = 200.0f;           /* [400 : 600] ~ [Front- : Back+] */
    // mpu->acc.offsetRaw[Y] = -150.0f;          /* [-200 : -100] ~ [Left- : Right+] */

    mpu->acc.offsetRaw[X] = 235.4255f;           /* [500 : 580] ~ [Front- : Back+] */
    mpu->acc.offsetRaw[Y] = -103.9925f;          /* [-200 : -100] ~ [Left- : Right+] */
}

HAL_StatusTypeDef MPU6050_Is_Ready(MPU6050_TypeDef *mpu) {
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(mpu->hi2c, ADD_MPU6050, 1, HAL_MAX_DELAY);
    return status;
}

/**
 * @name MPU6050_Get_Raw.
 * @brief This function used to get all raw values from the mpu6050.
 * @param mpu Pointer to a MPU6050_TypeDef structure.
 * @retval None.
 * @note The raw value stored in the mpu pointer.
 * 
*/
void MPU6050_Get_Raw(MPU6050_TypeDef *mpu) {
    uint8_t buffer[14] = {0};
    HAL_I2C_Mem_Read(mpu->hi2c, ADD_MPU6050, ADD_ALL_OUT, 1, buffer, 14, HAL_MAX_DELAY);

    Mpu_Get_Raw(mpu->acc.raw, &mpu->temp.raw, mpu->gyro.raw, buffer);
}



/**
 * @name MPU6050_Acc_Get_Offset_Raw.
 * @brief This function used to get raw offset for accelerometer.
 * @param mpu Pointer to a MPU6050_TypeDef structure.
 * @param num_of_sample Number of sample.
 * @retval None.
 * 
 * @note How to use: After use this function, the result will printed to serial monitor.
 *       Copy those 2 raw offset value to initialize for 2 offset value in MPU6050_Init function. 
 *       Only need use this function once when have any change in physical position of mpu6050. 
 *  
*/
void MPU6050_Acc_Get_Offset_Raw(MPU6050_TypeDef *mpu, uint16_t num_of_sample) {
    float sumRaw[2];
    for(uint16_t i=0; i<num_of_sample; i++) {
        MPU6050_Get_Raw(mpu);


        /*Calculate sum of the angle value of acc*/
        Acc_Get_Sum_Raw(sumRaw, mpu->acc.raw);

        printf("i=%-4d: raw[X] = %-10.2f, raw[Y] = %-10.2f\n", i, mpu->acc.raw[X], mpu->acc.raw[Y]);
    }

    Acc_Get_Offset_Raw(mpu->acc.offsetRaw, sumRaw, num_of_sample);

    printf("sumRaw[0] = %-10.2f, sumRaw[1] = %.2f\n", sumRaw[X], sumRaw[Y]);
    printf("acc.offsetRawX = %-10.5f, acc.offsetRawY = %.5f\n", mpu->acc.offsetRaw[X], mpu->acc.offsetRaw[Y]);
}




/**
 * @name MPU6050_Gyro_Get_Offset_Raw.
 * @brief This function used to get raw offset from gyroscope.
 * @param mpu Pointer to a MPU6050_TypeDef structure.
 * @param num_of_sample Number of sample.
 * @retval None.  
 * 
*/
void MPU6050_Gyro_Get_Offset_Raw(MPU6050_TypeDef *mpu, uint16_t num_of_sample) {
    float sumRaw[3];
    for(uint16_t i=0; i<num_of_sample; i++) {
        MPU6050_Get_Raw(mpu);

        /*Calculate sum of the raw value of gyro*/
        Gyro_Get_Sum_Raw(sumRaw, mpu->gyro.raw);

    }
    Gyro_Get_Offset_Raw(mpu->gyro.offsetRaw, sumRaw, num_of_sample);
}


/**
 * @name MPU6050_Get_DPS.
 * @brief This function used to get DPS from gyroscope.
 * @param mpu Pointer to a MPU6050_TypeDef structure.
 * @retval None.  
 * 
*/
void MPU6050_Get_Dps(MPU6050_TypeDef *mpu) {
    MPU6050_Get_Raw(mpu);
    Gyro_Get_Dps(mpu->gyro.dps, mpu->gyro.raw, mpu->gyro.offsetRaw);
}



/**
 * @name MPU6050_Get_Angle.
 * @brief This function used to get angle from the mpu6050.
 * @param mpu Pointer to a MPU6050_TypeDef structure.
 * @retval None.
 * @note The angle value stored in mpu pointer.
 * 
*/
void MPU6050_Get_Angle(MPU6050_TypeDef *mpu) {
    
    MPU6050_Get_Raw(mpu);

    Acc_Get_Angle(mpu->acc.angle, mpu->acc.raw, mpu->acc.offsetRaw);
    
    Gyro_Get_Dps(mpu->gyro.dps, mpu->gyro.raw, mpu->gyro.offsetRaw);
    Gyro_Get_Angle(mpu->gyro.angle, mpu->gyro.dps, mpu->anglePrev);

    Mpu_Get_Angle(mpu->angle, mpu->anglePrev, mpu->acc.angle, mpu->gyro.angle, ALPHA);
}



/* Interrupt Public Functions ---------------------------------------------------------------------*/\

/**
 * @name MPU6050_Get_Raw_IT.
 * @brief This function used to enable interrupt for getting raw values.
 * @param mpu Pointer to a MPU6050_TypeDef structure.
 * @retval None.
 * 
*/
void MPU6050_Get_Raw_IT(MPU6050_TypeDef *mpu) {
    HAL_I2C_Mem_Read_IT(mpu->hi2c, ADD_MPU6050, ADD_ALL_OUT, 1, bufferIT, 14);
}

/**
 * @name MPU6050_Get_Raw_IT_Complete.
 * @brief This function used to get raw values from mpu6050 using interrupt mode.
 * @param mpu Pointer to a MPU6050_TypeDef structure.
 * @retval None.
 * @note Use this function in HAL_I2C_MemRxCpltCallback function.
*/
void MPU6050_Get_Raw_IT_Complete(MPU6050_TypeDef *mpu) {
   /* Get the accelerometer data */
    Mpu_Get_Raw(mpu->acc.raw, &mpu->temp.raw, mpu->gyro.raw, bufferIT);

    /* Re-enable Interrupt */
    HAL_I2C_Mem_Read_IT(mpu->hi2c, ADD_MPU6050, ADD_ALL_OUT, 1, bufferIT, 14);
}



/**
 * @name MPU6050_Get_Dps_IT.
 * @brief This function used to enable interrupt for getting dps values.
 * @param mpu Pointer to a MPU6050_TypeDef structure.
 * @retval None.
*/
void MPU6050_Get_Dps_IT(MPU6050_TypeDef *mpu) {
    HAL_I2C_Mem_Read_IT(mpu->hi2c, ADD_MPU6050, ADD_ALL_OUT, 1, bufferIT, 14);
}


/**
 * @name MPU6050_Get_Dps_IT_Complete.
 * This function used to get dps values from mpu6050 using interrupt mode.
 * @param mpu Pointer to a MPU6050_TypeDef structure.
 * @retval None.
 * @note Use this function in HAL_I2C_MemRxCpltCallback function.
*/
void MPU6050_Get_Dps_IT_Complete(MPU6050_TypeDef *mpu) {
   /* Get the accelerometer data */
    Mpu_Get_Raw(mpu->acc.raw, &mpu->temp.raw, mpu->gyro.raw, bufferIT);
    Gyro_Get_Dps(mpu->gyro.dps, mpu->gyro.raw, mpu->gyro.offsetRaw);

    /* Re-enable Interrupt */
    HAL_I2C_Mem_Read_IT(mpu->hi2c, ADD_MPU6050, ADD_ALL_OUT, 1, bufferIT, 14);
}



/**
 * @name MPU6050_Get_Angle_IT.
 * @brief This function used to enable interrupt for getting angle values.
 * @param mpu Pointer to a MPU6050_TypeDef structure.
 * @retval None.
*/
void MPU6050_Get_Angle_IT(MPU6050_TypeDef *mpu) {
    HAL_I2C_Mem_Read_IT(mpu->hi2c, ADD_MPU6050, ADD_ALL_OUT, 1, bufferIT, 14);
}


/**
 * @name MPU6050_Get_Angle_IT_Complete.
 * @brief This function used to get angle values from mpu6050 using interrupt mode.
 * @param mpu Pointer to a MPU6050_TypeDef structure.
 * @retval None.
 * @note Use this function in HAL_I2C_MemRxCpltCallback function.
*/
void MPU6050_Get_Angle_IT_Complete(MPU6050_TypeDef *mpu) {
    Mpu_Get_Raw(mpu->acc.raw, &mpu->temp.raw, mpu->gyro.raw, bufferIT);

    Acc_Get_Angle(mpu->acc.angle, mpu->acc.raw, mpu->acc.offsetRaw);

    Gyro_Get_Dps(mpu->gyro.dps, mpu->gyro.raw, mpu->gyro.offsetRaw);
    Gyro_Get_Angle(mpu->gyro.angle, mpu->gyro.dps, mpu->anglePrev);

    Mpu_Get_Angle(mpu->angle, mpu->anglePrev, mpu->acc.angle, mpu->gyro.angle, ALPHA);

    /* Re-enable Interrupt */
    HAL_I2C_Mem_Read_IT(mpu->hi2c, ADD_MPU6050, ADD_ALL_OUT, 1, bufferIT, 14);
}








/* Private Functions ------------------------------------------------------------------------------*/\
/**
 * @name Mpu_Get_Raw.
 * @brief This function used to get all the raw values from mpu6050. 
 * @param acc_raw Output: Pointer to acc raw variable.
 * @param temp_raw Output: Pointer to temp raw variable.
 * @param gyro_raw Output: Pointer to gyro raw variable.
 * @param buffer Input: Pointer to buffer variable.
 * @retval None.
 * 
*/
static void Mpu_Get_Raw(float *acc_raw, float *temp_raw, float *gyro_raw, uint8_t *buffer) {
    /* Get the accelerometer data */
    acc_raw[X] = (int16_t) (buffer[GYRO_XOUT_H]<<8 | buffer[GYRO_XOUT_L]);
    acc_raw[Y] = (int16_t) (buffer[GYRO_YOUT_H]<<8 | buffer[GYRO_YOUT_L]);
    acc_raw[Z] = (int16_t) (buffer[GYRO_ZOUT_H]<<8 | buffer[GYRO_ZOUT_L]);

    /*Get the temperate data*/
    *temp_raw = (int16_t) (buffer[TEMP_OUT_H]<<8 | buffer[TEMP_OUT_L]);

    /*Get the gyroscope data*/
    gyro_raw[X] = (int16_t) (buffer[ACC_XOUT_H]<<8 | buffer[ACC_XOUT_L]);
    gyro_raw[Y] = (int16_t) (buffer[ACC_YOUT_H]<<8 | buffer[ACC_YOUT_L]);
    gyro_raw[Z] = (int16_t) (buffer[ACC_ZOUT_H]<<8 | buffer[ACC_ZOUT_L]); 
}


/**
 * @name Mpu_Get_Angle.
 * @brief This function used to get angle from mpu6050. 
 * @param mpu_angle Output: Pointer to mpu angle variable.
 * @param mpu_angle_prev Input: Pointer to previous mpu angle variable.
 * @param acc_angle Input: Pointer to acc angle variable.
 * @param gyro_angle Input: Pointer to gyro angle variable.
 * @param alpha This alpha factor can be in range [0:1].
 *              If alpha -> 0, means using more gyro angle.
 *              If alpha -> 1, means using more acc angle.
 * @retval None.
 * 
*/
static void Mpu_Get_Angle(float *mpu_angle, 
                          float *mpu_angle_prev, 
                          float *acc_angle, 
                          float *gyro_angle, 
                          float alpha) {
    static uint8_t initAngle = 0;

    /* If mpu angle have not initialized yet. */
    if(initAngle == 0) {
        /* Initilaze mpu angle with acc angle */
        mpu_angle[X] = acc_angle[X];
        mpu_angle[Y] = acc_angle[Y];

        initAngle = 1;
    }

    /* If mpu angle have been initialized. */
    else {
        mpu_angle[X] = (gyro_angle[X]*(1-alpha)) + (acc_angle[X]*alpha); 
        mpu_angle[Y] = (gyro_angle[Y]*(1-alpha)) + (acc_angle[Y]*alpha); 
    }

    /* Update value for previous mpu angle. */
    mpu_angle_prev[X] = mpu_angle[X];
    mpu_angle_prev[Y] = mpu_angle[Y];
}


/**
 * @name Acc_Get_Sum_Raw.
 * @brief This function used to get the sum raw from accelerometer. 
 * @param sum_raw Output: Pointer to sum raw variable.
 * @param raw Input: Pointer to raw variable.
 * @retval None.
 * 
*/
static void Acc_Get_Sum_Raw(float *sum_raw, float *raw) {
    sum_raw[X] += raw[X];
    sum_raw[Y] += raw[Y];
}


/**
 * @name Gyro_Get_Sum_Raw.
 * @brief This function used to get the sum raw from gyroscope. 
 * @param sum_raw Output: Pointer to sum raw variable.
 * @param raw Input: Pointer to raw variable.
 * @retval None.
 * 
*/
static void Gyro_Get_Sum_Raw(float *sum_raw, float *raw) {
    sum_raw[X] += raw[X];
    sum_raw[Y] += raw[Y];
    sum_raw[Z] += raw[Z];
}


/**
 * @name Acc_Get_Offset_Raw.
 * @brief This function used to get the sum raw from accelerometer. 
 * @param sum_raw Output: Pointer to sum raw variable.
 * @param raw Input: Pointer to raw variable.
 * @retval None.
*/
static void Acc_Get_Offset_Raw(float *offset_raw, float *sum_raw,  uint16_t num_of_sample) {
    offset_raw[X] = (sum_raw[X] / (float)num_of_sample);
    offset_raw[Y] = (sum_raw[Y] / (float)num_of_sample);
}


/**
 * @name Gyro_Get_Offset_Raw.
 * @brief This function used to get the offset raw from gyroscope. 
 * @param offset_raw Output: Pointer to offset raw variable.
 * @param sum_raw Input: Pointer to sum raw variable.
 * @param num_of_sample Number of sample.
 * @retval None.
 * 
*/
static void Gyro_Get_Offset_Raw(float *offset_raw, float *sum_raw,  uint16_t num_of_sample) {
    offset_raw[X] = (sum_raw[X] / (float)num_of_sample);
    offset_raw[Y] = (sum_raw[Y] / (float)num_of_sample);
    offset_raw[Z] = (sum_raw[Z] / (float)num_of_sample);
}



/**
 * @name Acc_Get_Angle.
 * @brief This function used to get the offset raw from accelerometer. 
 * @param offset_raw Output: Pointer to offset raw variable.
 * @param sum_raw Input: Pointer to sum raw variable.
 * @param num_of_sample Number of sample.
 * @retval None.
*/
static void Acc_Get_Angle(float *angle, float *raw, float *offset_raw) {
    raw[X] -= offset_raw[X];
    raw[Y] -= offset_raw[Y];

    float totalVector = sqrtf((raw[X]*raw[X]) + (raw[Y]*raw[Y]) + (raw[Z]*raw[Z]));
    
    /* Prevent the asin function to produce a NaN */
    if(fabsf(raw[Y]) < totalVector) {
        angle[X] = asinf(raw[Y]/totalVector) * (RAD_TO_DEG);
    }
    if(fabsf(raw[X]) < totalVector) {
        angle[Y] = asinf(raw[X]/totalVector) * (-RAD_TO_DEG); 
    }
}


/**
 * @name Gyro_Get_Angle.
 * @brief  This function used to get the angle from gyroscope. 
 * @param gyro_angle Output: Pointer to angle variable.
 * @param dps Input: Pointer to dps variable.
 * @param mpu_angle_prev Input: Pointer to previous mpu angle variable.
 * @retval None.
 * 
*/
static void Gyro_Get_Angle(float *gyro_angle, float *dps, float *mpu_angle_prev) {

    gyro_angle[X] = mpu_angle_prev[X] + (dps[X]*(DPS_TO_DPxS));
    gyro_angle[Y] = mpu_angle_prev[Y] + (dps[Y]*(DPS_TO_DPxS));
    
    gyro_angle[X] += gyro_angle[Y] * sinf(dps[Z] * DPS_TO_DPxR);
    gyro_angle[Y] -= gyro_angle[X] * sinf(dps[Z] * DPS_TO_DPxR);
}



/**
 * @name Gyro_Get_Dps.
 * @brief  This function used to get the dps from gyroscope. 
 * @param dps Output: Pointer to dps variable.
 * @param raw Input: Pointer to raw variable.
 * @param offset_raw Pointer to offset raw variable.
 * @retval None.
 * 
*/
static void Gyro_Get_Dps(float *dps, float *raw, float *offset_raw) {
    raw[X] -= offset_raw[X];
    raw[Y] -= offset_raw[Y];
    raw[Z] -= offset_raw[Z];

    dps[X] = raw[X] * rawToDps;
    dps[Y] = raw[Y] * rawToDps;
    dps[Z] = raw[Z] * rawToDps;
}








