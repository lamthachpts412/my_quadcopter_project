/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file            : main.c
 * @author          : Lam Thach
 * @date            : 27/09/23
 * @brief           : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "battery.h"
#include "led_error.h"
#include "printf_uart.h"
#include "receiver.h"
#include "pwm.h"
#include "mpu6050.h"
#include "pid.h"
#include "micro_second.h"
#include "lowpass_filter.h"

#include "drone.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMEOUT_MAIN	4000u			/* The unit is micro second */
#define T_SAMPLE	(TIMEOUT_MAIN * 1e-6f)	/* The unit is second */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define Throttle_Is_Low()		(drone.receiver->out[THROTTLE] < 1050)
#define Yaw_Is_Left()			(drone.receiver->out[YAW] < 1050)	
#define Yaw_Is_Center()			(drone.receiver->out[YAW] > 1450)
#define Yaw_Is_Right()			(drone.receiver->out[YAW] > 1950)

#define Motor_Is_Stop()			(drone.state == STOP)
#define Motor_Is_Prepare()		(drone.state == PREPARE)
#define Motor_Is_Start()		(drone.state == START)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Drone_TypeDef drone;
MPU6050_TypeDef mpu;
Receiver_TypeDef receiver;
PID_TypeDef pid[3];

uint32_t timeNow = 0u;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	Receiver_Get_Value_IT_Complete(&receiver);
}

/* USER CODE END 0 */
















/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* Initial peripheral -------------------------------------------------------------------------*/\
  Printf_UART_Init(&huart1);
  HAL_Delay(100);

  MPU6050_Init(&mpu, &hi2c2, ACC_RANGE_8G, GYRO_RANGE_1000, USE_NO_IT, TIMEOUT_MAIN);
  HAL_Delay(100);

  /* Calibrate value for accelerometer and gyroscope */
  // MPU6050_Acc_Get_Offset_Raw(&mpu, 2000);
  MPU6050_Gyro_Get_Offset_Raw(&mpu, 2000);
  HAL_Delay(100);

  PID_Init(&pid[X], 6, 0, 1, T_SAMPLE); 		/* P = 6, I = [1:6], D = [0:1] */
  PID_Init(&pid[Y], 6, 0, 1, T_SAMPLE);
  PID_Init(&pid[Z], 1, 6, 0, T_SAMPLE); 		/* P = [0:1], I = 6, D = 0 */
  HAL_Delay(100);

  PWM_Init(&htim4);
  HAL_Delay(100);

  Receiver_Get_Value_IT(&receiver, &htim2, GPIOA);
  HAL_Delay(100);

  Drone_Init(&drone, 
          &hadc1, 
          &mpu, 
          &receiver, 
          pid);
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Error Handler --------------------------------------------------------------------------------*/\
  Drone_Warn_Low_Battery(&drone, 11.0f);
  Drone_Warn_Not_Detect_MPU6050(&drone);
  Drone_Warn_Not_Detect_Receiver(&drone);
  Drone_Warn_Throttle_Not_In_Lowest_Position(&drone);

  while (1) {
    timeNow = Get_Micro_Second();

    /* MPU6050: Get angle and dps value -----------------------------------------------------------*/\
    MPU6050_Get_Angle(&mpu);


    /* Drone: Get setpoint value ------------------------------------------------------------------*/\
    Drone_Get_Angle_Setpoint(&drone, X, ROLL, 30.0f);
    Drone_Get_Angle_Setpoint(&drone, Y, PITCH, 30.0f);
    Drone_Get_Dps_Setpoint(&drone, Z, YAW, 150.0f);


    /* PID Controller: Calculate PID output -------------------------------------------------------*/\
    PID_Controller(&pid[X], &mpu.angle[X], &drone.setpoint[X]);
    PID_Controller(&pid[Y], &mpu.angle[Y], &drone.setpoint[Y]);
    PID_Controller(&pid[Z], &mpu.gyro.dps[Z], &drone.setpoint[Z]);


    /* Motor: On/Off mechanism --------------------------------------------------------------------*/\
    if (Throttle_Is_Low() && Yaw_Is_Left()) {
      drone.state = PREPARE;
    }

    if (Motor_Is_Prepare() && Throttle_Is_Low() && Yaw_Is_Center()) {
      drone.state = START;
      Turn_Off_Green_Led();

      /* Reset PID parameter */
      PID_Reset_Parameter(drone.pid[X]);
      PID_Reset_Parameter(drone.pid[Y]);
      PID_Reset_Parameter(drone.pid[Z]);
    }

    if (Motor_Is_Start() && Throttle_Is_Low() && Yaw_Is_Right()) {
      drone.state = STOP;
      Turn_On_Green_Led();
    }


    /* ESC: Calculate value and control -----------------------------------------------------------*/\
    if (Motor_Is_Start()) {
      Drone_Esc_Calc_Value(&drone);
    }
    else {
      Drone_Esc_Reset_Value(&drone);
    }

    PWM_Control_ESC(&htim4, TIM_CHANNEL_1, drone.esc[0]);
    PWM_Control_ESC(&htim4, TIM_CHANNEL_2, drone.esc[1]);
    PWM_Control_ESC(&htim4, TIM_CHANNEL_3, drone.esc[2]);
    PWM_Control_ESC(&htim4, TIM_CHANNEL_4, drone.esc[3]);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    while ((Get_Micro_Second() - timeNow) < TIMEOUT_MAIN);
  } /* End of while loop */

  /* USER CODE END 3 */
}













/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
