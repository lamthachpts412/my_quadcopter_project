#include "pwm.h"


/* Public Functions -------------------------------------------------------------------------------*/\

/**
 * @name PWM_Init.
 * @brief This function used to enable PWM.
 * @param htim Pointer to a TIM_HandleTypeDef structure.
 * @retval None.
*/
void PWM_Init(TIM_HandleTypeDef *htim) {
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
}


/**
 * @name PWM_Control_ESC.
 * @brief This function used to create a PWM at output of timmer.
 * @param htim Pointer to a TIM_HandleTypeDef structure.
 * @param tim_channel This parameter can be one of the following value:
 *                    @arg TIM_CHANNEL_1,
 *                    @arg TIM_CHANNEL_2,
 *                    @arg TIM_CHANNEL_3,
 *                    @arg TIM_CHANNEL_4.
 * @param pulse_width This parameter used to adjust pulse with.
 *                    The value should be in range [1000:2000].
*/
void PWM_Control_ESC(TIM_HandleTypeDef *htim, uint8_t tim_channel, uint16_t pulse_width) {
    __HAL_TIM_SetCompare(htim, tim_channel, pulse_width);
}