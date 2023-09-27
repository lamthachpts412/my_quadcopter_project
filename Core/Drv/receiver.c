#include "receiver.h"

/* Global Variables -------------------------------------------------------------------------------*/\



/* Private Defines --------------------------------------------------------------------------------*/\
#define TIM_CHANNEL(ch)         ((ch == HAL_TIM_ACTIVE_CHANNEL_1)? TIM_CHANNEL_1 : \
                                 ((ch == HAL_TIM_ACTIVE_CHANNEL_2)? TIM_CHANNEL_2 : \
                                  ((ch == HAL_TIM_ACTIVE_CHANNEL_3)? TIM_CHANNEL_3 : \
                                                                      TIM_CHANNEL_4 )))

#define GPIO_PIN(ch)            ((ch == HAL_TIM_ACTIVE_CHANNEL_1)? GPIO_PIN_0 : \
                                 ((ch == HAL_TIM_ACTIVE_CHANNEL_2)? GPIO_PIN_1 : \
                                  ((ch == HAL_TIM_ACTIVE_CHANNEL_3)? GPIO_PIN_2 : \
                                                                      GPIO_PIN_3 )))

#define INDEX(ch)               ((ch == HAL_TIM_ACTIVE_CHANNEL_1)? 0 : \
                                 ((ch == HAL_TIM_ACTIVE_CHANNEL_2)? 1 : \
                                  ((ch == HAL_TIM_ACTIVE_CHANNEL_3)? 2 : \
                                                                      3 )))



/* Global Functions -------------------------------------------------------------------------------*/\

/**
 * @name Receiver_Get_Value_IT.
 * @brief This function used to initialize for FS-IA10B receiver.
 * @param receiver Pointer to a Receiver_TypeDef structure.
 * @param htim Pointer to a TIM_HandleTypeDef structure.
 * @param gpio Pointer to a GPIO_TypeDef structure.
 * @retval None.
*/
void Receiver_Get_Value_IT(Receiver_TypeDef *receiver, TIM_HandleTypeDef *htim, GPIO_TypeDef *gpio) {
    /* Initialize value for some variables */
    receiver->htim = htim;
    receiver->gpio = gpio;

    /* Enable input capture interrupt for 4 channel of timer */
    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_4);
}


/**
 * @name Receiver_Get_Value_IT_Complete.
 * @brief This function used to get value from FS_IA10B receiver.
 * @param receiver Pointer to a Receiver_TypeDef structure.
 * @retval None.
 * @note This function should be use in HAL_TIM_IC_CaptureCallback function.
*/
void Receiver_Get_Value_IT_Complete(Receiver_TypeDef *receiver) {

    uint8_t timChannel = TIM_CHANNEL(HAL_TIM_GetActiveChannel(receiver->htim));
    uint16_t gpioPin = GPIO_PIN(HAL_TIM_GetActiveChannel(receiver->htim));
    uint8_t index = INDEX(HAL_TIM_GetActiveChannel(receiver->htim));

    /* If detect a rising edge */
    if(HAL_GPIO_ReadPin(receiver->gpio, gpioPin) == 1) {
        /* Record start time of pulse */
        receiver->tStart[index] = __HAL_TIM_GetCompare(receiver->htim, timChannel);

        /* Set capture polarity is falling edge */
        __HAL_TIM_SET_CAPTUREPOLARITY(receiver->htim, timChannel, TIM_INPUTCHANNELPOLARITY_FALLING);
    }

    /* If detect a falling edge */
    else {
        /* Record end time of pulse */
        receiver->tEnd[index] = __HAL_TIM_GetCompare(receiver->htim, timChannel); 

        /* Calculate width of pulse*/
        receiver->out[index] = receiver->tEnd[index] - receiver->tStart[index];

        /* If width of pulse is a negative number */
        if(receiver->out[index] < 0) {
            /* Convert width of pulse to a positive number */
            receiver->out[index] += UINT16_MAX;
        }

        /* Set capture polarity is rising edge*/
        __HAL_TIM_SET_CAPTUREPOLARITY(receiver->htim, timChannel, TIM_INPUTCHANNELPOLARITY_RISING);
    } 
}
