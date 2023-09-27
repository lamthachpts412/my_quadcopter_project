#include "battery.h"

/* Private Prototype ------------------------------------------------------------------------------*/\
#define RAW_TO_VOLT         (36.3f / 4095.0f)


/* Public Function --------------------------------------------------------------------------------*/\
float Battery_Get_Value(ADC_HandleTypeDef *hadc) {
    uint32_t adc_value = 0;
    
    HAL_ADC_Start(hadc);
    adc_value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    return ((adc_value*RAW_TO_VOLT) + 0.5f);
}