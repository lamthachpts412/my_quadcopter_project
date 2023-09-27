#include "micro_second.h"

/* Public Function --------------------------------------------------------------------------------*/\
/**
 * @brief This funtion used to get 1 tick after 1 micro second. 
 * @retval Tick. 
 */
uint32_t Get_Micro_Second(void)
{
  uint32_t m0 = HAL_GetTick();
  __IO uint32_t u0 = SysTick->VAL;
  uint32_t m1 = HAL_GetTick();
  __IO uint32_t u1 = SysTick->VAL;
  const uint32_t tms = SysTick->LOAD + 1;

  if (m1 != m0) {
    return (m1 * 1000 + ((tms - u1) * 1000) / tms);
  } else {
    return (m0 * 1000 + ((tms - u0) * 1000) / tms);
  }
}