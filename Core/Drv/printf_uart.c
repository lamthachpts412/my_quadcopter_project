#include "printf_uart.h"

UART_HandleTypeDef *pHuart;

/* Public Function --------------------------------------------------------------------------------*/\
/**
 * @brief This function used to initialize for for uart printf
 * @param huart Pointer to a UART_HandleTypeDef structure.
 * @retval None.
*/
void Printf_UART_Init(UART_HandleTypeDef *huart) {
	pHuart = huart;
}


int _write(int fd, char * ptr, int len) {
  HAL_UART_Transmit(pHuart, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}

