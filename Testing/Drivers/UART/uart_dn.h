#ifndef INC_UART_HANDLER_H_
#define INC_UART_HANDLER_H_

#include "main.h"

void InitHandlerUART(UART_HandleTypeDef* huart);
void TransmitStringHandlerUART(const char* str);
bool GetLineHandlerUART(char* buffer, uint8_t max_len);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_UART_HANDLER_H_ */
