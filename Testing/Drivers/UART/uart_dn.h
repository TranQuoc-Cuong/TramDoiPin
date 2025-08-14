#ifndef INC_UART_HANDLER_H_
#define INC_UART_HANDLER_H_

#include "main.h"
#include <stdbool.h>

#define UART_RX_BUFFER_SIZE 256 

void UART_Handler_Init(UART_HandleTypeDef* huart);


void UART_Handler_TransmitString(const char* str);


bool UART_Handler_GetLine(char* buffer, uint8_t max_len);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_UART_HANDLER_H_ */
