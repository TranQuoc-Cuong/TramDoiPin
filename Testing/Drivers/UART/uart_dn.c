#include "uart_dn.h"

#include <string.h>

#include "ringbuffer.h"

#define UART_RX_BUFFER_SIZE 256 

static UART_HandleTypeDef* uart_handle;
static RingBuf rx_ringbuf;
static uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart_rx_byte;

void InitHandlerUART(UART_HandleTypeDef* huart) {
	uart_handle = huart;
	InitRingBuf(&rx_ringbuf, rx_buffer, sizeof(rx_buffer));
	HAL_UART_Receive_IT(uart_handle, &uart_rx_byte, 1);
}

void TransmitStringHandlerUART(const char* str) {
	HAL_UART_Transmit(uart_handle, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

bool GetLineHandlerUART(char* buffer, uint8_t max_len) {
	IRQn_Type irq_num;
	
	if (uart_handle->Instance == USART1) irq_num = USART1_IRQn;
	else if (uart_handle->Instance == USART2) irq_num = USART2_IRQn;
	else if (uart_handle->Instance == USART3) irq_num = USART3_IRQn;
	
	HAL_NVIC_DisableIRQ(irq_num); 

	bool found_line = false;
	uint32_t current_pos = rx_ringbuf.rd;
//	uint32_t next_pos;
	uint32_t i = 0;

	while (current_pos != rx_ringbuf.w) {
		if (rx_ringbuf.buf[current_pos] == '\n') {
			found_line = true;
			break;
		}
		current_pos = (current_pos + 1) % rx_ringbuf.size;
	}

	if (found_line) {
		while (GetRingBuf(&rx_ringbuf, (uint8_t*)&buffer[i]) == RING_OK) {
			if (buffer[i] == '\n' || i >= (max_len - 1)) {
					break;
			}
			i++;
		}
		buffer[i] = '\0'; 
	}

	HAL_NVIC_EnableIRQ(irq_num);
	
	return found_line;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == uart_handle->Instance) {
		PutRingBuf(&rx_ringbuf, uart_rx_byte);
		HAL_UART_Receive_IT(uart_handle, &uart_rx_byte, 1);
	}
}
