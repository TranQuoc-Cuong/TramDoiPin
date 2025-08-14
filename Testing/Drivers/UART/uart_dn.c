#include "uart_dn.h"
#include "ringbuffer.h"
#include <string.h>

static UART_HandleTypeDef* uart_handle;
static RINGBUF rx_ringbuf;
static uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart_rx_byte;


void UART_Handler_Init(UART_HandleTypeDef* huart) {
    uart_handle = huart;
    RINGBUF_Init(&rx_ringbuf, rx_buffer, sizeof(rx_buffer));
    HAL_UART_Receive_IT(uart_handle, &uart_rx_byte, 1);
}

void UART_Handler_TransmitString(const char* str) {
    HAL_UART_Transmit(uart_handle, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

bool UART_Handler_GetLine(char* buffer, uint8_t max_len) {
    HAL_NVIC_DisableIRQ(USART1_IRQn); 

    bool found_line = false;
    uint32_t current_pos = rx_ringbuf.rd;
    uint32_t next_pos;
    uint32_t i = 0;

    while (current_pos != rx_ringbuf.w) {
        if (rx_ringbuf.buf[current_pos] == '\n') {
            found_line = true;
            break;
        }
        current_pos = (current_pos + 1) % rx_ringbuf.size;
    }

    if (found_line) {
        while (RINGBUF_Get(&rx_ringbuf, (uint8_t*)&buffer[i]) == RING_OK) {
            if (buffer[i] == '\n' || i >= (max_len - 1)) {
                break;
            }
            i++;
        }
        buffer[i] = '\0'; 
    }

    HAL_NVIC_EnableIRQ(USART1_IRQn);
    return found_line;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == uart_handle->Instance) {
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        RINGBUF_Put(&rx_ringbuf, uart_rx_byte);
        HAL_UART_Receive_IT(uart_handle, &uart_rx_byte, 1);
    }
}
