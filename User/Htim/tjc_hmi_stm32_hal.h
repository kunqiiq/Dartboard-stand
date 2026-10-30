#ifndef TJC_HMI_STM32_HAL_H
#define TJC_HMI_STM32_HAL_H

#include "main.h"
#include "tjc_hmi.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TJC_HAL_RX_RING_SIZE
#define TJC_HAL_RX_RING_SIZE 256u
#endif

typedef struct {
    TJC_HMI hmi;
    UART_HandleTypeDef *uart;
    uint32_t tx_timeout_ms;
    uint8_t interrupt_rx_byte;
    uint8_t rx_ring[TJC_HAL_RX_RING_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint32_t rx_overflow_count;
} TJC_HMI_HAL;

void TJC_HMI_HAL_Init(TJC_HMI_HAL *adapter,
                      UART_HandleTypeDef *uart,
                      uint32_t tx_timeout_ms);

/* One-byte interrupt mode. Call once after MX_USARTx_UART_Init(). */
HAL_StatusTypeDef TJC_HMI_HAL_StartReceiveIT(TJC_HMI_HAL *adapter);

/* Forward the matching HAL callback to these functions. */
void TJC_HMI_HAL_UART_RxCpltCallback(TJC_HMI_HAL *adapter,
                                     UART_HandleTypeDef *uart);
void TJC_HMI_HAL_UART_ErrorCallback(TJC_HMI_HAL *adapter,
                                    UART_HandleTypeDef *uart);

/* For DMA/IDLE users: enqueue bytes from HAL_UARTEx_RxEventCallback(). */
size_t TJC_HMI_HAL_QueueRxFromISR(TJC_HMI_HAL *adapter,
                                  const uint8_t *data,
                                  size_t len);

/* Call frequently from the main loop or a dedicated RTOS task. */
size_t TJC_HMI_HAL_Process(TJC_HMI_HAL *adapter);

TJC_HMI *TJC_HMI_HAL_Core(TJC_HMI_HAL *adapter);
uint32_t TJC_HMI_HAL_GetOverflowCount(const TJC_HMI_HAL *adapter);

#ifdef __cplusplus
}
#endif

#endif
