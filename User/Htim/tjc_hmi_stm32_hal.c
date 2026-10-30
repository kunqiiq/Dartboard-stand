#include "tjc_hmi_stm32_hal.h"

#include <string.h>

static int hal_write(const uint8_t *data, size_t len, void *user)
{
    TJC_HMI_HAL *adapter = (TJC_HMI_HAL *)user;
    HAL_StatusTypeDef status;

    if (adapter == NULL || adapter->uart == NULL || data == NULL ||
        len == 0u || len > UINT16_MAX) {
        return -1;
    }

    status = HAL_UART_Transmit(adapter->uart,
                               (uint8_t *)(uintptr_t)data,
                               (uint16_t)len,
                               adapter->tx_timeout_ms);
    return status == HAL_OK ? 0 : -1;
}

static uint16_t ring_next(uint16_t index)
{
    ++index;
    if (index >= (uint16_t)TJC_HAL_RX_RING_SIZE) {
        index = 0u;
    }
    return index;
}

static bool ring_push(TJC_HMI_HAL *adapter, uint8_t byte)
{
    const uint16_t next = ring_next(adapter->rx_head);
    if (next == adapter->rx_tail) {
        ++adapter->rx_overflow_count;
        return false;
    }
    adapter->rx_ring[adapter->rx_head] = byte;
    adapter->rx_head = next;
    return true;
}

void TJC_HMI_HAL_Init(TJC_HMI_HAL *adapter,
                      UART_HandleTypeDef *uart,
                      uint32_t tx_timeout_ms)
{
    if (adapter == NULL) {
        return;
    }
    memset(adapter, 0, sizeof(*adapter));
    adapter->uart = uart;
    adapter->tx_timeout_ms = tx_timeout_ms;
    TJC_HMI_Init(&adapter->hmi, hal_write, adapter);
}

HAL_StatusTypeDef TJC_HMI_HAL_StartReceiveIT(TJC_HMI_HAL *adapter)
{
    if (adapter == NULL || adapter->uart == NULL) {
        return HAL_ERROR;
    }
    return HAL_UART_Receive_IT(adapter->uart, &adapter->interrupt_rx_byte, 1u);
}

void TJC_HMI_HAL_UART_RxCpltCallback(TJC_HMI_HAL *adapter,
                                     UART_HandleTypeDef *uart)
{
    if (adapter == NULL || uart != adapter->uart) {
        return;
    }
    (void)ring_push(adapter, adapter->interrupt_rx_byte);
    (void)HAL_UART_Receive_IT(adapter->uart, &adapter->interrupt_rx_byte, 1u);
}

void TJC_HMI_HAL_UART_ErrorCallback(TJC_HMI_HAL *adapter,
                                    UART_HandleTypeDef *uart)
{
    if (adapter == NULL || uart != adapter->uart) {
        return;
    }
    (void)HAL_UART_Receive_IT(adapter->uart, &adapter->interrupt_rx_byte, 1u);
}

size_t TJC_HMI_HAL_QueueRxFromISR(TJC_HMI_HAL *adapter,
                                  const uint8_t *data,
                                  size_t len)
{
    size_t accepted = 0u;
    if (adapter == NULL || (data == NULL && len != 0u)) {
        return 0u;
    }
    while (accepted < len) {
        if (!ring_push(adapter, data[accepted])) {
            break;
        }
        ++accepted;
    }
    return accepted;
}

size_t TJC_HMI_HAL_Process(TJC_HMI_HAL *adapter)
{
    size_t processed = 0u;
    if (adapter == NULL) {
        return 0u;
    }

    while (adapter->rx_tail != adapter->rx_head) {
        const uint8_t byte = adapter->rx_ring[adapter->rx_tail];
        adapter->rx_tail = ring_next(adapter->rx_tail);
        (void)TJC_HMI_FeedByte(&adapter->hmi, byte);
        ++processed;
    }
    return processed;
}

TJC_HMI *TJC_HMI_HAL_Core(TJC_HMI_HAL *adapter)
{
    return adapter == NULL ? NULL : &adapter->hmi;
}

uint32_t TJC_HMI_HAL_GetOverflowCount(const TJC_HMI_HAL *adapter)
{
    return adapter == NULL ? 0u : adapter->rx_overflow_count;
}
