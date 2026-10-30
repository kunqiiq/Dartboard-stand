#include "bsp_uart.h"
#include <usart.h>

/*BSP层内部只读指针 用于查表回调函数*/
static UART_RxCallback_t rx_callback[UART_ID_Num] = {NULL};
static UART_ErrCallback_t error_callback[UART_ID_Num] = {NULL};

/*查表*/
static UART_ID Get_UART_ID(UART_HandleTypeDef *huart)
{
        if (huart->Instance == USART1) return UART_ID_USART1;
    else if (huart->Instance == USART3) return UART_ID_USART3;
    else if (huart->Instance == USART6) return UART_ID_USART6;
    else return UART_ID_Num;
}

/*注册函数实现 在中间层Init函数进行注册*/
void BSP_UART_RegisterRxCallback(UART_ID id, UART_RxCallback_t cb)
{
    if (id < UART_ID_Num)
    {
        rx_callback[id] = cb;
    }
}

void BSP_UART_RegisterErrorCallback(UART_ID id, UART_ErrCallback_t cb)
{
    if (id < UART_ID_Num)
    {
        error_callback[id] = cb;
    }
}

/*回调层查表实现 不需要再更改*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    UART_ID id = Get_UART_ID(huart);
    if (id < UART_ID_Num && rx_callback[id] != NULL)
    {
        rx_callback[id](Size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UART_ID id = Get_UART_ID(huart);
    if (id < UART_ID_Num && error_callback[id] != NULL)
    {
        error_callback[id]();
    }
}

