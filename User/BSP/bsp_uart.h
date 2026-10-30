/**
 * @file    bsp_uart.h
 * @brief   UART 板级支持包 (BSP) 接口声明
 *
 * 本模块统一分发 HAL UART 接收和错误回调。
 *
 * @note    HAL_UARTEx_RxEventCallback 在 bsp_uart.c 中统一实现，
 *          应用层如需接入新的 UART 接收处理，在该回调中按外设实例扩展即可。
 */

#ifndef BSP_UART_H
#define BSP_UART_H

#include <usart.h>

/*回调类型定义*/
typedef void (*UART_RxCallback_t)(uint16_t size);
typedef void (*UART_ErrCallback_t)(void);

/*UART索引*/
typedef enum
{
    	UART_ID_USART1,
	UART_ID_USART3,
	UART_ID_USART6,
    UART_ID_Num
} UART_ID;

/*外部注册接口*/
void BSP_UART_RegisterRxCallback(UART_ID id, UART_RxCallback_t cb);
void BSP_UART_RegisterErrorCallback(UART_ID id, UART_ErrCallback_t cb);

#endif /* BSP_UART_H */
