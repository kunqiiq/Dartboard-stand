/**
 * @file    bsp_can.h
 * @brief   CAN 总线板级支持包 (BSP) 接口声明
 *
 * 提供 CAN1/CAN2 外设的初始化、数据发送及接收中断回调封装。
 * 仅依赖 HAL CAN 驱动，不包含任何应用层逻辑。
 *
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx_hal.h"
#include "can.h"

/* ================================================================== */
/*  CAN ID 统一管理（收/发分组）                                          */
/* ================================================================== */

/* ================================================================== */
/*  接收回调机制                                                        */
/* ================================================================== */

/** @brief CAN 接收回调函数类型 */
typedef void (*CAN_RxCallback)(uint32_t can_id, uint8_t *data, uint8_t dlc);

/** @brief CAN 接收帧缓存 */
typedef struct {
    CAN_RxHeaderTypeDef rx_header;  /**< HAL 接收帧头（ID、DLC 等） */
    uint8_t             rx_data[8]; /**< 接收数据，固定 8 字节 */
} CAN_RxPacket_t;

/* ================================================================== */
/*  公开 API                                                            */
/* ================================================================== */

/**
 * @brief  CAN 外设初始化（过滤器 + 启动 + 开中断）
 */
uint8_t BSP_CAN_Init(void);
/* 在主循环中执行 Bus-off/错误后的非中断恢复。 */
void BSP_CAN_Task(void);
uint8_t BSP_CAN_IsReady(const CAN_HandleTypeDef *hcan);
uint32_t BSP_CAN_GetErrorCount(const CAN_HandleTypeDef *hcan);

/**
 * @brief  CAN 非阻塞发送一帧数据
 * @param  hcan    CAN 外设句柄（&hcan1 或 &hcan2）
 * @param  pHeader 帧头（ID、DLC、RTR 等）
 * @param  aData   数据指针
 * @return HAL_OK、HAL_BUSY或HAL_ERROR；控制任务中不等待邮箱
 */
HAL_StatusTypeDef BSP_CANTransmit(CAN_HandleTypeDef *hcan,
                                  const CAN_TxHeaderTypeDef *pHeader,
                                  const uint8_t aData[]);

/**
 * @brief  注册 CAN1 接收回调
 * @param  callback 回调函数指针
 */
void CAN1_RegisterRxCallback(CAN_RxCallback callback);

/**
 * @brief  注册 CAN2 接收回调
 * @param  callback 回调函数指针
 */
void CAN2_RegisterRxCallback(CAN_RxCallback callback);

#endif /* BSP_CAN_H */
