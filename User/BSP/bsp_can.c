#include "bsp_can.h"

#define CAN1_RX_CALLBACK_MAX 4U   /* 回调函数最多可注册函数 */
#define CAN2_Rx_CALLBACK_MAX 2U	 /* CAN2 同时承载电机反馈和视觉桥接反馈 */
//回调函数注册表
static CAN_RxCallback can1_rx_callbacks[CAN1_RX_CALLBACK_MAX] = {0};
static CAN_RxCallback can2_rx_callbacks[CAN2_Rx_CALLBACK_MAX] = {0};
//回调注册索引
static uint8_t can1_rx_count;
static uint8_t can2_rx_count;

static volatile uint8_t can_ready[2];
static volatile uint8_t can_recovery_required[2];
static volatile uint32_t can_error_count[2];

#define BSP_CAN_NOTIFICATIONS \
    (CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF | \
     CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | \
     CAN_IT_LAST_ERROR_CODE)

static int8_t BSP_CAN_GetIndex(const CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL) return -1;
    if (hcan->Instance == CAN1) return 0;
    if (hcan->Instance == CAN2) return 1;
    return -1;
}




/*CAN数据解析回调注册接口*/
void CAN1_RegisterRxCallback(CAN_RxCallback callback)
{
      if (callback != NULL && can1_rx_count < CAN1_RX_CALLBACK_MAX)
      {
          can1_rx_callbacks[can1_rx_count++] = callback;
      }
}
void CAN2_RegisterRxCallback(CAN_RxCallback callback)
{
      if (callback != NULL && can2_rx_count < CAN2_Rx_CALLBACK_MAX)
      {
          can2_rx_callbacks[can2_rx_count++] = callback;
      }
}


/**
 * @brief  配置 CAN 过滤器 — 接受所有消息
 * @param  hcan      CAN 外设句柄
 * @param  bank_idx  过滤器组编号（CAN1 用 0，CAN2 用 14）
 */
static HAL_StatusTypeDef BSP_CANstrainer(CAN_HandleTypeDef *hcan,
                                         uint32_t bank_idx)
{
    CAN_FilterTypeDef filterConfig = {0};
    filterConfig.FilterBank = bank_idx;
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    filterConfig.FilterIdHigh = 0x0000;
    filterConfig.FilterIdLow = 0x0000;
    filterConfig.FilterMaskIdHigh = 0x0000;
    filterConfig.FilterMaskIdLow = 0x0000;
    /* CAN1/CAN2 共用 28 组过滤器：CAN1 使用 0~13，CAN2 使用 14~27。 */
    filterConfig.SlaveStartFilterBank = 14U;
    filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    filterConfig.FilterActivation = ENABLE;
    return HAL_CAN_ConfigFilter(hcan, &filterConfig);
}

static uint8_t BSP_CAN_StartOne(CAN_HandleTypeDef *hcan,
                               uint32_t filter_bank)
{
    int8_t index = BSP_CAN_GetIndex(hcan);

    if (index < 0) return 0U;
    if (BSP_CANstrainer(hcan, filter_bank) != HAL_OK) return 0U;
    if (HAL_CAN_Start(hcan) != HAL_OK) return 0U;
    if (HAL_CAN_ActivateNotification(hcan, BSP_CAN_NOTIFICATIONS) != HAL_OK)
    {
        (void)HAL_CAN_Stop(hcan);
        return 0U;
    }

    can_ready[index] = 1U;
    can_recovery_required[index] = 0U;
    return 1U;
}

uint8_t BSP_CAN_Init(void)
{
    uint8_t can1_ok;
    uint8_t can2_ok;

    can_ready[0] = 0U;
    can_ready[1] = 0U;
    can_recovery_required[0] = 0U;
    can_recovery_required[1] = 0U;
    can_error_count[0] = 0U;
    can_error_count[1] = 0U;

    can1_ok = BSP_CAN_StartOne(&hcan1, 0U);
    can2_ok = BSP_CAN_StartOne(&hcan2, 14U);

    if (can1_ok == 0U) can_recovery_required[0] = 1U;
    if (can2_ok == 0U) can_recovery_required[1] = 1U;

    return ((can1_ok != 0U) && (can2_ok != 0U)) ? 1U : 0U;
}

void BSP_CAN_Task(void)
{
    CAN_HandleTypeDef *handles[2] = {&hcan1, &hcan2};
    uint32_t banks[2] = {0U, 14U};

    for (uint8_t i = 0U; i < 2U; i++)
    {
        if (can_recovery_required[i] == 0U) continue;

        (void)HAL_CAN_Stop(handles[i]);
        can_ready[i] = 0U;
        (void)BSP_CAN_StartOne(handles[i], banks[i]);
    }
}

uint8_t BSP_CAN_IsReady(const CAN_HandleTypeDef *hcan)
{
    int8_t index = BSP_CAN_GetIndex(hcan);
    return (index >= 0) ? can_ready[index] : 0U;
}

uint32_t BSP_CAN_GetErrorCount(const CAN_HandleTypeDef *hcan)
{
    int8_t index = BSP_CAN_GetIndex(hcan);
    return (index >= 0) ? can_error_count[index] : 0U;
}




HAL_StatusTypeDef BSP_CANTransmit(CAN_HandleTypeDef *hcan,
                                  const CAN_TxHeaderTypeDef *pHeader,
                                  const uint8_t aData[])
{
    uint32_t tx_mailbox;

    if ((hcan == NULL) || (pHeader == NULL) || (aData == NULL))
    {
        return HAL_ERROR;
    }

    if (BSP_CAN_IsReady(hcan) == 0U)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return HAL_BUSY;
    }

    return HAL_CAN_AddTxMessage(hcan, pHeader, aData, &tx_mailbox);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    int8_t index = BSP_CAN_GetIndex(hcan);

    if (index < 0) return;

    can_ready[index] = 0U;
    can_recovery_required[index] = 1U;
    can_error_count[index]++;
}

/* ------------------------------------------------------------------ */
/*  HAL 回调函数                                                        */
/* ------------------------------------------------------------------ */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxPacket_t pkt;

    /* 循环清空整个 FIFO，防止 FIFO 满导致丢帧 */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pkt.rx_header, pkt.rx_data) != HAL_OK)
            break;

        if (hcan->Instance == CAN1)
        {
            for (uint8_t i = 0; i < can1_rx_count; i++)
                can1_rx_callbacks[i](pkt.rx_header.StdId,
                                     pkt.rx_data,
                                     (uint8_t)pkt.rx_header.DLC);
        }
        else if (hcan->Instance == CAN2)
        {
            for (uint8_t i = 0; i < can2_rx_count; i++)
                can2_rx_callbacks[i](pkt.rx_header.StdId,
                                     pkt.rx_data,
                                     (uint8_t)pkt.rx_header.DLC);
        }
    }
}


