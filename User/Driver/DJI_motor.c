/*
 * DJI RM 电机 DAL - 原始数据精简版
 *
 *【修改点 R0】本版本明确不实现：角度累计、跨零、多圈、减速比、
 * 速度换算、安培换算、反转、异常跳变解算。只保留协议原始数据。
 */
#include "DJI_motor.h"
#include <string.h>

#define RM_MOTOR_MIN_DLC (7U)

typedef struct
{
    CAN_HandleTypeDef *hcan;
    RM_Motor_t *motors[RM_CAN_MAX_MOTORS];
    int16_t tx_output[RM_CAN_MAX_MOTORS];
    uint8_t pending_groups;
} RM_CANManager_t;

static RM_CANManager_t g_can_mgr[RM_CAN_BUS_NUM];
static bool g_rm_motor_initialized;

static int8_t RM_GetBusIndex(const CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL) return -1;
    if (hcan->Instance == CAN1) return 0;
    if (hcan->Instance == CAN2) return 1;
    return -1;
}

static int8_t RM_CalculateSlot(uint16_t id, RM_MotorType_e type)
{
    if (type == RM_MOTOR_M3508 || type == RM_MOTOR_M2006) {
        if (id >= 0x201U && id <= 0x208U) return (int8_t)(id - 0x201U);
    } else if (type == RM_MOTOR_GM6020) {
        if (id >= 0x205U && id <= 0x20BU) return (int8_t)(id - 0x205U + 4);
    }
    return -1;
}

static int8_t RM_GetGroupBySlot(int8_t slot)
{
    if (slot >= 0 && slot <= 3) return 0;
    if (slot >= 4 && slot <= 7) return 1;
    if (slot >= 8 && slot <= 10) return 2;
    return -1;
}

static RM_Motor_t *RM_FindMotorByPointer(const RM_CANManager_t *manager,
                                        const RM_Motor_t *motor)
{
    if (manager == NULL || motor == NULL) return NULL;
    for (uint8_t slot = 0U; slot < RM_CAN_MAX_MOTORS; slot++) {
        if (manager->motors[slot] == motor) return manager->motors[slot];
    }
    return NULL;
}

/*【修改点 R4】回调只负责按 CAN ID 找到电机并复制原始字节，不做任何浮点计算。*/
static void RM_DecodeRawOnBus(uint8_t bus, uint32_t can_id,
                              uint8_t *data, uint8_t dlc)
{
    if (bus >= RM_CAN_BUS_NUM || data == NULL || dlc < RM_MOTOR_MIN_DLC) return;
    for (uint8_t slot = 0U; slot < RM_CAN_MAX_MOTORS; slot++) {
        RM_Motor_t *motor = g_can_mgr[bus].motors[slot];
        if (motor != NULL && motor->registered && motor->param.id == can_id) {
            ++motor->feedback_sequence;
            __DMB();
            motor->feedback.raw_angle = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
            motor->feedback.raw_speed = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
            motor->feedback.raw_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
            motor->feedback.temperature = data[6];
            motor->online = true;
            motor->last_update_tick = HAL_GetTick();
            __DMB();
            ++motor->feedback_sequence;
            return;
        }
    }
}

static void RM_CAN1_RxCallback(uint32_t can_id, uint8_t *data, uint8_t dlc)
{
    RM_DecodeRawOnBus(0U, can_id, data, dlc);
}

static void RM_CAN2_RxCallback(uint32_t can_id, uint8_t *data, uint8_t dlc)
{
    RM_DecodeRawOnBus(1U, can_id, data, dlc);
}

/* 依赖：CPU 启动清单须先调用 BSP_CAN_Init()，再调用本函数。 */
void RM_Motor_Init(void)
{
    if (g_rm_motor_initialized) return;
    memset(g_can_mgr, 0, sizeof(g_can_mgr));
    g_can_mgr[0].hcan = &hcan1;
    g_can_mgr[1].hcan = &hcan2;
    CAN1_RegisterRxCallback(RM_CAN1_RxCallback);
    CAN2_RegisterRxCallback(RM_CAN2_RxCallback);
    g_rm_motor_initialized = true;
}

int8_t RM_Motor_Register(RM_Motor_t *motor)
{
    if (motor == NULL || motor->param.hcan == NULL) return RM_MOTOR_ERR_NULL;
    const int8_t bus = RM_GetBusIndex(motor->param.hcan);
    const int8_t slot = RM_CalculateSlot(motor->param.id, motor->param.type);
    if (bus < 0) return RM_MOTOR_ERR_CAN;
    if (slot < 0) return RM_MOTOR_ERR_ID;
    if (g_can_mgr[bus].motors[slot] != NULL) return RM_MOTOR_ERR_DUPLICATE;

    /*【修改点 R5】注册时只清空原始反馈和状态；不创建角度/速度内部状态。*/
    memset(&motor->feedback, 0, sizeof(motor->feedback));
    motor->online = false;
    motor->registered = true;
    motor->last_update_tick = HAL_GetTick();
    motor->feedback_sequence = 0U;
    g_can_mgr[bus].motors[slot] = motor;
    return RM_MOTOR_OK;
}

int8_t RM_Motor_SetCurrent(RM_Motor_t *motor, int16_t current_raw)
{
    if (motor == NULL || motor->param.hcan == NULL) return RM_MOTOR_ERR_NULL;
    const int8_t bus = RM_GetBusIndex(motor->param.hcan);
    const int8_t slot = RM_CalculateSlot(motor->param.id, motor->param.type);
    if (bus < 0) return RM_MOTOR_ERR_CAN;
    if (slot < 0) return RM_MOTOR_ERR_ID;
    if (!motor->registered || RM_FindMotorByPointer(&g_can_mgr[bus], motor) == NULL) {
        return RM_MOTOR_ERR_NOT_REGISTERED;
    }

    /*【修改点 R3】原始值原样写入发送缓存，不做安培换算、限幅或反转。*/
    g_can_mgr[bus].tx_output[slot] = current_raw;
    const int8_t group = RM_GetGroupBySlot(slot);
    if (group < 0) return RM_MOTOR_ERR_FRAME;
    g_can_mgr[bus].pending_groups |= (uint8_t)(1U << (uint8_t)group);
    return RM_MOTOR_OK;
}

int8_t RM_Motor_Relax(RM_Motor_t *motor)
{
    return RM_Motor_SetCurrent(motor, 0);
}

static void RM_PackOutput(const RM_CANManager_t *manager, uint8_t first_slot,
                          uint8_t count, uint8_t data[8])
{
    memset(data, 0, 8U);
    for (uint8_t i = 0U; i < count; i++) {
        const uint16_t value = (uint16_t)manager->tx_output[first_slot + i];
        data[i * 2U] = (uint8_t)(value >> 8);
        data[i * 2U + 1U] = (uint8_t)value;
    }
}

static int8_t RM_FillTxFrame(const RM_CANManager_t *manager, uint8_t group,
                             CAN_TxHeaderTypeDef *header, uint8_t data[8])
{
    if (manager == NULL || header == NULL || data == NULL) return RM_MOTOR_ERR_NULL;
    header->IDE = CAN_ID_STD;
    header->RTR = CAN_RTR_DATA;
    header->DLC = 8U;
    if (group == 0U) {
        header->StdId = 0x200U;
        RM_PackOutput(manager, 0U, 4U, data);
        return RM_MOTOR_OK;
    }
    if (group == 1U) {
        header->StdId = 0x1FFU;
        RM_PackOutput(manager, 4U, 4U, data);
        return RM_MOTOR_OK;
    }
    if (group == 2U) {
        header->StdId = 0x2FFU;
        RM_PackOutput(manager, 8U, 3U, data);
        return RM_MOTOR_OK;
    }
    return RM_MOTOR_ERR_FRAME;
}

//int8_t RM_Motor_Flush(CAN_HandleTypeDef *hcan)
//{
//    const int8_t bus = RM_GetBusIndex(hcan);
//    if (bus < 0) return RM_MOTOR_ERR_CAN;
//    RM_CANManager_t *manager = &g_can_mgr[bus];
//    CAN_TxHeaderTypeDef header;
//    uint8_t data[8];

//    for (uint8_t group = 0U; group < RM_MOTOR_TX_GROUP_COUNT; group++) {
//        if ((manager->pending_groups & (uint8_t)(1U << group)) == 0U) continue;
//        if (RM_FillTxFrame(manager, group, &header, data) != RM_MOTOR_OK) {
//            return RM_MOTOR_ERR_FRAME;
//        }
//        if (BSP_CANTransmit(hcan, &header, data) != HAL_OK) {
//            return RM_MOTOR_ERR_FRAME;
//        }
//        manager->pending_groups &= (uint8_t)~(1U << group);
//    }
//    return RM_MOTOR_OK;
//}

int8_t RM_Motor_Flush(CAN_HandleTypeDef *hcan)
{
    const int8_t bus = RM_GetBusIndex(hcan);

    RM_CANManager_t *manager;
    CAN_TxHeaderTypeDef header;
    HAL_StatusTypeDef status;

    uint8_t data[8];
    uint8_t group;

    if (bus < 0)
    {
        return RM_MOTOR_ERR_CAN;
    }

    manager = &g_can_mgr[bus];

    for (group = 0U;
         group < RM_MOTOR_TX_GROUP_COUNT;
         group++)
    {
        if ((manager->pending_groups &
             (uint8_t)(1U << group)) == 0U)
        {
            continue;
        }

        if (RM_FillTxFrame(
                manager,
                group,
                &header,
                data) != RM_MOTOR_OK)
        {
            return RM_MOTOR_ERR_FRAME;
        }

        status = BSP_CANTransmit(
            hcan,
            &header,
            data
        );

        if (status == HAL_BUSY)
        {
            /*
             * CAN邮箱暂时繁忙。
             *
             * 不清除pending_groups。
             * 下一周期继续尝试发送本组数据。
             */
            return RM_MOTOR_ERR_BUSY;
        }

        if (status != HAL_OK)
        {
            /*
             * HAL_ERROR、HAL_TIMEOUT等属于真正发送错误。
             */
            return RM_MOTOR_ERR_FRAME;
        }

        /*
         * 只有成功提交到CAN发送邮箱后，
         * 才能清除该组待发送标志。
         */
        manager->pending_groups &=
            (uint8_t)~(1U << group);
    }

    return RM_MOTOR_OK;
}

//int8_t RM_Motor_FlushPeriodic(CAN_HandleTypeDef *hcan)
//{
//    const int8_t bus = RM_GetBusIndex(hcan);
//    if (bus < 0) return RM_MOTOR_ERR_CAN;
//    for (uint8_t slot = 0U; slot < RM_CAN_MAX_MOTORS; slot++) {
//        RM_Motor_t *motor = g_can_mgr[bus].motors[slot];
//        if (motor != NULL && motor->registered) {
//            const int8_t group = RM_GetGroupBySlot((int8_t)slot);
//            if (group >= 0) {
//                g_can_mgr[bus].pending_groups |= (uint8_t)(1U << (uint8_t)group);
//            }
//        }
//    }
//    return RM_Motor_Flush(hcan);
//}

int8_t RM_Motor_FlushPeriodic(CAN_HandleTypeDef *hcan)
{
    const int8_t bus = RM_GetBusIndex(hcan);

    uint8_t slot;

    if (bus < 0)
    {
        return RM_MOTOR_ERR_CAN;
    }

    for (slot = 0U;
         slot < RM_CAN_MAX_MOTORS;
         slot++)
    {
        RM_Motor_t *motor =
            g_can_mgr[bus].motors[slot];

        if ((motor != NULL) &&
            motor->registered)
        {
            const int8_t group =
                RM_GetGroupBySlot((int8_t)slot);

            if (group >= 0)
            {
                g_can_mgr[bus].pending_groups |=
                    (uint8_t)(1U << (uint8_t)group);
            }
        }
    }

    return RM_Motor_Flush(hcan);
}

void RM_Motor_CheckTimeout(void)
{
    const uint32_t now = HAL_GetTick();
    for (uint8_t bus = 0U; bus < RM_CAN_BUS_NUM; bus++) {
        for (uint8_t slot = 0U; slot < RM_CAN_MAX_MOTORS; slot++) {
            RM_Motor_t *motor = g_can_mgr[bus].motors[slot];
            if (motor != NULL && motor->registered) {
                uint32_t interrupt_state = __get_PRIMASK();
                __disable_irq();
                if ((uint32_t)(now - motor->last_update_tick) >
                    RM_MOTOR_TIMEOUT_MS) {
                    motor->online = false;
                }
                if (interrupt_state == 0U) {
                    __enable_irq();
                }
            }
        }
    }
}

bool RM_Motor_GetSnapshot(const RM_Motor_t *motor,
                          RM_MotorSnapshot_t *snapshot)
{
    uint32_t sequence_before;
    uint32_t sequence_after;

    if (motor == NULL || snapshot == NULL || !motor->registered) {
        return false;
    }

    for (;;) {
        sequence_before = motor->feedback_sequence;
        if ((sequence_before & 1U) != 0U) {
            continue;
        }

        __DMB();
        snapshot->feedback = motor->feedback;
        snapshot->online = motor->online;
        snapshot->last_update_tick = motor->last_update_tick;
        __DMB();
        sequence_after = motor->feedback_sequence;
        if ((sequence_before == sequence_after) &&
            ((sequence_after & 1U) == 0U)) {
            break;
        }
    }

    return true;
}

bool RM_Motor_IsOnline(const RM_Motor_t *motor)
{
    RM_MotorSnapshot_t snapshot;

    return RM_Motor_GetSnapshot(motor, &snapshot) && snapshot.online;
}
