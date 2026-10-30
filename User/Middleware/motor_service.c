/**
 * @file    motor_service.c
 * @brief   电机实例与通用控制服务实现
 */

#include "motor_service.h"
#include "DJI_motor.h"
#include "pid_service.h"
#include "can.h"
#include "servo.h"
#include "bsp_system.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* ===================== 安全参数 ===================== */

/* 电机过温保护阈值 */
#define MOTOR_SERVICE_MAX_TEMPERATURE_C  80U

/*
 * CAN发送邮箱允许连续繁忙的最长时间。
 *
 * 偶发HAL_BUSY只重试，不锁定故障；
 * 连续繁忙超过该时间后进入安全停止。
 */
#define MOTOR_CAN_BUSY_TIMEOUT_MS        20U

/* ===================== 故障类型 ===================== */

#define MOTOR_FAULT_NONE             0U
#define MOTOR_FAULT_REGISTER         1U
#define MOTOR_FAULT_SERVO_INIT       2U
#define MOTOR_FAULT_INIT_STOP        3U
#define MOTOR_FAULT_OVER_TEMP        4U
#define MOTOR_FAULT_SET_CURRENT      5U
#define MOTOR_FAULT_CAN_FLUSH        6U

#define MOTOR_FAULT_NODE_UNKNOWN     0xFFU

/* ===================== 调试变量 ===================== */

/* 电机服务故障状态 */
volatile uint8_t motor_fault_debug = 0U;

/* 3508电机状态 */
volatile uint8_t motor_3508_online_debug = 0U;
volatile uint8_t motor_3508_enabled_debug = 0U;

volatile float motor_3508_target_debug = 0.0f;
volatile float motor_3508_actual_debug = 0.0f;

/* MotorService_Task运行次数 */
volatile uint32_t motor_task_count_debug = 0U;

/* 第一次故障原因 */
volatile uint8_t motor_fault_reason =
    MOTOR_FAULT_NONE;

/* 第一次故障节点 */
volatile uint8_t motor_fault_node =
    MOTOR_FAULT_NODE_UNKNOWN;

/* 第一次故障底层错误码 */
volatile int32_t motor_fault_error = 0;

/* 第一次故障发生时间 */
volatile uint32_t motor_fault_tick = 0U;

/* 故障记录函数调用次数 */
volatile uint32_t motor_fault_count = 0U;

/* 当前各电机在线状态 */
volatile uint8_t motor_online_debug[3] =
{
    0U,
    0U,
    0U
};

/* 当前各电机使能状态 */
volatile uint8_t motor_enabled_debug[3] =
{
    0U,
    0U,
    0U
};

/* 当前各电机目标转速 */
volatile float motor_target_debug[3] =
{
    0.0f,
    0.0f,
    0.0f
};

/* 当前各电机实际转速 */
volatile float motor_actual_debug[3] =
{
    0.0f,
    0.0f,
    0.0f
};

/* 最近一次CAN周期发送返回值 */
volatile int8_t motor_can_status_debug =
    RM_MOTOR_OK;

/* HAL_BUSY累计出现次数 */
volatile uint32_t motor_can_busy_count_debug =
    0U;

/* 连续繁忙超时次数 */
volatile uint32_t
    motor_can_busy_timeout_count_debug = 0U;

/* ===================== 电机实例 ===================== */

static RM_Motor_t g_motor_3508 =
{
    .param.id = 0x204U,
    .param.hcan = &hcan1,
    .param.type = RM_MOTOR_M3508
};

static RM_Motor_t g_motor_2006 =
{
    .param.id = 0x201U,
    .param.hcan = &hcan1,
    .param.type = RM_MOTOR_M2006
};

static RM_Motor_t g_motor_6020 =
{
    .param.id = 0x205U,
    .param.hcan = &hcan1,
    .param.type = RM_MOTOR_GM6020
};

/* ===================== 舵机实例 ===================== */

SERVO_t send_dart =
{
    .pwm_ch = BSP_PWM_TIM1_CH1,
    .min_duty = 0.025f,
    .max_duty = 0.118f
};

SERVO_t send_dart_2 =
{
    .pwm_ch = BSP_PWM_TIM1_CH2,
    .min_duty = 0.025f,
    .max_duty = 0.10f
};

SERVO_t send_dart_3 =
{
    .pwm_ch = BSP_PWM_TIM1_CH3,
    .min_duty = 0.025f,
    .max_duty = 0.10f
};

SERVO_t send_dart_4 =
{
    .pwm_ch = BSP_PWM_TIM1_CH4,
    .min_duty = 0.025f,
    .max_duty = 0.10f
};

/* ===================== 电机节点 ===================== */

/*
 * reduction_ratio:
 *
 * M3508 : 3591 / 187
 * M2006 : 36
 * GM6020: 1
 */

MotorServiceNode_t motor_nodes[] =
{
    {
        &g_motor_3508,
        &g_pid_gather[PID_3508],
        3591.0f / 187.0f,
        0.0f,
        false
    },

    {
        &g_motor_2006,
        &g_pid_gather[PID_2006],
        36.0f,
        0.0f,
        false
    },

    {
        &g_motor_6020,
        &g_pid_gather[PID_6020],
        1.0f,
        0.0f,
        false
    }
};

/* ===================== 私有变量 ===================== */

/* 全局故障锁定 */
static uint8_t motor_service_fault = 0U;

/* CAN邮箱连续繁忙状态 */
static uint8_t motor_can_busy_active = 0U;

/* CAN邮箱开始连续繁忙的时间 */
static uint32_t motor_can_busy_start_tick = 0U;

/* ===================== 故障记录函数 ===================== */

/**
 * @brief 记录电机服务故障
 *
 * @param reason 故障类型
 * @param node   故障节点编号
 * @param error  底层错误码或附加信息
 *
 * @note
 * 只保存第一次故障原因。
 * 后续故障不会覆盖第一次故障原因。
 */
static void MotorService_RecordFault(
    uint8_t reason,
    uint8_t node,
    int32_t error)
{
    uint32_t state;

    state = BSP_System_Lock();

    motor_fault_count++;

    if (motor_fault_reason == MOTOR_FAULT_NONE)
    {
        motor_fault_reason = reason;
        motor_fault_node = node;
        motor_fault_error = error;
        motor_fault_tick = HAL_GetTick();
    }

    /* 锁定全局故障 */
    motor_service_fault = 1U;

    BSP_System_Unlock(state);
}

/* ===================== 查找电机节点 ===================== */

static MotorServiceNode_t *MotorService_FindNode(
    const RM_Motor_t *motor)
{
    uint8_t i;

    for (i = 0U;
         i < MOTOR_SERVICE_NODE_COUNT;
         i++)
    {
        if (motor_nodes[i].motor == motor)
        {
            return &motor_nodes[i];
        }
    }

    return NULL;
}

/* ===================== 获取实际转速 ===================== */

static float MotorService_ReadNodeSpeedRpm(
    const MotorServiceNode_t *node)
{
    RM_MotorSnapshot_t snapshot;

    if (node == NULL)
    {
        return 0.0f;
    }

    if (!(node->reduction_ratio > 0.0f))
    {
        return 0.0f;
    }

    if (!RM_Motor_GetSnapshot(
            node->motor,
            &snapshot))
    {
        return 0.0f;
    }

    return
        (float)snapshot.feedback.raw_speed /
        node->reduction_ratio;
}

/* ===================== 电机服务初始化 ===================== */

void MotorService_Init(void)
{
    uint8_t i;

    /* 清除故障锁定 */
    motor_service_fault = 0U;

    /* 清除CAN连续繁忙状态 */
    motor_can_busy_active = 0U;
    motor_can_busy_start_tick = 0U;

    /* 清除调试变量 */
    motor_fault_debug = 0U;

    motor_fault_reason = MOTOR_FAULT_NONE;
    motor_fault_node = MOTOR_FAULT_NODE_UNKNOWN;
    motor_fault_error = 0;
    motor_fault_tick = 0U;
    motor_fault_count = 0U;

    motor_task_count_debug = 0U;

    motor_can_status_debug = RM_MOTOR_OK;
    motor_can_busy_count_debug = 0U;
    motor_can_busy_timeout_count_debug = 0U;

    for (i = 0U;
         i < MOTOR_SERVICE_NODE_COUNT;
         i++)
    {
        motor_online_debug[i] = 0U;
        motor_enabled_debug[i] = 0U;

        motor_target_debug[i] = 0.0f;
        motor_actual_debug[i] = 0.0f;
    }

    motor_3508_online_debug = 0U;
    motor_3508_enabled_debug = 0U;
    motor_3508_target_debug = 0.0f;
    motor_3508_actual_debug = 0.0f;

    /* 注册电机 */
    for (i = 0U;
         i < MOTOR_SERVICE_NODE_COUNT;
         i++)
    {
        int8_t status;

        status = RM_Motor_Register(
            motor_nodes[i].motor
        );

        if (status != RM_MOTOR_OK)
        {
            MotorService_RecordFault(
                MOTOR_FAULT_REGISTER,
                i,
                status
            );
        }
    }

    /* 初始化四路舵机 */

    if (SERVO_Init(&send_dart) != HAL_OK)
    {
        MotorService_RecordFault(
            MOTOR_FAULT_SERVO_INIT,
            0U,
            -1
        );
    }

    if (SERVO_Init(&send_dart_2) != HAL_OK)
    {
        MotorService_RecordFault(
            MOTOR_FAULT_SERVO_INIT,
            1U,
            -1
        );
    }

    if (SERVO_Init(&send_dart_3) != HAL_OK)
    {
        MotorService_RecordFault(
            MOTOR_FAULT_SERVO_INIT,
            2U,
            -1
        );
    }

    if (SERVO_Init(&send_dart_4) != HAL_OK)
    {
        MotorService_RecordFault(
            MOTOR_FAULT_SERVO_INIT,
            3U,
            -1
        );
    }

    /* 初始化时停止全部电机 */
    if (MotorService_StopAll() == 0U)
    {
        MotorService_RecordFault(
            MOTOR_FAULT_INIT_STOP,
            MOTOR_FAULT_NODE_UNKNOWN,
            -1
        );
    }

    motor_fault_debug = motor_service_fault;
}

/* ===================== 设置目标速度 ===================== */

void MotorService_SetSpeedTarget(
    RM_Motor_t *motor,
    float speed_rpm)
{
    MotorServiceNode_t *node;
    uint32_t state;

    node = MotorService_FindNode(motor);

    if (node == NULL)
    {
        return;
    }

    if (!isfinite(speed_rpm))
    {
        return;
    }

    state = BSP_System_Lock();

    /*
     * 只修改目标速度。
     * 不自动使能电机。
     */
    node->target_rpm = speed_rpm;

    BSP_System_Unlock(state);
}

/* ===================== 设置电机使能 ===================== */

void MotorService_SetEnabled(
    RM_Motor_t *motor,
    bool enabled)
{
    MotorServiceNode_t *node;
    uint32_t state;

    node = MotorService_FindNode(motor);

    if (node == NULL)
    {
        return;
    }

    /*
     * 故障锁定后禁止重新使能。
     * 必须复位主控或实现经过确认的故障恢复流程。
     */
    if (enabled &&
        (motor_service_fault != 0U))
    {
        return;
    }

    state = BSP_System_Lock();

    node->enabled = enabled;

    BSP_System_Unlock(state);
}

/* ===================== 停止所有电机 ===================== */

uint8_t MotorService_StopAll(void)
{
    uint8_t result = 1U;
    uint8_t i;
    uint32_t state;
    int8_t flush_status;

    /* 清除目标速度和使能 */
    state = BSP_System_Lock();

    for (i = 0U;
         i < MOTOR_SERVICE_NODE_COUNT;
         i++)
    {
        motor_nodes[i].target_rpm = 0.0f;
        motor_nodes[i].enabled = false;
    }

    BSP_System_Unlock(state);

    /* 清除PID并设置零电流 */
    for (i = 0U;
         i < MOTOR_SERVICE_NODE_COUNT;
         i++)
    {
        int8_t status;

        PID_Reset(motor_nodes[i].pid);

        status = RM_Motor_SetCurrent(
            motor_nodes[i].motor,
            0
        );

        if (status != RM_MOTOR_OK)
        {
            result = 0U;
        }
    }

    /*
     * 尝试立即发送零电流。
     *
     * 如果返回BUSY，待发送标志仍然保留，
     * MotorService_Task下一周期会继续发送。
     */
    flush_status = RM_Motor_Flush(&hcan1);

    if ((flush_status != RM_MOTOR_OK) &&
        (flush_status != RM_MOTOR_ERR_BUSY))
    {
        result = 0U;
    }

    return result;
}

/* ===================== 获取故障状态 ===================== */

uint8_t MotorService_HasFault(void)
{
    return motor_service_fault;
}

/* ===================== 电机周期控制任务 ===================== */

void MotorService_Task(void)
{
    uint8_t i;
    uint8_t output_ok = 1U;

    int8_t can_status;

    uint32_t now;

    /* 任务运行计数 */
    motor_task_count_debug++;

    /* 检查电机反馈超时 */
    RM_Motor_CheckTimeout();

    /* 更新故障调试状态 */
    motor_fault_debug = motor_service_fault;

    /* 遍历全部电机 */
    for (i = 0U;
         i < MOTOR_SERVICE_NODE_COUNT;
         i++)
    {
        MotorServiceNode_t *node =
            &motor_nodes[i];

        RM_MotorSnapshot_t snapshot;

        float actual;
        float target_speed;

        bool enabled;
        bool online;

        uint32_t state;

        int16_t output_current;

        /* 读取实际速度 */
        actual =
            MotorService_ReadNodeSpeedRpm(node);

        /* 读取目标速度和使能状态 */
        state = BSP_System_Lock();

        target_speed = node->target_rpm;
        enabled = node->enabled;

        BSP_System_Unlock(state);

        /* 获取在线状态 */
        online =
            RM_Motor_IsOnline(node->motor);

        /* 更新通用调试变量 */
        motor_online_debug[i] =
            online ? 1U : 0U;

        motor_enabled_debug[i] =
            enabled ? 1U : 0U;

        motor_target_debug[i] =
            target_speed;

        motor_actual_debug[i] =
            actual;

        /* 更新3508专用调试变量 */
        if (i == MOTOR_SERVICE_NODE_3508)
        {
            motor_3508_online_debug =
                online ? 1U : 0U;

            motor_3508_enabled_debug =
                enabled ? 1U : 0U;

            motor_3508_target_debug =
                target_speed;

            motor_3508_actual_debug =
                actual;
        }

        /* ================= 温度保护 ================= */

        if (RM_Motor_GetSnapshot(
                node->motor,
                &snapshot))
        {
            if (snapshot.feedback.temperature >=
                MOTOR_SERVICE_MAX_TEMPERATURE_C)
            {
                MotorService_RecordFault(
                    MOTOR_FAULT_OVER_TEMP,
                    i,
                    (int32_t)
                        snapshot.feedback.temperature
                );
            }
        }

        /* ================= 停止条件 ================= */

        if ((motor_service_fault != 0U) ||
            (!enabled) ||
            (!online))
        {
            PID_Reset(node->pid);

            /*
             * 故障、失能或离线时发送零电流。
             */
            if (RM_Motor_SetCurrent(
                    node->motor,
                    0) != RM_MOTOR_OK)
            {
                output_ok = 0U;

                MotorService_RecordFault(
                    MOTOR_FAULT_SET_CURRENT,
                    i,
                    -1
                );
            }
        }
        else
        {
            /* ================= 速度闭环 ================= */

            PID_ChangeSetpoint(
                node->pid,
                target_speed
            );

            output_current =
                (int16_t)PID_Compute1_Rectangle(
                    node->pid,
                    actual,
                    0U
                );

            if (RM_Motor_SetCurrent(
                    node->motor,
                    output_current) !=
                RM_MOTOR_OK)
            {
                output_ok = 0U;

                MotorService_RecordFault(
                    MOTOR_FAULT_SET_CURRENT,
                    i,
                    -1
                );
            }
        }
    }

    /* ================= CAN周期发送 ================= */

    can_status =
        RM_Motor_FlushPeriodic(&hcan1);

    motor_can_status_debug = can_status;

    now = HAL_GetTick();

    if (can_status == RM_MOTOR_OK)
    {
        /*
         * 发送恢复成功。
         * 清除连续繁忙计时。
         */
        motor_can_busy_active = 0U;
        motor_can_busy_start_tick = 0U;
    }
    else if (can_status == RM_MOTOR_ERR_BUSY)
    {
        /*
         * 邮箱暂时繁忙。
         *
         * 不立即锁定全局故障，
         * 保留待发送帧，下一周期重试。
         */
        motor_can_busy_count_debug++;

        if (motor_can_busy_active == 0U)
        {
            motor_can_busy_active = 1U;
            motor_can_busy_start_tick = now;
        }
        else if ((uint32_t)(
                     now -
                     motor_can_busy_start_tick) >=
                 MOTOR_CAN_BUSY_TIMEOUT_MS)
        {
            /*
             * 连续繁忙超过安全时间。
             * 此时才按真实发送故障处理。
             */
            output_ok = 0U;

            if (motor_service_fault == 0U)
            {
                motor_can_busy_timeout_count_debug++;

                MotorService_RecordFault(
                    MOTOR_FAULT_CAN_FLUSH,
                    MOTOR_FAULT_NODE_UNKNOWN,
                    can_status
                );
            }
        }
    }
    else
    {
        /*
         * 非BUSY错误属于真正的CAN发送错误。
         */
        motor_can_busy_active = 0U;
        motor_can_busy_start_tick = 0U;

        output_ok = 0U;

        MotorService_RecordFault(
            MOTOR_FAULT_CAN_FLUSH,
            MOTOR_FAULT_NODE_UNKNOWN,
            can_status
        );
    }

    /* ================= 故障停止 ================= */

    if (output_ok == 0U)
    {
        /*
         * 目标速度清零、所有电机失能，
         * 并将零电流写入待发送缓存。
         */
        (void)MotorService_StopAll();
    }

    motor_fault_debug = motor_service_fault;
}

/* ===================== 获取实际速度 ===================== */

float MotorService_GetSpeedRpm(
    const RM_Motor_t *motor)
{
    MotorServiceNode_t *node;

    node = MotorService_FindNode(motor);

    return
        MotorService_ReadNodeSpeedRpm(node);
}

