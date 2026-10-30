/**
 * @file    DJI_motor.h
 * @brief   DJI RM 电机设备抽象层 (DAL) - 原始数据精简版
 * @note    设计目标：
 *          1. 仅解析并保存大疆 RM 电机 8 字节反馈报文中的原始数据，完全剔除减速比、
 *             过零多圈累加、角度弧度制转换、方向镜像及物理安培计算等上层逻辑。
 *          2. 保留双 CAN 总线隔离、11 逻辑槽位管理、组帧合并发送与周期保活机制。
 *          3. 适合追求极致执行速度、RAM 极简占用，或由外部上层算法自主做运动学解算的场景。
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "can.h"
#include "bsp_can.h"
#include <stdbool.h>
#include <stdint.h>

/* ========================================================================== */
/*  常量与配置宏定义                                                           */
/* ========================================================================== */

#define RM_CAN_BUS_NUM          2U      /**< 系统支持的物理 CAN 总线数 (CAN1: 0, CAN2: 1) */
#define RM_CAN_MAX_MOTORS       11U     /**< 单条 CAN 总线上允许挂载的最大大疆电机数 (协议上限 11 节点) */
#define RM_MOTOR_TIMEOUT_MS     100U    /**< 电机通信超时判定阈值 (ms)，超过此时间未收帧则标记离线 */
#define RM_MOTOR_TX_GROUP_COUNT 3U      /**< 大疆 CAN 控制帧组数 (Group 0: 0x200, Group 1: 0x1FF, Group 2: 0x2FF) */

/* ========================================================================== */
/*  枚举定义                                                                  */
/* ========================================================================== */

/**
 * @brief 支持的大疆 RM 电机型号枚举
 * @note  主要用于在底层计算槽位映射 (Slot) 及控制量量程边界：
 *        - M3508 : 电流量程 [-16384, 16384]，对应转矩电流 [-20A, 20A]
 *        - M2006 : 电流量程 [-10000, 10000]，对应转矩电流 [-10A, 10A]
 *        - GM6020: 电压/转矩控制量程 [-30000, 30000]
 */
typedef enum
{
    RM_MOTOR_M3508 = 0,                 /**< M3508 无刷减速电机 (配 C620 电调) */
    RM_MOTOR_M2006,                     /**< M2006 无刷减速电机 (配 C610 电调) */
    RM_MOTOR_GM6020,                    /**< GM6020 直驱无刷云台电机 (内置电调) */
} RM_MotorType_e;

/**
 * @brief 驱动接口执行状态/错误码枚举
 */
typedef enum
{
    RM_MOTOR_OK                 =  0,   /**< 执行成功 */
    RM_MOTOR_ERR_NULL           = -1,   /**< 传入了非法空指针 */
    RM_MOTOR_ERR_CAN            = -2,   /**< 无效的 CAN 外设句柄 (非 CAN1 / CAN2) */
    RM_MOTOR_ERR_ID             = -3,   /**< 电机 CAN ID 超出合法协议范围 (0x201~0x20B) 或与型号不匹配 */
    RM_MOTOR_ERR_DUPLICATE      = -4,   /**< 槽位冲突：同一 CAN 总线上重复注册了相同 ID 的电机 */
    RM_MOTOR_ERR_NOT_REGISTERED = -5,   /**< 电机尚未完成注册即调用了控制接口 */
    RM_MOTOR_ERR_FRAME          = -6,   /**< CAN 组帧异常或发送失败 */
    RM_MOTOR_ERR_BUSY           = -7    //can发送邮箱暂时繁忙
} RM_MotorError_e;

/* ========================================================================== */
/*  结构体定义                                                                */
/* ========================================================================== */

/**
 * @brief 电机静态硬件配置结构体
 * @note  【修改点 R1】仅包含注册与通信寻址所需的必要参数，完全移除了 gear / reverse 等解算标志。
 */
typedef struct
{
    CAN_HandleTypeDef *hcan;            /**< 挂载的物理 CAN 句柄指针 (&hcan1 或 &hcan2) */
    uint16_t           id;              /**< 电机反馈 CAN ID (范围 0x201 ~ 0x20B) */
    RM_MotorType_e     type;            /**< 电机型号 */
} RM_MotorParam_t;

/**
 * @brief 电机原始反馈数据结构体
 * @note  【修改点 R2】1:1 直接映射大疆电调标准 8 字节反馈报文中的前 7 字节原始数值：
 *        - Byte 0~1: 机械角度 (raw_angle)
 *        - Byte 2~3: 转子转速 (raw_speed)
 *        - Byte 4~5: 实际转矩电流 (raw_current)
 *        - Byte 6  : 电机温度 (temperature)
 */
typedef struct
{
    uint16_t raw_angle;                 /**< 13 位编码器原始机械角度，范围 [0, 8191]，对应 0°~360° */
    int16_t  raw_speed;                 /**< 转子侧原始转速，单位 RPM (未除以减速比) */
    int16_t  raw_current;               /**< 实际转矩电流原始 LSB 值 (未换算为安培) */
    uint8_t  temperature;               /**< 电机/电调当前温度，单位 ℃ */
} RM_MotorRawFeedback_t;

/** @brief 从 CAN 中断更新区读取的一致反馈快照。 */
typedef struct
{
    RM_MotorRawFeedback_t feedback;
    bool                  online;
    uint32_t              last_update_tick;
} RM_MotorSnapshot_t;

/**
 * @brief 电机统一管理句柄结构体
 * @note  由用户在应用层定义实例（建议全局或静态存储），生命周期需贯穿整个运行期。
 */
typedef struct
{
    RM_MotorParam_t       param;        /**< 静态配置参数 */
    RM_MotorRawFeedback_t feedback;     /**< 最新一帧原始反馈缓存 */

    /* 运行状态标志 (驱动内部自动维护) */
    bool                  online;       /**< 在线状态标志 (true: 正常通信中; false: 离线超时) */
    bool                  registered;   /**< 注册完成标志 (true: 已绑定至槽位; false: 未注册) */
    uint32_t              last_update_tick; /**< 上一次成功接收到有效报文时的系统时间戳 (HAL Tick) */
    volatile uint32_t     feedback_sequence; /**< 偶数=稳定，奇数=CAN ISR正在更新 */
} RM_Motor_t;

/* ========================================================================== */
/*  公开 API 接口函数声明                                                     */
/* ========================================================================== */

/**
 * @brief  初始化 RM 电机底层驱动系统
 * @note   1. 调用方（CPU）须先 BSP_CAN_Init()，本函数不再初始化 CAN 外设。
 *         2. 静态清空 CAN1 与 CAN2 的所有逻辑槽位及发送缓存。
 *         3. 自动向 bsp_can 注册 CAN1 与 CAN2 的中断接收回调。
 *         4. 支持幂等调用（重复调用不会破坏已有数据）。
 */
void RM_Motor_Init(void);

/**
 * @brief  将电机实例注册至底层 CAN 管理器
 * @param  motor 指向用户定义的电机句柄结构体指针
 * @return int8_t 状态码：
 *         - RM_MOTOR_OK (0)       : 注册成功
 *         - RM_MOTOR_ERR_NULL     : 空指针
 *         - RM_MOTOR_ERR_CAN      : CAN 句柄无效
 *         - RM_MOTOR_ERR_ID       : ID 与型号不匹配
 *         - RM_MOTOR_ERR_DUPLICATE: 该总线该槽位已被占用
 */
int8_t RM_Motor_Register(RM_Motor_t *motor);

/**
 * @brief  设置电机目标控制量（写入待发送槽位缓存）
 * @note   【修改点 R3】直接接收大疆协议原始控制值，不进行安培换算，不改变正负方向：
 *         - M3508 : 输入范围 [-16384, 16384]
 *         - M2006 : 输入范围 [-10000, 10000]
 *         - GM6020: 输入范围 [-30000, 30000]
 *         此接口仅写入内存缓存并标记组待发，需配合 Flush 接口完成实际硬件发送。
 * @param  motor       电机句柄指针
 * @param  current_raw 目标原始 LSB 控制量
 * @return int8_t 状态码 (RM_MOTOR_OK 表示成功)
 */
int8_t RM_Motor_SetCurrent(RM_Motor_t *motor, int16_t current_raw);

/**
 * @brief  使电机松弛（控制量置 0）
 * @param  motor 电机句柄指针
 * @return int8_t 状态码
 */
int8_t RM_Motor_Relax(RM_Motor_t *motor);

/**
 * @brief  按需组帧并发送指定 CAN 总线上所有发生了数值变动的控制帧
 * @note   仅发送 pending_groups 掩码置位的控制组（0x200 / 0x1FF / 0x2FF）。
 *         发送完毕后自动清除对应待发标志，避免重复占用总线带宽。
 * @param  hcan 待刷新的 CAN 外设句柄 (&hcan1 或 &hcan2)
 * @return int8_t 状态码
 */
int8_t RM_Motor_Flush(CAN_HandleTypeDef *hcan);

/**
 * @brief  周期性保活/心跳强制发送
 * @note   无视 pending 状态，无条件扫描该 CAN 上已注册的全部电机，强制向对应控制帧发送当前缓存值。
 *         作用：防止电机控制量长期不变时，因总线静默导致大疆 C610/C620 电调触发断流急停保护。
 *         推荐在 1ms~5ms 控制任务末尾周期性调用。
 * @param  hcan 待刷新的 CAN 外设句柄 (&hcan1 或 &hcan2)
 * @return int8_t 状态码
 */
int8_t RM_Motor_FlushPeriodic(CAN_HandleTypeDef *hcan);

/**
 * @brief  轮询检查所有已注册电机的离线超时状态
 * @note   若当前时间距离 last_update_tick 超过 RM_MOTOR_TIMEOUT_MS (100ms)，
 *         则自动将 online 置为 false。建议放入 1ms 控制循环或定时监控任务中执行。
 */
void RM_Motor_CheckTimeout(void);

/** 将中断中更新的反馈原子地复制到调用者缓冲区。 */
bool RM_Motor_GetSnapshot(const RM_Motor_t *motor,
                          RM_MotorSnapshot_t *snapshot);

/**
 * @brief  查询电机当前是否在线
 * @param  motor 电机句柄指针
 * @return bool true: 在线通信中; false: 已离线或句柄无效
 */
bool RM_Motor_IsOnline(const RM_Motor_t *motor);

#endif /* DJI_MOTOR_H */
