/**
 * @file    motor_service.h
 * @brief   电机实例与通用控制服务。
 *
 * Middleware 创建并注册电机实例。App 通过本模块接口设置目标速度，
 * 并读取已经按减速比换算后的输出轴转速。
 */

#ifndef MOTOR_SERVICE_H
#define MOTOR_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include "DJI_motor.h"
#include "servo.h"
#include "PID.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
	RM_Motor_t *motor;
	PID_TypeDef *pid;
	float reduction_ratio;
	float target_rpm;
	bool enabled;
} MotorServiceNode_t;

/* motor_nodes[] 中各电机节点的固定索引；新增节点时同步追加枚举和配置项。 */
typedef enum
{
	MOTOR_SERVICE_NODE_3508 = 0U,
	MOTOR_SERVICE_NODE_2006,
	MOTOR_SERVICE_NODE_6020,
	MOTOR_SERVICE_NODE_COUNT
} MotorServiceNodeIndex_e;

extern MotorServiceNode_t motor_nodes[];

/* 电机和舵机实例由 Middleware 统一创建，APP 仅作为接口参数使用。 */
//extern RM_Motor_t g_motor_3508;
//extern RM_Motor_t g_motor_2006;
//extern RM_Motor_t g_motor_6020;
extern SERVO_t send_dart;
extern SERVO_t send_dart_2;
extern SERVO_t send_dart_3;
extern SERVO_t send_dart_4;


/* 注册电机节点并启动舵机 PWM；依赖 CPU 已调用 BSP_CAN/RM_Motor/PidService_Init。 */
void MotorService_Init(void);
/* 发布单电机输出轴目标 RPM；速度范围与联动关系由 APP 决定。 */
void MotorService_SetSpeedTarget(RM_Motor_t *motor, float speed_rpm);
/* 设置单电机速度环使能；禁用时由服务复位 PID 并松弛电机。 */
void MotorService_SetEnabled(RM_Motor_t *motor, bool enabled);
/* 立即禁用所有 CAN1 电机并尝试发送零输出。 */
uint8_t MotorService_StopAll(void);
/* 故障在本次上电期间锁存，防止通信异常后自动恢复运动。 */
uint8_t MotorService_HasFault(void);
/* 固定周期执行反馈读取、PID 运算和 CAN 刷新。 */
void MotorService_Task(void);
/* 读取最新输出轴 RPM；无匹配节点返回 0，离线保留最后反馈值。 */
float MotorService_GetSpeedRpm(const RM_Motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_SERVICE_H */
