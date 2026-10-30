#ifndef BALANCE_APP_H
#define BALANCE_APP_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{

    BALANCE_APP_MODE_IDLE = 0,      /* 调平系统停止，不运行自动任务或测试向导 */
    BALANCE_APP_MODE_AUTO,      /* 自动执行零点标定和完整调平 */
    BALANCE_APP_MODE_MANUAL,      /* page3手动测试向导 */
    BALANCE_APP_MODE_FAULT      /* 调平子系统初始化或自动任务发生故障 */
} BalanceApp_Mode_t;

/*
 * 初始化调平子系统：
 *
 * - 初始化CAN2四个调平电机；
 * - 初始化Balance；
 * - 初始化AutoTask；
 * - 初始化BMI088；
 * - 初始化姿态解算；
 * - 根据宏配置决定是否启动自动调平。
 */
void BalanceApp_Init(void);

/*
 * 调平系统1 ms周期任务。
 *
 * 在scheduler.c中周期调用。
 */
void BalanceApp_Task(void);

/*
 * 启动自动调平：
 *
 * 1. 停止手动测试；
 * 2. 执行零点标定；
 * 3. 零点标定通过后执行完整调平。
 *
 * 返回：
 * 1 = 启动成功
 * 0 = 启动失败
 */
uint8_t BalanceApp_StartAuto(void);

/*
 * 启动page3手动测试向导：
 *
 * 1. 停止自动调平；
 * 2. 停止当前调平动作；
 * 3. 初始化ChassisTest；
 * 4. 切换为MANUAL模式。
 *
 * 返回：
 * 1 = 启动成功
 * 0 = 调平子系统未准备好
 */
uint8_t BalanceApp_StartManual(void);

/*
 * 停止自动调平或手动测试，
 * 并确保CAN2四个调平电机电流为0。
 */
void BalanceApp_Stop(void);

/* 阻塞式 Flash 操作前停止调平电机，并确认零电流帧发送成功。 */
uint8_t BalanceApp_StopForFlash(void);

/* 调平子系统是否初始化成功 */
uint8_t BalanceApp_IsReady(void);

/* 获取当前调平模式 */
BalanceApp_Mode_t BalanceApp_GetMode(void);

#ifdef __cplusplus
}
#endif

#endif /* BALANCE_APP_H */

