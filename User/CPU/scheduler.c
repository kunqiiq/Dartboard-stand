/**
 * @file scheduler.c
 * @brief 裸机主循环任务调度器
 *
 * 调度功能：
 *
 * 1. BalanceApp_Task
 *    - 读取BMI088；
 *    - 更新姿态；
 *    - 更新调平控制；
 *    - AUTO模式运行AutoTask；
 *    - MANUAL模式运行ChassisTest。
 *
 * 2. DartboardHMI_Task
 *    - 解析USART1串口屏数据；
 *    - 分发page1、page2、page3和page4触摸事件；
 *    - 保存page2输入并刷新page4最近四条记录。
 *
 * 3. PageBalance_Task
 *    - 检测page3中b_bottom的长按；
 *    - 更新t_step和t_txt。
 *
 * 4. PageFire_Task
 *    - 检查发射页面通信超时；
 *    - 执行失联保护。
 *
 * 5. MotorService_Task
 *    - 更新发射机构电机PID；
 *    - 输出CAN1电机电流。
 */

#include "main.h"
#include "scheduler.h"

#include "bsp_can.h"

#include "DJI_motor.h"
#include "motor_service.h"
#include "pid_service.h"

#include "dartboard_hmi.h"
#include "page_balance.h"
#include "page_fire.h"

#include "balance_app.h"

/* ======================== 调度任务类型 ======================== */

typedef void (*SchedulerTaskFunc_t)(void);

typedef struct
{
    /* 任务入口函数 */
    SchedulerTaskFunc_t task_func;

    /* 任务执行周期，单位ms */
    uint32_t interval_ms;

    /* 上一次执行时刻 */
    uint32_t last_run_time;

    /* 1=启用，0=停用 */
    uint8_t is_running;
} SchedulerTask_t;

/* ======================== 调度任务表 ======================== */

/*
 * 任务顺序说明：
 *
 * BalanceApp_Task首先执行：
 *   ChassisTest产生的新状态和文字可以在同一轮调度中由
 *   PageBalance_Task显示。
 *
 * DartboardHMI_Task随后执行：
 *   可以及时处理触摸事件。
 *
 * PageBalance_Task使用10ms周期：
 *   足够识别1秒长按，同时不会每1ms刷新串口屏。
 */
static SchedulerTask_t scheduler_tasks[] =
{
    /*
     * 任务函数             周期   上次运行   启用
     */
    {BSP_CAN_Task,          10, 0, 1},
    {BalanceApp_Task,        1, 0, 1},
    {DartboardHMI_Task,      1, 0, 1},
    {PageBalance_Task,      10, 0, 1},
    {PageFire_Task,          1, 0, 1},
    {MotorService_Task,      1, 0, 1}
};

/* ======================== 内部辅助函数 ======================== */

static uint32_t Scheduler_GetTaskCount(void)
{
    return (uint32_t)(sizeof(scheduler_tasks) / sizeof(scheduler_tasks[0]));
}

/* ======================== 初始化 ======================== */

void Scheduler_Init(void)
{
    uint32_t index;
    uint32_t now;

    /*
     * BSP初始化
     *
     * 初始化CAN1和CAN2公共接收、发送及回调分发。
     */
    (void)BSP_CAN_Init();

    /*
     * Middleware / Driver初始化
     *
     * PidService：
     * 发射机构的PID服务。
     *
     * RM_Motor：
     * CAN1、CAN2共用的大疆电机驱动。
     *
     * MotorService：
     * 发射机构电机控制服务。
     */
    PidService_Init();
    RM_Motor_Init();
    MotorService_Init();

    /*
     * APP层初始化
     *
     * PageFire_Init：
     * - 3508目标速度归零；
     * - 6020目标速度归零；
     * - 2006停止并失能；
     * - 舵机回到安全位置；
     * - 串口屏链路状态初始化为失联保护。
     */
    PageFire_Init();

    /*
     * PageBalance_Init必须在DartboardHMI_Init之前调用，
     * 保证收到page3事件时内部状态已经初始化。
     */
    PageBalance_Init();

    /*
     * 初始化USART1串口屏：
     * - 注册USART1 DMA接收回调；
     * - 初始化TJC协议解析器；
     * - 默认显示page5（页面ID=0）。
     */
    DartboardHMI_Init();

    /*
     * 初始化CAN2调平子系统：
     *
     * - 注册四个M3508调平电机；
     * - 初始化Balance；
     * - 初始化AutoTask；
     * - 初始化BMI088；
     * - 初始化姿态解算；
     * - 根据BALANCE_AUTO_START_ENABLED决定是否上电自动调平。
     *
     * 进入page3时，PageBalance_Enter会调用
     * BalanceApp_StartManual()切换到手动测试模式。
     */
    BalanceApp_Init();

    /*
     * 将所有任务的时间基准设置为当前时刻。
     * 防止初始化耗时导致任务在主循环中集中补执行。
     */
    now = HAL_GetTick();

    for (index = 0; index < Scheduler_GetTaskCount(); index++)
    {
        scheduler_tasks[index].last_run_time = now;
    }
}

/* ======================== 主循环调度 ======================== */

void Scheduler_Run(void)
{
    uint32_t index;
    uint32_t current_time;

    current_time = HAL_GetTick();

    for (index = 0;index < Scheduler_GetTaskCount();index++)
    {
        SchedulerTask_t *task = &scheduler_tasks[index];

        /*
         * 跳过：
         * - 未启用任务；
         * - 空函数任务；
         * - 周期为0的错误任务。
         */
        if ((task->is_running == 0) || (task->task_func == NULL) || (task->interval_ms == 0))
        {
            continue;
        }

        if ((uint32_t)(current_time - task->last_run_time) >= task->interval_ms)
        {
            /*
             * 在执行任务前更新时间基准。
             *
             * 如果任务内部执行时间较长，不会在返回后立即
             * 连续追赶执行多次。
             */
            task->last_run_time = current_time;

            task->task_func();
        }
    }
}

