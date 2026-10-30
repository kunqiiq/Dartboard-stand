/*
 * 调平系统APP层
 * 负责：
 *
 * - BMI088读取；
 * - 姿态解算；
 * - Balance控制更新；
 * - 自动调平和手动测试的模式切换；
 * - 调平子系统故障隔离；
 * - 看门狗刷新。
 *
 * 控制权规则：
 *
 * AUTO模式：
 *     只有AutoTask可以下发Balance命令。
 *
 * MANUAL模式：
 *     只有ChassisTest可以下发Balance命令。
 *
 * IDLE模式：
 *     不下发新的运动命令，电机保持停止。
 *
 * FAULT模式：
 *     调平电机停止，但其他飞镖盘任务继续运行。
 */

#include "balance_app.h"

#include "auto_task.h"
#include "balance.h"
#include "balance_motor.h"
#include "bmi088.h"
#include "chassic_test.h"
#include "imu_attitude.h"
#include "iwdg.h"

/* ======================== 配置参数 ======================== */

/* IMU数据最大允许更新时间间隔 */
#define BALANCE_IMU_TIMEOUT_MS          20
/* Balance和姿态解算使用的控制周期 */
#define BALANCE_CONTROL_PERIOD_SEC      0.001f
/*
 * 1：上电后自动执行零点标定和完整调平；
 * 0：上电后保持停止，等待外部调用启动。
 */
#define BALANCE_AUTO_START_ENABLED      0

/* ======================== 内部状态 ======================== */

static uint8_t balance_app_ready;
static BalanceApp_Mode_t balance_app_mode;

/* ======================== 内部安全函数 ======================== */

static void BalanceApp_StopMotorOutput(void)
{
    (void)BalanceMotor_Stop();
}

static void BalanceApp_EnterFault(
    Balance_Error_t error)
{
    if (Balance_GetState() != BALANCE_STATE_ERROR)
    {
        Balance_Stop(error);
    }

    BalanceApp_StopMotorOutput();

    balance_app_mode = BALANCE_APP_MODE_FAULT;
}

/* ======================== 初始化 ======================== */

void BalanceApp_Init(void)
{
    balance_app_ready = 0;
    balance_app_mode = BALANCE_APP_MODE_IDLE;

    if (BalanceMotor_Init() == 0)
    {
        balance_app_mode = BALANCE_APP_MODE_FAULT;
        return;
    }

    /*
     * 初始化调平状态机、PID、限位、
     * 编码器累计值和错误状态。
     */
    Balance_Init();

    /*
     * 初始化自动调平任务。
     * 初始化完成后处于AUTO_TASK_IDLE。
     */
    AutoTask_Init();

    /*
     * 初始化BMI088。
     *
     * BMI088_Init内部会执行陀螺仪零偏校准，
     * 校准过程中平台必须保持静止。
     */
    if (BMI088_Init() == 0)
    {
        BalanceApp_EnterFault(BALANCE_ERROR_IMU_OFFLINE);
        return;
    }

    /* 初始化姿态解算 */
    IMU_Attitude_Init();

    balance_app_ready = 1;
    balance_app_mode = BALANCE_APP_MODE_IDLE;

#if BALANCE_AUTO_START_ENABLED

    /*
     * 保留原来的上电自动调平。
     *
     * 如果自动任务启动失败，
     * BalanceApp_StartAuto会切换到FAULT模式。
     */
    (void)BalanceApp_StartAuto();

#endif
}

/* ======================== 自动模式 ======================== */

uint8_t BalanceApp_StartAuto(void)
{
    if (balance_app_ready == 0)
    {
        return 0U;
    }

    /*
     * 如果当前正在运行手动测试向导，
     * 必须先终止手动测试。
     */
    if (balance_app_mode ==
        BALANCE_APP_MODE_MANUAL)
    {
        ChassisTest_ForceStop();
    }
    else
    {
        /*
         * 重新启动自动模式前，
         * 先停止可能存在的旧AutoTask。
         */
        AutoTask_Stop();
    }

    /*
     * 再次直接停止CAN2输出，
     * 防止模式切换期间残留电流。
     */
    BalanceApp_StopMotorOutput();

    AutoTask_Init();
    AutoTask_Start();

    if (AutoTask_GetState() == AUTO_TASK_FAULT)
    {
        BalanceApp_StopMotorOutput();
        balance_app_mode = BALANCE_APP_MODE_FAULT;
        return 0;
    }

    balance_app_mode = BALANCE_APP_MODE_AUTO;
    return 1;
}

/* ======================== 手动测试模式 ======================== */

uint8_t BalanceApp_StartManual(void)
{
    if (balance_app_ready == 0)
    {
        return 0;
    }

    /*
     * 无论当前是否正在自动调平，
     * 都先停止AutoTask。
     *
     * AutoTask_Stop内部会发送
     * BALANCE_COMMAND_ABORT。
     */
    AutoTask_Stop();

    /*
     * 如果此前已经在MANUAL模式，
     * 重新进入page3时先停止旧向导。
     */
    if (balance_app_mode == BALANCE_APP_MODE_MANUAL)
    {
        ChassisTest_ForceStop();
    }

    BalanceApp_StopMotorOutput();

    /*
     * ChassisTest_Init会：
     *
     * - 清除旧测试结果；
     * - 将步骤恢复到0；
     * - 停止当前Balance动作；
     * - 准备显示初始提示。
     */
    ChassisTest_Init();

    balance_app_mode = BALANCE_APP_MODE_MANUAL;

    return 1U;
}

/* ======================== 停止控制 ======================== */

void BalanceApp_Stop(void)
{
    /*
     * 根据当前模式停止对应的上层状态机。
     */
    if (balance_app_mode == BALANCE_APP_MODE_MANUAL)
    {
        ChassisTest_ForceStop();
    }
    else
    {
        AutoTask_Stop();
    }

    /*
     * 上层停止函数已经发送ABORT，
     * 这里再次直接发送CAN2零电流。
     */
    BalanceApp_StopMotorOutput();
    balance_app_mode = BALANCE_APP_MODE_IDLE;
}

uint8_t BalanceApp_StopForFlash(void)
{
    BalanceApp_Stop();
    return BalanceMotor_Stop();
}

/* ======================== 1 ms周期任务 ======================== */

void BalanceApp_Task(void)
{
    uint8_t imu_valid;
    const IMU_Attitude_t *imu;
    uint32_t now;

    /*
     * 刷新CAN2调平电机在线状态。
     *
     * 即使调平APP未准备好，也需要持续更新，
     * 避免底层电机状态长期不刷新。
     */
    BalanceMotor_UpdateTimeout();

    /*
     * 初始化失败时保持调平电机停止，
     * 但仍然刷新看门狗，使其他飞镖盘功能继续运行。
     */
    if (balance_app_ready == 0U)
    {
        BalanceApp_StopMotorOutput();

        (void)HAL_IWDG_Refresh(&hiwdg);
        return;
    }

    imu_valid = BMI088_Update();

    if (imu_valid != 0)
    {
        imu_valid = IMU_Attitude_Update(BALANCE_CONTROL_PERIOD_SEC);
    }

    imu = IMU_Attitude_GetData();
    now = HAL_GetTick();

    /*
     * 检查姿态数据指针、有效标志和更新时间。
     */
    if ((imu == NULL) || (imu->data_valid == 0) || ((uint32_t)(now - imu->last_update_tick) > BALANCE_IMU_TIMEOUT_MS))
    {
        imu_valid = 0;
    }

    /*
     * Balance_Update无论处于AUTO、MANUAL还是IDLE
     * 都应周期调用：
     *
     * - 更新限位；
     * - 更新电机反馈；
     * - 更新编码器累计值；
     * - 执行当前Balance状态。
     */
    if (imu == NULL)
    {
        Balance_Update(
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0);
    }
    else
    {
        Balance_Update(
            imu->pitch,
            imu->roll,
            imu->pitch_rate,
            imu->roll_rate,
            imu_valid);
    }

    /*
     * Balance_Update执行后再更新上层状态机。
     *
     * 新命令将在下一次1 ms Balance_Update中执行。
     */
    switch (balance_app_mode)
    {
        case BALANCE_APP_MODE_AUTO:
        {
            AutoTask_Update();

            if (AutoTask_GetState() == AUTO_TASK_FAULT)
            {
                /*
                 * Balance发生错误时通常已经发送零电流，
                 * 这里再次直接停止。
                 */
                BalanceApp_StopMotorOutput();

                balance_app_mode = BALANCE_APP_MODE_FAULT;
            }

            /*
             * AUTO_TASK_COMPLETE时保持AUTO模式。
             *
             * 此时Balance处于HOLD且电机停止，
             * 不会重新启动自动流程。
             */
            break;
        }

        case BALANCE_APP_MODE_MANUAL:
        {
            /*
             * 手动模式运行page3测试向导。
             *
             * ChassisTest内部根据当前步骤决定是否：
             * - 读取电机状态；
             * - 运行单轴测试；
             * - 读取限位；
             * - 检查IMU；
             * - 执行零点标定或调平。
             */
            ChassisTest_Update(
                now,
                imu,
                imu_valid);
            break;
        }

        case BALANCE_APP_MODE_IDLE:
        {
            /*
             * IDLE模式不启动任何新动作。
             *
             * BalanceApp_Stop已经发送过零电流；
             * Balance处于HOLD或ABORTED状态。
             */
            break;
        }

        case BALANCE_APP_MODE_FAULT:
        default:
        {
            /*
             * FAULT模式持续确保调平电机停止。
             *
             * 不影响PageFire和CAN1发射机构。
             */
            BalanceApp_StopMotorOutput();
            break;
        }
    }

    /*
     * 调平子系统发生错误时仍继续喂狗，
     * 避免单个调平故障导致整个飞镖盘反复重启。
     */
    (void)HAL_IWDG_Refresh(&hiwdg);
}

/* ======================== 状态查询 ======================== */

uint8_t BalanceApp_IsReady(void)
{
    return balance_app_ready;
}

BalanceApp_Mode_t BalanceApp_GetMode(void)
{
    return balance_app_mode;
}

