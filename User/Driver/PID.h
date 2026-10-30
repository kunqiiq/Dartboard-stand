/**
 * @file    PID.h
 * @brief   通用 PID 控制器算法接口
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>

/**
 * @brief PID控制器初始化参数结构体
 * @details 用于在PID_Init时传入初始化参数，配置PID控制器的基本参数
 */
typedef struct{
	float Kp;               ///< 比例系数(Proportional)，用于调节当前误差的影响，值越大响应越快
	float Ki;               ///< 积分系数(Integral)，用于消除稳态误差，值越大消除误差越快
	float Kd;               ///< 微分系数(Derivative)，用于抑制超调和振荡，值越大系统越稳定
	float Setpoint;         ///< 设定值(SP)，期望系统达到的目标值
	float OutputUpperLimit; ///< 输出上限，PID计算输出的最大值限制
	float OutputLowerLimit; ///< 输出下限，PID计算输出的最小值限制
	float ITermUpperLimit;  ///< 积分项上限，防止积分饱和
	float ITermLowerLimit;  ///< 积分项下限，防止积分饱和
	float DefaultOutput;    ///< 默认输出值，在系统启动或复位时的初始输出值
}PID_InitTypeDef;

/**
 * @brief PID控制器运行时结构体
 * @details 包含PID控制器的所有运行时状态，包括参数、状态变量和历史数据
 */
typedef struct{
	PID_InitTypeDef Init;   ///< 初始化参数副本，用于保存初始配置
	uint64_t LastTime;      ///< 上一次执行的时间(微秒)，用于计算时间间隔
	float LastOutput;       ///< 上一次的输出值
	float ITerm;            ///< 积分项累加值，用于积分控制
	float DTerm;            ///< 微分项值，用于微分控制
	float LastInput;        ///< 上一次的输入值(传感器读数)，用于计算微分项
	float LastError;        ///< 上一次的误差值，用于积分计算
	float Kp;               ///< 当前比例系数
	float Ki;               ///< 当前积分系数
	float Kd;               ///< 当前微分系数
	float OutputUpperLimit; ///< 当前输出上限
	float OutputLowerLimit; ///< 当前输出下限
	float ITermUpperLimit;  ///< 积分项上限
	float ITermLowerLimit;  ///< 积分项下限
	float Setpoint;         ///< 当前设定值
	uint8_t Manual;         ///< 手动模式标志：1-手动模式(禁用PID)，0-自动模式(启用PID)
	float ManualOutput;     ///< 手动模式下的输出值
}PID_TypeDef;

/**
 * @brief PID控制器初始化
 * @details 初始化PID控制器，设置所有参数和状态变量为初始值
 * @param PID PID控制器句柄，指向PID_TypeDef结构体
 * @param PID_InitStruct 初始化参数结构体指针，包含Kp、Ki、Kd等配置
 * @return 无
 */
void PID_Init(PID_TypeDef *PID, PID_InitTypeDef *PID_InitStruct);

/**
 * @brief PID控制器使能/禁用
 * @details 启用或禁用PID控制器，禁用后进入手动模式，输出由ManualOutput决定
 * @param PID PID控制器句柄
 * @param NewState 状态：非零-启用PID(自动模式)，0-禁用PID(手动模式)
 * @return 无
 * @note 从禁用切换到启用时，会将积分项ITerm初始化为ManualOutput的值
 */
void PID_Cmd(PID_TypeDef *PID, uint8_t NewState);

/**
 * @brief PID控制器复位
 * @details 重置PID控制器状态，清除积分项，类似于软重启
 * @param PID PID控制器句柄
 * @return 无
 */
void PID_Reset(PID_TypeDef *PID);

/**
 * @brief PID计算(自动计算微分项)
 * @details 执行一次PID运算，根据当前输入值计算输出，适用于大多数场景
 *          此函数会自动计算输入的变化率(dInput/dt)作为微分项
 * @param PID PID控制器句柄
 * @param Input 当前传感器输入值(如温度、速度、位置等)
 * @param Now 当前时间(微秒)，通常使用系统时钟获取
 * @return PID控制器输出值
 */
float PID_Compute1(PID_TypeDef *PID, float Input, uint64_t Now);

/**
 * @brief PID计算(用户提供的微分项)
 * @details 执行一次PID运算，需要用户自行计算并提供输入的变化率
 *          适用于用户需要特殊处理微分项或已有现成微分值的场景
 * @param PID PID控制器句柄
 * @param Input 当前传感器输入值
 * @param dInputDt 输入变化率，即 dInput/dt = (Input(k) - Input(k-1)) / dt
 * @param Now 当前时间(微秒)
 * @return PID控制器输出值
 */
float PID_Compute2(PID_TypeDef *PID, float Input, float dInputDt, uint64_t Now);

/**
 * @brief PID计算(直接传入误差值)
 * @details 当调用方已经计算出 error = setpoint - feedback 时使用。
 *          内部沿用固定周期矩形积分算法，并更新PID运行状态。
 * @param PID PID控制器句柄
 * @param Error 当前控制误差
 * @return PID控制器输出值
 */
float PID_ComputeError(PID_TypeDef *PID, float Error);

/**
 * @brief 修改PID参数
 * @details 动态修改PID控制器的三个参数，可以实现参数自整定或在线调参
 * @param PID PID控制器句柄
 * @param NewKp 新的比例系数
 * @param NewKi 新的积分系数
 * @param NewKd 新的微分系数
 * @return 无
 */
void PID_ChangeTunings(PID_TypeDef *PID, float NewKp, float NewKi, float NewKd);

/**
 * @brief 获取当前PID参数
 * @details 获取当前正在使用的PID参数值
 * @param PID PID控制器句柄
 * @param pKpOut 输出参数，返回当前Kp值
 * @param pKiOut 输出参数，返回当前Ki值
 * @param pKdOut 输出参数，返回当前Kd值
 * @return 无
 */
void PID_GetTunings(PID_TypeDef *PID, float *pKpOut, float *pKiOut, float *pKdOut);

/**
 * @brief 修改设定值
 * @details 动态修改PID控制器的设定值(SP)，改变控制目标
 * @param PID PID控制器句柄
 * @param NewSetpoint 新的设定值
 * @return 无
 */
void PID_ChangeSetpoint(PID_TypeDef *PID, float NewSetpoint);

/**
 * @brief 获取当前设定值
 * @details 获取PID控制器当前的设定值
 * @param PID PID控制器句柄
 * @return 当前设定值
 */
float PID_GetSetpoint(PID_TypeDef *PID);

/**
 * @brief 修改手动模式输出值
 * @details 在手动模式下设置PID输出值，当PID被禁用时生效
 * @param PID PID控制器句柄
 * @param NewValue 手动模式下的输出值
 * @return 无
 */
void PID_ChangeManualOutput(PID_TypeDef *PID, float NewValue);

/**
 * @brief 修改积分项限幅
 * @details 动态修改积分项的上下限，防止积分饱和
 * @param PID PID控制器句柄
 * @param UpperLimit 积分项上限
 * @param LowerLimit 积分项下限
 * @return 无
 */
void PID_ChangeITermLimits(PID_TypeDef *PID, float UpperLimit, float LowerLimit);

/**
 * @brief PID计算(自动计算微分项，离散矩形积分法)
 * @details 执行PID控制算法，使用离散矩形积分法
 *          积分项：ITerm += error; I_output = Ki * ITerm
 *          微分项：D_output = Kd * (error - lastError)
 *          适用于固定周期调用的场景（如 1ms 调度器）
 * @param PID PID控制器句柄
 * @param Input 当前传感器输入值
 * @param now 当前时间(微秒，保留接口兼容，内部不使用)
 * @return PID控制器输出值
 * @note
 *   - 比例项(P): Kp * error
 *   - 积分项(I): Ki * Σerror（离散累加）
 *   - 微分项(D): Kd * Δerror
 */
float PID_Compute1_Rectangle(PID_TypeDef *PID, float Input, uint64_t now);

/**
 * @brief PID计算(用户提供的微分项，离散矩形积分法)
 * @details 与PID_Compute2类似，使用离散矩形积分法
 *          积分项：ITerm += error; I_output = Ki * ITerm
 *          微分项由用户直接提供
 * @param PID PID控制器句柄
 * @param Input 当前传感器输入值
 * @param dInputDt 输入变化率，即 dInput/dt（保留接口兼容，内部不使用）
 * @param now 当前时间(微秒，保留接口兼容，内部不使用）
 * @return PID控制器输出值
 * @note 微分项计算公式: D = Kd * (error - lastError)
 */
float PID_Compute2_Rectangle(PID_TypeDef *PID, float Input, float dInputDt, uint64_t now);

#endif /* PID_H */
