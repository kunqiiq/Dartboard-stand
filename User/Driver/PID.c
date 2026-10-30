/**
 * @file    PID.c
 * @brief   通用 PID 控制器算法实现
 */

#include "PID.h"

/**
 * @brief 无效时间标记
 * @details 用于表示PID控制器尚未执行过，用于判断是否为首次计算
 */
#define INVALID_TICK 0xffffffffffffffff

/**
 * @brief PID控制器初始化
 * @details 将初始化参数复制到PID控制器结构体中，并初始化所有运行时状态
 * @param PID PID控制器句柄
 * @param PID_InitStruct 初始化参数结构体指针
 * @return 无
 * @note 此函数会初始化设定值、参数、输出限制、积分项初始值等所有状态
 */
void PID_Init(PID_TypeDef *PID, PID_InitTypeDef *PID_InitStruct)
{
	// 复制初始化参数
	PID->Init = *PID_InitStruct;

	// 初始化设定值为设定目标值
	PID->Setpoint = PID->Init.Setpoint;
	// 初始化PID参数
	PID->Kp = PID->Init.Kp;
	PID->Ki = PID->Init.Ki;
	PID->Kd = PID->Init.Kd;
	// 初始化输出限制
	PID->OutputLowerLimit = PID->Init.OutputLowerLimit;
	PID->OutputUpperLimit = PID->Init.OutputUpperLimit;
	// 初始化积分项限制
	PID->ITermUpperLimit = PID->Init.ITermUpperLimit;
	PID->ITermLowerLimit = PID->Init.ITermLowerLimit;
	// 初始化积分项为默认输出值(启动时提供一个基础输出)
	PID->ITerm = PID->Init.DefaultOutput;
	// 标记为尚未执行过(无效时间)
	PID->LastTime = INVALID_TICK;
	// 初始化上次输入为0
	PID->LastInput = 0;
	// 初始化手动输出为默认输出值
	PID->ManualOutput = PID->Init.DefaultOutput;
	// 初始化为自动模式(非手动)
	PID->Manual = 0;
}

/**
 * @brief PID控制器复位
 * @details 重置PID控制器状态，清除积分项和历史数据，使控制器回到初始状态
 * @param PID PID控制器句柄
 * @return 无
 * @note 复位后会清除积分项累加，但保留Kp、Ki、Kd等参数值
 */
void PID_Reset(PID_TypeDef *PID)
{
	// 重置时间标记为无效
	PID->LastTime = INVALID_TICK;
	// 重置积分项为默认输出值
	PID->ITerm = PID->Init.DefaultOutput;
}

/**
 * @brief PID计算(自动计算微分项)
 * @details 执行标准的PID控制算法，根据当前输入值计算输出
 *          使用位置式PID算法，包含比例、积分、微分三个环节
 * @param PID PID控制器句柄
 * @param Input 当前传感器输入值
 * @param now 当前时间(微秒)
 * @return PID控制器输出值
 * @note 
 *   - 比例项(P): Kp * error
 *   - 积分项(I): Ki * ∫error dt (使用梯形积分)
 *   - 微分项(D): Kd * d(Input)/dt
 */
float PID_Compute1(PID_TypeDef *PID, float Input, uint64_t now)
{
	float Output;
	float timeChange = 0;
	float error;
	
	// 计算误差：设定值 - 当前输入值
	error	= PID->Setpoint - Input;
	// 计算时间变化量(微秒转换为秒)
	timeChange = (now - PID->LastTime) * 1.0e-6f;
	
	// 计算比例项 P = Kp * error
	Output = PID->Kp * error;
	
	// 如果不是首次计算(已有上一次的时间记录)，则计算积分和微分项
	if(PID->LastTime != INVALID_TICK) 
	{
		// 计算微分项 D = Kd * d(Input)/dt
		// 微分项使用输入的变化率，而非误差的变化率(避免设定值突变导致的冲击)
		if(PID->Kd != 0)
		{
			PID->DTerm = PID->Kd * (PID->LastInput - Input) / timeChange;
			Output += PID->DTerm;
		}
		
		// 计算积分项 I = Ki * ∫error dt
		// 使用梯形积分法: ∫error dt ≈ (error(k) + error(k-1)) / 2 * dt
		if(PID->Ki != 0)
		{
			PID->ITerm += PID->Ki * (error + PID->LastError) * 0.5f * timeChange;

			// 积分项独立限幅，防止积分饱和
			if(PID->ITerm > PID->ITermUpperLimit)
			{
				PID->ITerm = PID->ITermUpperLimit;
			}
			else if(PID->ITerm < PID->ITermLowerLimit)
			{
				PID->ITerm = PID->ITermLowerLimit;
			}

			// 累加积分项到输出
			Output += PID->ITerm;
		}
	}

	// 输出限幅(限制在[OutputLowerLimit, OutputUpperLimit]范围内)
	if(Output > PID->OutputUpperLimit)
	{
		Output = PID->OutputUpperLimit;
	}
	else if(Output < PID->OutputLowerLimit)
	{
		Output = PID->OutputLowerLimit;
	}
	
	// 如果处于手动模式，直接使用手动输出值覆盖计算结果
	if(PID->Manual != 0) 
	{
		Output = PID->ManualOutput;
	}
	
	// 保存当前状态，供下次计算使用
	PID->LastInput = Input;      // 保存当前输入值
	PID->LastTime = now;        // 保存当前时间
	PID->LastError = error;     // 保存当前误差
	PID->LastOutput = Output;   // 保存当前输出值
	
	return Output;
}

/**
 * @brief PID计算(用户提供的微分项)
 * @details 与PID_Compute1类似，但微分项由用户直接提供，适用于：
 *          - 用户需要特殊处理微分项的场景
 *          - 已经计算好输入变化率的场景
 *          - 使用外部传感器提供变化率的场景
 * @param PID PID控制器句柄
 * @param Input 当前传感器输入值
 * @param dInputDt 输入变化率，即 dInput/dt
 * @param now 当前时间(微秒)
 * @return PID控制器输出值
 * @note 微分项计算公式: D = -Kd * dInputDt (负号是因为微分项应该抵抗输入的变化)
 */
float PID_Compute2(PID_TypeDef *PID, float Input, float dInputDt, uint64_t now)
{
	float Output;
	float timeChange = 0;
	float error;
	
	// 计算误差
	error	= PID->Setpoint - Input;
	// 计算时间变化量
	timeChange = (now - PID->LastTime) * 1.0e-6f;
	
	// 计算比例项
	Output = PID->Kp * error;
	
	// 非首次计算时计算积分和微分项
	if(PID->LastTime != INVALID_TICK) 
	{
		// 直接使用用户提供的输入变化率计算微分项
		if(PID->Kd != 0)
		{
			PID->DTerm = -PID->Kd * dInputDt;
			Output += PID->DTerm;
		}
		
		// 计算积分项(与PID_Compute1相同)
		if(PID->Ki != 0)
		{
			PID->ITerm += PID->Ki * (error + PID->LastError) * 0.5f * timeChange;

			// 积分项独立限幅
			if(PID->ITerm > PID->ITermUpperLimit)
			{
				PID->ITerm = PID->ITermUpperLimit;
			}
			else if(PID->ITerm < PID->ITermLowerLimit)
			{
				PID->ITerm = PID->ITermLowerLimit;
			}

			Output += PID->ITerm;
		}
	}
	
	// 输出限幅
	if(Output > PID->OutputUpperLimit)
	{
		Output = PID->OutputUpperLimit;
	}
	else if(Output < PID->OutputLowerLimit)
	{
		Output = PID->OutputLowerLimit;
	}
	
	// 手动模式处理
	if(PID->Manual != 0) 
	{
		Output = PID->ManualOutput;
	}
	
	// 保存当前状态
	PID->LastInput = Input;
	PID->LastTime = now;
	PID->LastError = error;
	PID->LastOutput = Output;
	
	return Output;
}

/**
 * @brief PID计算(直接传入误差值)
 * @details 用于调用方已经在PID库外部计算好误差的控制环。
 *          算法与PID_Compute1_Rectangle保持一致，不需要伪造设定值和输入值。
 * @param PID PID控制器句柄
 * @param Error 当前控制误差
 * @return PID控制器输出值
 */
float PID_ComputeError(PID_TypeDef *PID, float Error)
{
	float Output;
	float dError;

	dError = Error - PID->LastError;

	PID->ITerm += Error;

	if(PID->ITerm > PID->ITermUpperLimit)
	{
		PID->ITerm = PID->ITermUpperLimit;
	}
	else if(PID->ITerm < PID->ITermLowerLimit)
	{
		PID->ITerm = PID->ITermLowerLimit;
	}

	Output = PID->Kp * Error;
	Output += PID->Ki * PID->ITerm;

	if(PID->Kd != 0)
	{
		PID->DTerm = PID->Kd * dError;
		Output += PID->DTerm;
	}

	if(Output > PID->OutputUpperLimit)
	{
		Output = PID->OutputUpperLimit;
	}
	else if(Output < PID->OutputLowerLimit)
	{
		Output = PID->OutputLowerLimit;
	}

	if(PID->Manual != 0)
	{
		Output = PID->ManualOutput;
	}

	PID->LastInput = PID->Setpoint - Error;
	PID->LastError = Error;
	PID->LastOutput = Output;

	return Output;
}

/**
 * @brief 修改PID参数
 * @details 动态修改PID控制器的三个参数，用于在线调参或实现自适应控制
 * @param PID PID控制器句柄
 * @param NewKp 新的比例系数Kp
 * @param NewKi 新的积分系数Ki
 * @param NewKd 新的微分系数Kd
 * @return 无
 */
void PID_ChangeTunings(PID_TypeDef *PID, float NewKp, float NewKi, float NewKd)
{
	PID->Kp = NewKp;
	PID->Ki = NewKi;
	PID->Kd = NewKd;
}

/**
 * @brief 修改设定值
 * @details 动态修改PID控制器的目标值，实现对被控对象的期望控制
 * @param PID PID控制器句柄
 * @param NewSetpoint 新的设定值
 * @return 无
 */
void PID_ChangeSetpoint(PID_TypeDef *PID, float NewSetpoint)
{
	PID->Setpoint = NewSetpoint;
}

/**
 * @brief 获取当前设定值
 * @details 返回PID控制器当前的设定值
 * @param PID PID控制器句柄
 * @return 当前设定值
 */
float PID_GetSetpoint(PID_TypeDef *PID)
{
	return PID->Setpoint;
}

/**
 * @brief 获取当前PID参数
 * @details 获取当前正在使用的PID参数值
 * @param PID PID控制器句柄
 * @param pKpOut 输出参数，用于返回Kp值
 * @param pKiOut 输出参数，用于返回Ki值
 * @param pKdOut 输出参数，用于返回Kd值
 * @return 无
 */
void PID_GetTunings(PID_TypeDef *PID, float *pKpOut, float *pKiOut, float *pKdOut)
{
	*pKpOut = PID->Kp;
	*pKiOut = PID->Ki;
	*pKdOut = PID->Kd;
}

/**
 * @brief PID控制器使能/禁用
 * @details 启用或禁用PID控制器，禁用后进入手动模式
 *          手动模式下，PID计算输出被忽略，而是使用手动设置的输出值
 * @param PID PID控制器句柄
 * @param NewState 状态：非零-启用(自动模式)，0-禁用(手动模式)
 * @return 无
 * @note 当从禁用切换到启用时，会将积分项初始化为手动输出值，实现无扰切换
 */
void PID_Cmd(PID_TypeDef *PID, uint8_t NewState)
{
	// 如果是从手动模式切换到自动模式，初始化积分项为当前手动输出值
	// 这样可以实现无扰切换，避免输出突变
	if(PID->Manual == 1 && NewState == 1)
	{
		PID->ITerm = PID->ManualOutput;
		// 限幅处理
		if(PID->ITerm > PID->OutputUpperLimit)
		{
			PID->ITerm = PID->OutputUpperLimit;
		}
		
		if(PID->ITerm < PID->OutputLowerLimit)
		{
			PID->ITerm = PID->OutputLowerLimit;
		}
	}
	
	// 设置手动模式标志：NewState为0时Manual为1(手动模式)，否则为0(自动模式)
	PID->Manual = NewState == 0 ? 1 : 0;
}

/**
 * @brief 修改手动模式输出值
 * @details 在手动模式下设置PID输出值，当PID被禁用时生效
 *        常用于：紧急停止、手动操作、无扰切换等场景
 * @param PID PID控制器句柄
 * @param NewValue 手动模式下的输出值
 * @return 无
 */
void PID_ChangeManualOutput(PID_TypeDef *PID, float NewValue)
{
	PID->ManualOutput = NewValue;
}

/**
 * @brief 修改积分项限幅
 * @details 动态修改积分项的上下限，防止积分饱和
 * @param PID PID控制器句柄
 * @param UpperLimit 积分项上限
 * @param LowerLimit 积分项下限
 * @return 无
 */
void PID_ChangeITermLimits(PID_TypeDef *PID, float UpperLimit, float LowerLimit)
{
	PID->ITermUpperLimit = UpperLimit;
	PID->ITermLowerLimit = LowerLimit;
}

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
float PID_Compute1_Rectangle(PID_TypeDef *PID, float Input, uint64_t now)
{
	float Output;
	float error;
	float dError;

	// 计算误差：设定值 - 当前输入值
	error = PID->Setpoint - Input;
	// 计算误差变化量
	dError = error - PID->LastError;

	// 累加误差（离散矩形积分）
	PID->ITerm += error;

	// 积分项独立限幅，防止积分饱和
	if(PID->ITerm > PID->ITermUpperLimit)
	{
		PID->ITerm = PID->ITermUpperLimit;
	}
	else if(PID->ITerm < PID->ITermLowerLimit)
	{
		PID->ITerm = PID->ITermLowerLimit;
	}

	// 计算各部分输出
	// 比例项 P = Kp * error
	Output = PID->Kp * error;
	// 积分项 I = Ki * Σerror
	Output += PID->Ki * PID->ITerm;
	// 微分项 D = Kd * Δerror
	if(PID->Kd != 0)
	{
		PID->DTerm = PID->Kd * dError;
		Output += PID->DTerm;
	}

	// 输出限幅(限制在[OutputLowerLimit, OutputUpperLimit]范围内)
	if(Output > PID->OutputUpperLimit)
	{
		Output = PID->OutputUpperLimit;
	}
	else if(Output < PID->OutputLowerLimit)
	{
		Output = PID->OutputLowerLimit;
	}

	// 如果处于手动模式，直接使用手动输出值覆盖计算结果
	if(PID->Manual != 0)
	{
		Output = PID->ManualOutput;
	}

	// 保存当前状态，供下次计算使用
	PID->LastInput = Input;      // 保存当前输入值
	PID->LastTime = now;        // 保存当前时间
	PID->LastError = error;     // 保存当前误差
	PID->LastOutput = Output;   // 保存当前输出值

	return Output;
}

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
float PID_Compute2_Rectangle(PID_TypeDef *PID, float Input, float dInputDt, uint64_t now)
{
	float Output;
	float error;
	float dError;

	(void)dInputDt;

	// 计算误差
	error = PID->Setpoint - Input;
	// 计算误差变化量
	dError = error - PID->LastError;

	// 累加误差（离散矩形积分）
	PID->ITerm += error;

	// 积分项独立限幅
	if(PID->ITerm > PID->ITermUpperLimit)
	{
		PID->ITerm = PID->ITermUpperLimit;
	}
	else if(PID->ITerm < PID->ITermLowerLimit)
	{
		PID->ITerm = PID->ITermLowerLimit;
	}

	// 计算各部分输出
	// 比例项 P = Kp * error
	Output = PID->Kp * error;
	// 积分项 I = Ki * Σerror
	Output += PID->Ki * PID->ITerm;
	// 微分项 D = Kd * Δerror
	if(PID->Kd != 0)
	{
		PID->DTerm = PID->Kd * dError;
		Output += PID->DTerm;
	}

	// 输出限幅
	if(Output > PID->OutputUpperLimit)
	{
		Output = PID->OutputUpperLimit;
	}
	else if(Output < PID->OutputLowerLimit)
	{
		Output = PID->OutputLowerLimit;
	}

	// 手动模式处理
	if(PID->Manual != 0)
	{
		Output = PID->ManualOutput;
	}

	// 保存当前状态
	PID->LastInput = Input;
	PID->LastTime = now;
	PID->LastError = error;
	PID->LastOutput = Output;

	return Output;
}
