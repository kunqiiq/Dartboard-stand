#ifndef BALANCE_PID_H
#define BALANCE_PID_H

/* Includes ------------------------------------------------------------------*/

#include "drv_math.h"

/**
 * @brief 微分先行
 *枚举类型：是否使用微分先行
 */
typedef enum
{
    BALANCE_PID_D_FIRST_DISABLE = 0,
    BALANCE_PID_D_FIRST_ENABLE
} BalancePID_DFirst_e;

/**
 * @brief Reusable, PID算法
 *
 */
typedef struct
{
    /* PID 计算周期 */
    float D_T;

    /* 死区 */
    float Dead_Zone;

    /* 是否使用微分先行 */
    BalancePID_DFirst_e D_First;

    /* 上一次数据 */
    float Pre_Now;
    float Pre_Target;
    float Pre_Out;
    float Pre_Error;

    /* PID 输出 */
    float Out;

    /* PID 参数 */
    float K_P;
    float K_I;
    float K_D;
    float K_F;

    /* 限幅参数 */
    float I_Out_Max;
    float Out_Max;

    /* 变速积分参数 */
    float I_Variable_Speed_A;
    float I_Variable_Speed_B;

    /* 积分分离阈值 */
    float I_Separate_Threshold;

    /* 目标值和当前值 */
    float Target;
    float Now;

    /* 积分误差 */
    float Integral_Error;

    /* PID 各项输出 */
    float P_Out;
    float I_Out;
    float D_Out;
    float F_Out;

} BalancePID_t;

void BalancePID_Init(BalancePID_t *pid,
              float k_p,
              float k_i,
              float k_d,
              float k_f,
              float i_out_max,
              float out_max,
              float d_t,
              float dead_zone,
              float variable_speed_a,
              float variable_speed_b,
              float separate_threshold,
              BalancePID_DFirst_e d_first);


void BalancePID_Set_K_P(BalancePID_t *pid, float value);
void BalancePID_Set_K_I(BalancePID_t *pid, float value);
void BalancePID_Set_K_D(BalancePID_t *pid, float value);
void BalancePID_Set_K_F(BalancePID_t *pid, float value);

void BalancePID_Set_I_Out_Max(BalancePID_t *pid, float value);
void BalancePID_Set_Out_Max(BalancePID_t *pid, float value);

void BalancePID_Set_I_Variable_Speed_A(BalancePID_t *pid, float value);
void BalancePID_Set_I_Variable_Speed_B(BalancePID_t *pid, float value);
void BalancePID_Set_I_Separate_Threshold(BalancePID_t *pid, float value);

void BalancePID_Set_Target(BalancePID_t *pid, float value);
void BalancePID_Set_Now(BalancePID_t *pid, float value);
void BalancePID_Set_Integral_Error(BalancePID_t *pid, float value);

void BalancePID_Adjust(BalancePID_t *pid);

float BalancePID_Get_Integral_Error(const BalancePID_t *pid);
float BalancePID_Get_Out(const BalancePID_t *pid);

float BalancePID_Get_P_Out(const BalancePID_t *pid);
float BalancePID_Get_I_Out(const BalancePID_t *pid);
float BalancePID_Get_D_Out(const BalancePID_t *pid);
float BalancePID_Get_F_Out(const BalancePID_t *pid);

float BalancePID_Get_K_P(const BalancePID_t *pid);
float BalancePID_Get_K_I(const BalancePID_t *pid);
float BalancePID_Get_K_D(const BalancePID_t *pid);
float BalancePID_Get_K_F(const BalancePID_t *pid);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
