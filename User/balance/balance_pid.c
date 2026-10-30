#include "balance_pid.h"

#include <string.h>

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
              BalancePID_DFirst_e d_first)
{
    if (pid == 0)
    {
        return;
    }

    /* 清零全部运行状态 */
    memset(pid, 0, sizeof(BalancePID_t));

    pid->K_P = k_p;
    pid->K_I = k_i;
    pid->K_D = k_d;
    pid->K_F = k_f;

    pid->I_Out_Max = i_out_max;
    pid->Out_Max = out_max;

    pid->D_T = d_t;
    pid->Dead_Zone = dead_zone;

    pid->I_Variable_Speed_A = variable_speed_a;
    pid->I_Variable_Speed_B = variable_speed_b;
    pid->I_Separate_Threshold = separate_threshold;

    pid->D_First = d_first;
}

void BalancePID_Set_K_P(BalancePID_t *pid, float value)
{
    pid->K_P = value;
}

void BalancePID_Set_K_I(BalancePID_t *pid, float value)
{
    pid->K_I = value;
}

void BalancePID_Set_K_D(BalancePID_t *pid, float value)
{
    pid->K_D = value;
}

void BalancePID_Set_K_F(BalancePID_t *pid, float value)
{
    pid->K_F = value;
}

void BalancePID_Set_I_Out_Max(BalancePID_t *pid, float value)
{
    pid->I_Out_Max = value;
}

void BalancePID_Set_Out_Max(BalancePID_t *pid, float value)
{
    pid->Out_Max = value;
}

void BalancePID_Set_I_Variable_Speed_A(BalancePID_t *pid, float value)
{
    pid->I_Variable_Speed_A = value;
}

void BalancePID_Set_I_Variable_Speed_B(BalancePID_t *pid, float value)
{
    pid->I_Variable_Speed_B = value;
}

void BalancePID_Set_I_Separate_Threshold(BalancePID_t *pid, float value)
{
    pid->I_Separate_Threshold = value;
}

void BalancePID_Set_Target(BalancePID_t *pid, float value)
{
    pid->Target = value;
}

void BalancePID_Set_Now(BalancePID_t *pid, float value)
{
    pid->Now = value;
}

void BalancePID_Set_Integral_Error(BalancePID_t *pid, float value)
{
    pid->Integral_Error = value;
}

void BalancePID_Adjust(BalancePID_t *pid)
{
    float error;
    float abs_error;
    float speed_ratio;

    if (pid == 0)
    {
        return;
    }

    /* 防止微分计算除以零 */
    if (pid->D_T <= 0.0f)
    {
        pid->Out = 0.0f;
        return;
    }

    pid->P_Out = 0.0f;
    pid->I_Out = 0.0f;
    pid->D_Out = 0.0f;
    pid->F_Out = 0.0f;

    error = pid->Target - pid->Now;
    abs_error = Math_Abs_Float(error);

    /* 死区处理 */
    if (abs_error < pid->Dead_Zone)
    {
        pid->Target = pid->Now;
        error = 0.0f;
        abs_error = 0.0f;
    }

    /* P 项 */
    pid->P_Out = pid->K_P * error;

    /* 计算变速积分比例 */
    if ((pid->I_Variable_Speed_A == 0.0f) &&
        (pid->I_Variable_Speed_B == 0.0f))
    {
        speed_ratio = 1.0f;
    }
    else if (abs_error <= pid->I_Variable_Speed_A)
    {
        speed_ratio = 1.0f;
    }
    else if ((pid->I_Variable_Speed_B > 0.0f) &&
             (abs_error <
              pid->I_Variable_Speed_A +
              pid->I_Variable_Speed_B))
    {
        speed_ratio =
            1.0f -
            (abs_error - pid->I_Variable_Speed_A) /
            pid->I_Variable_Speed_B;
    }
    else
    {
        speed_ratio = 0.0f;
    }

    /* 原代码是在累加积分前限幅，这里保留原算法顺序 */
    if ((pid->I_Out_Max != 0.0f) &&
        (pid->K_I != 0.0f))
    {
        float integral_max;

        integral_max =
            pid->I_Out_Max / Math_Abs_Float(pid->K_I);

        Math_Constrain_Float(
            &pid->Integral_Error,
            -integral_max,
            integral_max);
    }

    /* I 项和积分分离 */
    if ((pid->I_Separate_Threshold == 0.0f) ||
        (abs_error < pid->I_Separate_Threshold))
    {
        pid->Integral_Error +=
            speed_ratio * pid->D_T * error;

        if ((pid->I_Out_Max != 0.0f) && (pid->K_I != 0.0f))
        {
            float integral_max;

            integral_max = pid->I_Out_Max / Math_Abs_Float(pid->K_I);
            Math_Constrain_Float(&pid->Integral_Error,
                                 -integral_max, integral_max);
        }

        pid->I_Out =
            pid->K_I * pid->Integral_Error;
    }
    else
    {
        pid->Integral_Error = 0.0f;
        pid->I_Out = 0.0f;
    }

    /* D 项 */
    if (pid->D_First == BALANCE_PID_D_FIRST_DISABLE)
    {
        pid->D_Out =
            pid->K_D *
            (error - pid->Pre_Error) /
            pid->D_T;
    }
    else
    {
        pid->D_Out =
            -pid->K_D *
            (pid->Now - pid->Pre_Now) /
            pid->D_T;
    }

    /* 前馈项 */
    pid->F_Out =
        (pid->Target - pid->Pre_Target) *
        pid->K_F;

    /* 总输出 */
    pid->Out =
        pid->P_Out +
        pid->I_Out +
        pid->D_Out +
        pid->F_Out;

    /* 输出限幅 */
    if (pid->Out_Max != 0.0f)
    {
        Math_Constrain_Float(
            &pid->Out,
            -pid->Out_Max,
            pid->Out_Max);
    }

    /* 保存本次状态 */
    pid->Pre_Now = pid->Now;
    pid->Pre_Target = pid->Target;
    pid->Pre_Out = pid->Out;
    pid->Pre_Error = error;
}

float BalancePID_Get_Integral_Error(const BalancePID_t *pid)
{
    return pid->Integral_Error;
}

float BalancePID_Get_Out(const BalancePID_t *pid)
{
    return pid->Out;
}

float BalancePID_Get_P_Out(const BalancePID_t *pid)
{
    return pid->P_Out;
}

float BalancePID_Get_I_Out(const BalancePID_t *pid)
{
    return pid->I_Out;
}

float BalancePID_Get_D_Out(const BalancePID_t *pid)
{
    return pid->D_Out;
}

float BalancePID_Get_F_Out(const BalancePID_t *pid)
{
    return pid->F_Out;
}

float BalancePID_Get_K_P(const BalancePID_t *pid)
{
    return pid->K_P;
}

float BalancePID_Get_K_I(const BalancePID_t *pid)
{
    return pid->K_I;
}

float BalancePID_Get_K_D(const BalancePID_t *pid)
{
    return pid->K_D;
}

float BalancePID_Get_K_F(const BalancePID_t *pid)
{
    return pid->K_F;
}

