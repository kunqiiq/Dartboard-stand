#include "pid_service.h"

PID_TypeDef g_pid_gather[PID_NUNBER];


void PidService_Configure(PID_TypeDef *pid, float p, float i, float d, float out_max, float out_min,float i_Outmax,float i_Outmin)
{
    PID_InitTypeDef pid_init = {0};  
    pid_init.Kp = p;
    pid_init.Ki = i;
    pid_init.Kd = d;
    pid_init.Setpoint = 0.0f;
    pid_init.OutputUpperLimit = out_max;
    pid_init.OutputLowerLimit = out_min;
    pid_init.ITermUpperLimit  = i_Outmax;
    pid_init.ITermLowerLimit  = i_Outmin;
    pid_init.DefaultOutput = 0.0f;
    PID_Init(pid, &pid_init);
}

void PidService_Init(void)
{
    /********************底盘电机初始PID参数***************/
    PidService_Configure(&g_pid_gather[PID_2006], 160.0f, 0.35f, 0.0f, 10000.0f, -10000.0f,8000.0f,-8000.0f);
    PidService_Configure(&g_pid_gather[PID_3508],  38.5f, 0.2f, 0.0f, 16384.0f, -16384.0f,8000.0f,-8000.0f);
    PidService_Configure(&g_pid_gather[PID_6020], 15.0f, 0.0f, 0.0f, 16384.0f, -16384.0f,8000.0f,-8000.0f);

}




//    Pid_parameter_setting(&pid_gather[PID_3508], 31.5f, 0.1f, 0.0f, 16384.0f, -16384.0f,8000.0f,-8000.0f);
