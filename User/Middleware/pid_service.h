#ifndef PID_SERVICE_H
#define PID_SERVICE_H
#include "PID.h"

typedef enum
{
    PID_2006 = 0,
	PID_3508 = 1,
	PID_6020 = 2,
    PID_NUNBER
} PID_ID_Typedef;


extern PID_TypeDef g_pid_gather[PID_NUNBER];

void PidService_Configure(PID_TypeDef *pid, float p, float i, float d, float out_max, float out_min, float i_Outmax, float i_Outmin);

void PidService_Init(void);

#endif /* PID_SERVICE_H */
