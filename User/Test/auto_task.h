#ifndef __AUTO_TASK_H__
#define __AUTO_TASK_H__

#include "main.h"
typedef enum
{
    AUTO_TASK_IDLE = 0,
    AUTO_TASK_ZERO,
    AUTO_TASK_LEVEL,
    AUTO_TASK_COMPLETE,
    AUTO_TASK_FAULT
}AutoTask_State_t;


void AutoTask_Init(void);
void AutoTask_Start(void);
void AutoTask_Update(void);
void AutoTask_Stop(void);

AutoTask_State_t AutoTask_GetState(void);

#endif

