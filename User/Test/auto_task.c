#include "auto_task.h"
#include "balance.h"

static AutoTask_State_t auto_state;

void AutoTask_Init(void)
{
    auto_state = AUTO_TASK_IDLE;
}


//先进行零点标定
void AutoTask_Start(void)
{
    Balance_SetCommand(BALANCE_COMMAND_ZERO_CALIBRATE);
    
    if(Balance_GetResult() == BALANCE_RESULT_FAILED)
    {
        auto_state = AUTO_TASK_FAULT;
    }
    else
    {
        auto_state = AUTO_TASK_ZERO;
    }
}

void AutoTask_Update(void)
{
    Balance_Result_t result;
    
    result = Balance_GetResult();
    
    switch(auto_state)
    {
        case AUTO_TASK_IDLE:
        {
        break;
        }
        
        case AUTO_TASK_ZERO:
        {
            if(result == BALANCE_RESULT_SUCCEEDED)
            {
                Balance_SetCommand(BALANCE_COMMAND_FULL_LEVEL);
                
                if(Balance_GetResult() == BALANCE_RESULT_FAILED)
                {
                    auto_state = AUTO_TASK_FAULT;
                }
                else
                {
                    auto_state = AUTO_TASK_LEVEL;
                }
            }
            else if((result == BALANCE_RESULT_FAILED) || (result == BALANCE_RESULT_ABORTED))
            {
                auto_state = AUTO_TASK_FAULT;
            }
            break;
        }
        
        case AUTO_TASK_LEVEL:
        {
            if(result == BALANCE_RESULT_SUCCEEDED)
            {
                auto_state = AUTO_TASK_COMPLETE;
            }
            else if((result == BALANCE_RESULT_FAILED) || (result == BALANCE_RESULT_ABORTED))
            {
                auto_state = AUTO_TASK_FAULT;
            }
            break;
        }
        
        case AUTO_TASK_COMPLETE:
        {
            break;
        }
        
        case AUTO_TASK_FAULT:
            
        {
            //balance发生错误的时候已经发送零电流
            break;
        }
    }
}

void AutoTask_Stop(void)
{
    Balance_SetCommand(BALANCE_COMMAND_ABORT);
    auto_state = AUTO_TASK_IDLE;
}

AutoTask_State_t AutoTask_GetState(void)
{
    return auto_state;
}


