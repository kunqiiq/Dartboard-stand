#ifndef __BALANCE_H__
#define __BALANCE_H__

#include "main.h"
#include "balance_motor.h"
#include "balance_pid.h"

typedef enum
{
    BALANCE_STATE_WAIT_READY = 0,
    BALANCE_STATE_FIND_BOTTOM,          //四个电机向下寻找限位
    BALANCE_STATE_ROLLBACK,             //向上回退
    BALANCE_STATE_LEVEL_ADJUST,         //四轴调平
    BALANCE_STATE_LEVEL_DESCEND,        //保持调平且下降
    BALANCE_STATE_REFERENCE_DESCEND,    //只让参考轴下降
    BALANCE_STATE_THREE_AXIS_LEVEL,     //其余三轴调平
    
    BALANCE_STATE_AXIS_TEST,            //安全单轴速度测试
    
    BALANCE_STATE_HOLD,                 //保持且电机停止
    BALANCE_STATE_ERROR                 //故障
} Balance_State_t;


typedef enum
{
    BALANCE_ERROR_NONE = 0,

    BALANCE_ERROR_MOTOR_OFFLINE,            //电机反馈超时
    BALANCE_ERROR_IMU_OFFLINE,              //IMU数据无效

    BALANCE_ERROR_FIND_BOTTOM_TIMEOUT,      //寻找限位超时
    BALANCE_ERROR_ROLLBACK_TIMEOUT,         //回退超时
    BALANCE_ERROR_LEVEL_TIMEOUT,            //调平超时

    BALANCE_ERROR_LEVEL_DESCEND_TIMEOUT,    //整体下降超时
    BALANCE_ERROR_REFERENCE_DESCEND_TIMEOUT,//参考轴下降超时
    BALANCE_ERROR_THREE_AXIS_TIMEOUT,       //三轴调平超时

    BALANCE_ERROR_UPPER_LIMIT,
    BALANCE_ERROR_MOTOR_STALL,
    BALANCE_ERROR_CAN_TX,
    BALANCE_ERROR_LIMIT_CONFLICT,
    BALANCE_ERROR_INVALID_STATE,
    BALANCE_ERROR_SCHEDULER,
    BALANCE_ERROR_EMERGENCY_STOP,
    BALANCE_ERROR_NO_VALID_ZERO,
    BALANCE_ERROR_NONFINITE,
    BALANCE_ERROR_GEOMETRY,
    BALANCE_ERROR_MOTOR_OVERHEAT
} Balance_Error_t;


typedef enum
{
    BALANCE_COMMAND_IDLE = 0,
    BALANCE_COMMAND_ABORT,
    BALANCE_COMMAND_ZERO_CALIBRATE,
    BALANCE_COMMAND_LEVEL_ONLY,
    BALANCE_COMMAND_FULL_LEVEL,
    BALANCE_COMMAND_LEVEL = BALANCE_COMMAND_FULL_LEVEL
} Balance_Command_t;


typedef enum
{
    BALANCE_RESULT_IDLE = 0,
    BALANCE_RESULT_RUNNING,
    BALANCE_RESULT_SUCCEEDED,
    BALANCE_RESULT_FAILED,
    BALANCE_RESULT_ABORTED
} Balance_Result_t;


typedef enum{
    BALANCE_AXIS_TEST_IDLE = 0,
    BALANCE_AXIS_TEST_RUNNING,
    BALANCE_AXIS_TEST_SUCCEEDED,
    BALANCE_AXIS_TEST_LIMITED,
    BALANCE_AXIS_TEST_OFFLINE,
    BALANCE_AXIS_TEST_STALLED,
    BALANCE_AXIS_TEST_ABORTED,
    BALANCE_AXIS_TEST_INVALID
}Balance_AxisTestResult_t;

typedef struct{
    uint8_t online;
    uint8_t bottom_limit_raw;
    uint8_t bottom_limit_active;
    uint16_t ecd;
    int32_t total_ecd;
    int32_t height;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperature;
    
}Balance_AxisDiagnostic_t;


void Balance_Init(void);
void Balance_SetCommand(Balance_Command_t command);

void Balance_Update(
    float pitch,
    float roll,
    float pitch_rate,
    float roll_rate,
    uint8_t imu_valid);

void Balance_Stop(Balance_Error_t error);

Balance_State_t Balance_GetState(void);
Balance_Error_t Balance_GetError(void);
Balance_Command_t Balance_GetCommand(void);
Balance_Result_t Balance_GetResult(void);
uint8_t Balance_ZeroValid(void);
    
uint8_t Balance_StartAxisTest(uint8_t axis, float target_speed,
                              uint32_t duration_ms);
void Balance_AbortAxisTest(void);
uint8_t Balance_AxisTestRunning(void);
Balance_AxisTestResult_t Balance_GetAxisTestResult(void);
uint8_t Balance_GetAxisDiagnostic(uint8_t axis,Balance_AxisDiagnostic_t *diagnostic);

    
    


#endif
    


    
