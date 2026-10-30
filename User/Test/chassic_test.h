#ifndef CHASSIS_TEST_H
#define CHASSIS_TEST_H

#include "main.h"
#include "imu_attitude.h"

#ifdef __cplusplus
extern "C" {
#endif

//初始化测试向导
void ChassisTest_Init(void);
//BalanceApp_Task每1ms调用
void ChassisTest_Update(uint32_t now,const IMU_Attitude_t *imu,uint8_t imu_valid);
//停止当前测试动作
void ChassisTest_ForceStop(void);
//在t_txt显示严重故障信息
void ChassisTest_ReportFatal(const char *message);
//串口屏短按接口，松开事件中调用
void ChassisTest_OnShortPress(uint32_t now);
//串口屏长按接口，松开事件中调用
void ChassisTest_OnLongPress(void);
//测试向导是否已经完成
uint8_t ChassisTest_IsFinished(void);
//8个测试步骤是否全部通过
uint8_t ChassisTest_AllPassed(void);
//步骤
uint8_t ChassisTest_GetStep(void);
//当前是否有测试动作正在运行
uint8_t ChassisTest_IsRunning(void);
//获取准备显示到串口屏中的文字
const char *ChassisTest_GetMessage(void);
//每次文字发生变化时递增，判断是否刷新
uint32_t ChassisTest_GetMessageRevision(void);


#ifdef __cplusplus
}
#endif

#endif
