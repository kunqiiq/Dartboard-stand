#ifndef PAGE_BALANCE_H
#define PAGE_BALANCE_H

#include "tjc_hmi.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/*
 * page_balance不直接控制dartboard_hmi.c中的页面状态变量，
 * 而是返回页面动作，由dartboard_hmi.c统一切换页面。
 */
typedef enum
{
    PAGE_BALANCE_ACTION_NONE = 0,
    PAGE_BALANCE_ACTION_RETURN_HOME
} PageBalance_Action_t;

// 初始化page3控制模块 
void PageBalance_Init(void);

//进入page3时调用,会停止自动调平并切换到手动测试向导模式。
void PageBalance_Enter(TJC_HMI *hmi);

//离开page3时调用。会立即停止当前测试和所有调平电机。
void PageBalance_Leave(void);

/*
 * 周期任务，建议每10 ms调用一次。
 * 用途：
 * - 检测b_bottom是否长按达到1秒；
 * - 更新t_step；
 * - 更新t_txt。
 */
void PageBalance_Task(void);

/*
 * 处理串口屏触摸事件。
 * 返回PAGE_BALANCE_ACTION_RETURN_HOME时，
 * dartboard_hmi.c应负责切换到page0（页面ID=1）。
 */
PageBalance_Action_t PageBalance_HandleEvent(
    TJC_HMI *hmi,
    const TJC_Event *event);

// 判断当前是否处于page3手动测试模式
uint8_t PageBalance_IsActive(void);
    
    
#ifdef __cplusplus
}
#endif

#endif

