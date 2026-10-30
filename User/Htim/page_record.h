#ifndef __PAGE_RECORD_H
#define __PAGE_RECORD_H

#include "tjc_hmi.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif


//串口屏页面ID
#define PAGE_RECORD_INPUT_PAGE_ID        3U
#define PAGE_RECORD_KEYBOARD_PAGE_ID     5U
#define PAGE_RECORD_HISTORY_PAGE_ID      6U

//页面事件处理结果
typedef enum
{
    PAGE_RECORD_ACTION_NONE = 0,
    PAGE_RECORD_ACTION_RETURN_HOME,
    PAGE_RECORD_ACTION_SHOW_HISTORY
}PageRecord_Action_t;


//初始化记录页面，清空ram缓存，初始化flash记录存储，从flash恢复最近四条记录
void PageRecord_Init(void);
//串口屏复位时调用，只取消尚未完成的文本读取
void PageRecord_OnDisplayReset(void);
//当前页面发生变化时调用，检测到键盘返回后，自动请求文本
void PageRecord_OnPageChanged(TJC_HMI *hmi,uint8_t previous_page,uint8_t current_page);
//处理记录相关事件
PageRecord_Action_t PageRecord_HandleEvent(TJC_HMI *hmi,const TJC_Event *event);
//将最近的四条记录写入界面page4
void PageRecord_ShowHistory(TJC_HMI *hmi);
//查询flash存储是否出现错误
uint8_t PageRecord_HasStorageError(void);

#ifdef __cplusplus
}
#endif

#endif
