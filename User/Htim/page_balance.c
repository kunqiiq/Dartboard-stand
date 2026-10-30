/*
 * page3调平测试向导界面
 * 页面与控件：
 * page3
 * ├── b_back    ID=1  返回page0（页面ID=1）
 * ├── b_bottom  ID=2  测试向导控制键
 * ├── t_txt     ID=3  显示测试信息
 * └── t_step    ID=4  显示当前步骤
 * b_bottom操作：
 * - 短按松开：下一步、重试；
 * - 测试动作运行中短按：中止当前步骤；
 * - 按住达到1秒：紧急停止并重置向导。
 * b_back操作：
 * - 松开时停止调平电机；
 * - 退出手动测试模式；
 * - 通知dartboard_hmi.c返回page0。
 */
#include "page_balance.h"
#include "balance_app.h"
#include "chassic_test.h"
#include "main.h"
#include <stdio.h>

//页面与控件ID
#define PAGE_BALANCE_PAGE_ID        4
#define PAGE_BALANCE_BACK_ID        1
#define PAGE_BALANCE_BOTTOM_ID      2
#define PAGE_BALANCE_TEXT_ID        3
#define PAGE_BALANCE_STEP_ID        4
//实际发送显示指令使用的是控件名称，ID仅用于识别触摸事件
#define PAGE_BALANCE_TEXT_OBJECT    "t_txt"
#define PAGE_BALANCE_STEP_OBJECT    "t_step"
//标准触摸事件值
#define PAGE_BALANCE_TOUCH_RELEASE     0
#define PAGE_BALANCE_TOUCH_PRESS        1
//b_bottom长按判定时间
#define PAGE_BALANCE_LONG_PRESS_MS      1000
//串口屏刷新时间
#define PAGE_BALANCE_UI_PERIOD_MS       100

//内部状态
static TJC_HMI *page_balance_hmi;
static uint8_t page_balance_active;
static uint8_t bottom_pressed;
static uint8_t bottom_long_reported;
static uint32_t bottom_press_tick;
//上一次屏幕刷新时间
static uint32_t last_ui_tick;
//已经显示在屏幕上的消息版本,发生变化的时候蔡刷新文本
static uint32_t displayed_message_revision;
//已经显示在屏幕上的步骤
static uint8_t displayed_step;
//强制下一次任务刷新屏幕
static uint8_t force_refresh;


static void PageBalance_SetStepText(uint8_t step)
{
    char step_text[20];
    if(page_balance_hmi == NULL)
    {
        return;
    }
    if(ChassisTest_IsFinished() != 0)
    {
        (void)snprintf(step_text,sizeof(step_text),"DONE");
    }
    else if(step == 0)
    {
        (void)snprintf(step_text,sizeof(step_text),"READY");
    }
    else if(step <= 8)
    {
        (void)snprintf(step_text,sizeof(step_text),"%u/8",(unsigned)step);
    }
    else
    {
        (void)snprintf(step_text,sizeof(step_text),"ERROR");
    }
    (void)TJC_Text_SetText(page_balance_hmi,PAGE_BALANCE_STEP_OBJECT,step_text);
}


static void PageBalance_SetMessageText(void)
{
    const char *message;
    if(page_balance_hmi == NULL)
    {
        return;
    }
    message = ChassisTest_GetMessage();
    if((message == NULL )|| (message[0] == '\0'))
    {
        message = "Press b_bottom to start step1.";
    }
    
    (void)TJC_Text_SetText(page_balance_hmi,PAGE_BALANCE_TEXT_OBJECT,message);
}

static void PageBalance_UpdateDisplay(uint32_t now)
{
    uint8_t current_step;
    uint32_t current_revision;
    if((page_balance_active == 0) || (page_balance_hmi == NULL))
    {
        return;
    }
    //强制刷新时不等待刷新周期，普通刷新显示为100ms刷新一次
    if((force_refresh == 0) && ((uint32_t)(now - last_ui_tick) < PAGE_BALANCE_UI_PERIOD_MS))
    {
        return;
    }
    current_step = ChassisTest_GetStep();
    current_revision = ChassisTest_GetMessageRevision();
    
    if((force_refresh != 0) || (current_step != displayed_step))
    {
        PageBalance_SetStepText(current_step);
        displayed_step = current_step;
    }
    if((force_refresh != 0) || (current_revision != displayed_message_revision))
    {
        PageBalance_SetMessageText();
        displayed_message_revision = current_revision;
    }
    last_ui_tick = now;
    force_refresh = 0;
}


void PageBalance_Init(void)
{
    page_balance_hmi = NULL;
    page_balance_active = 0;
    bottom_pressed = 0;
    bottom_long_reported  =0;
    bottom_press_tick = 0;
    last_ui_tick = 0;
    displayed_message_revision = 0;
    displayed_step  = 0;
    force_refresh = 0;
}

void PageBalance_Enter(TJC_HMI *hmi)
{
    if(hmi == NULL)
    {
        return;
    }
    page_balance_hmi = hmi;
    
    //防止同一次页面切换既收到首页按钮事件，又收到page3页面报告事件
    if(page_balance_active != 0)
    {
        force_refresh = 1;
        return;
    }
    page_balance_active = 0;
    bottom_pressed = 0;
    bottom_long_reported = 0;
    bottom_press_tick  = 0;
    //使用不可能的初值，保证第一次任务必定刷新两个文本控件
    displayed_message_revision = 0xffffffff;
    displayed_step = 0xff;
    last_ui_tick = HAL_GetTick();
    force_refresh = 1;
    
    //自动切换到手动模式
    if(BalanceApp_StartManual() == 0)
    {
        page_balance_active = 0;
        (void)TJC_Text_SetText(page_balance_hmi,PAGE_BALANCE_STEP_OBJECT,"ERROR");
        (void)TJC_Text_SetText(page_balance_hmi,PAGE_BALANCE_TEXT_OBJECT,"Balance subsysterm is not ready.");
        force_refresh = 0;
    }
    else
    {
        page_balance_active = 1;
    }
}

void PageBalance_Leave(void)
{
    BalanceApp_Stop();
    page_balance_active = 0;
    bottom_pressed = 0;
    bottom_long_reported = 0;
    bottom_press_tick = 0;
    force_refresh = 0;
}




////////////////////////////////////////////////串口屏事件处理
PageBalance_Action_t PageBalance_HandleEvent(TJC_HMI *hmi,const TJC_Event *event)
{
    uint32_t now;
    if((hmi == NULL) || (event == NULL))
    {
        return PAGE_BALANCE_ACTION_NONE;
    }
    if(event->type != TJC_EVENT_TOUCH)
    {
        return PAGE_BALANCE_ACTION_NONE;
    }
    if(event->page_id != PAGE_BALANCE_PAGE_ID)
    {
        return PAGE_BALANCE_ACTION_NONE;
    }
    now = HAL_GetTick();
    page_balance_hmi = hmi;
    
    //b_back，ID = 1
    if(event->component_id == PAGE_BALANCE_BACK_ID)
    {
        if(event->touch_event == PAGE_BALANCE_TOUCH_RELEASE)
        {
            PageBalance_Leave();
            return PAGE_BALANCE_ACTION_RETURN_HOME;
        }
        return PAGE_BALANCE_ACTION_NONE;
    }
    
    //b_bottom, ID = 2
    if(event->component_id != PAGE_BALANCE_BOTTOM_ID)
    {
        return PAGE_BALANCE_ACTION_NONE;
    }
    if(page_balance_active ==0)
    {
        PageBalance_Enter(hmi);
        if(page_balance_active == 0)
        {
            return PAGE_BALANCE_ACTION_NONE;
        }
    }
    if(event->touch_event == PAGE_BALANCE_TOUCH_PRESS)
    {
        bottom_pressed = 1;
        bottom_long_reported = 0;
        bottom_press_tick = now;
    }
    else if(event->touch_event == PAGE_BALANCE_TOUCH_RELEASE)
    {
        if(bottom_long_reported == 0)
        {
            ChassisTest_OnShortPress(now);
        }
        bottom_pressed = 0;
        bottom_long_reported = 0;
        force_refresh = 1;
    }
    return PAGE_BALANCE_ACTION_NONE;
}


///////////////////////////////////////////周期任务
void PageBalance_Task(void)
{
    uint32_t now;
    if(page_balance_active == 0)
    {
        return;
    }
    now = HAL_GetTick();
    
    if((bottom_pressed != 0) && (bottom_long_reported == 0) && ((uint32_t)(now - bottom_press_tick) >= PAGE_BALANCE_LONG_PRESS_MS))
    {
        bottom_long_reported = 1;
        ChassisTest_OnLongPress();
        force_refresh = 1;
    }
    PageBalance_UpdateDisplay(now);
}

uint8_t PageBalance_IsActive(void)
{
    return page_balance_active;
}

