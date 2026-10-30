#include "page_record.h"
#include <stddef.h>
#include <string.h>
#include "record_storage.h"
#include "balance_app.h"
#include "motor_service.h"
#include "page_fire.h"

//page2控件ID
#define PAGE_RECORD_BACK_ID     1
#define PAGE_RECORD_HISTORY_ID  2
#define PAGE_RECORD_INPUT_ID    3

//page4控件id
#define PAGE_HISTORY_BACK_ID    5

#define PAGE_HISTORY_TEXT1_OBJECT   "t_txt1"
#define PAGE_HISTORY_TEXT2_OBJECT   "t_txt2"
#define PAGE_HISTORY_TEXT3_OBJECT   "t_txt3"
#define PAGE_HISTORY_TEXT4_OBJECT   "t_txt4"

//公共配置
#define PAGE_RECORD_INPUT_OBJECT    "g0"
#define PAGE_RECORD_TOUCH_RELEASE    0
#define PAGE_RECORD_TOUCH_PRESS      1
#define PAGE_RECORD_HISTORY_COUNT     4

#define PAGE_RECORD_TEXT_MAX_BYTES   RECORD_STORAGE_TEXT_MAX_BYTES

//最近四条记录的RAM镜像
static char page_record_history[PAGE_RECORD_HISTORY_COUNT][PAGE_RECORD_TEXT_MAX_BYTES +1];
//是否已经发送go_txt，正在等待字符串返回
static uint8_t page_record_text_request_pending;
//收到g0文本后需要执行的页面动作
static PageRecord_Action_t page_record_action_after_text;

static uint8_t page_record_storage_error;


/* ======================== 文本处理 ======================== */

/*
 * 将TJC_EVENT_STRING中的字节复制为C字符串。
 *
 * TJC文本赋值命令不接受：
 * - 双引号
 * - 0xFF
 * - 小于0x20的控制字符
 */
 static void PageRecord_CopyText(char *destination,const uint8_t *source,size_t source_length)
 {
    size_t source_index;
     size_t destination_index;
     uint8_t value;
     
     if(destination == NULL)
     {
        return;
     }
     destination_index = 0;
     
     if(source != NULL)
     {
        for(source_index = 0;source_index < source_length;++source_index)
         {
            if(destination_index >= PAGE_RECORD_TEXT_MAX_BYTES)
            {
                break;
            }
            value = source[source_index];
            
            if((value == 0) || (value == 0xff))
            {
                break;
            }
            
            if(value == (uint8_t)'"')
            {
                value = (uint8_t)'\'';
            }
            else if(value < 0x20)
            {
                value = (uint8_t)' ';
            }
            
            destination[destination_index] = (char)value;
            ++destination_index;
         }
     }
     destination[destination_index] = '\0';
 }
 
 
 //将新记录加入最近四条记录；没有有效文字时返回0且不改变历史记录
 static uint8_t PageRecord_StoreText(const uint8_t *text,size_t text_length)
 {
    char new_text[PAGE_RECORD_TEXT_MAX_BYTES + 1];
     uint32_t index;
     size_t first_character;
     size_t new_text_length;
     
     PageRecord_CopyText(new_text,text,text_length);
     
     /* 去掉首尾空格，避免只输入空格也生成一条新记录。 */
     new_text_length = strlen(new_text);
     first_character = 0;

     while((first_character < new_text_length) &&
           (new_text[first_character] == ' '))
     {
        ++first_character;
     }

     while((new_text_length > first_character) &&
           (new_text[new_text_length - 1] == ' '))
     {
        --new_text_length;
     }

     if(first_character != 0)
     {
        (void)memmove(new_text,
                      &new_text[first_character],
                      new_text_length - first_character);
        new_text_length -= first_character;
     }

     new_text[new_text_length] = '\0';

     if(new_text_length == 0)
     {
        return 0;
     }

     /*
      * 单 Bank Flash 擦除期间 CPU 会停顿。扇区轮换前必须先停止
      * CAN1/CAN2 的所有运动输出，避免电调长时间保持上一帧非零电流。
      */
     if(RecordStorage_NeedsErase() != 0U)
     {
         if((BalanceApp_IsReady() != 0U) &&
            (BalanceApp_StopForFlash() == 0U))
         {
             page_record_storage_error = 1U;
             goto update_ram_history;
         }

         PageFire_Leave();

         if(MotorService_StopAll() == 0U)
         {
             page_record_storage_error = 1U;
             goto update_ram_history;
         }
     }

     //即使写入flash失败，仍然更新ram记录，保证本次上电期间历史界面可以正常显示
     if(RecordStorage_Append((const uint8_t *)new_text,(uint16_t)new_text_length) == 0)
     {
        page_record_storage_error = 1;
     }

update_ram_history:
     //旧记录依次向后移动
     for(index = PAGE_RECORD_HISTORY_COUNT - 1;index > 0;--index)
     {
        (void)memcpy(page_record_history[index],page_record_history[index - 1],sizeof(page_record_history[index]));
     }
     //新记录放到第一项
     (void)memset(page_record_history[0],0,sizeof(page_record_history[0]));
     (void)memcpy(page_record_history[0],new_text,new_text_length + 1);

     return 1;
 }  
 
 
 /* ======================== g0读取 ======================== */
/*
 * 请求读取page2中的g0.txt。
 * TJC屏返回：
 * 0x70 + 字符串 + 0xFF 0xFF 0xFF
 * tjc_hmi.c会将其解析成TJC_EVENT_STRING。
 */
 static PageRecord_Action_t PageRecord_RequestInput(TJC_HMI *hmi,PageRecord_Action_t action_after_text)
 {
    TJC_Status status;
     if(hmi == NULL)
     {
        return action_after_text;
     }
     
     if(page_record_text_request_pending != 0)
     {
        if(action_after_text != PAGE_RECORD_ACTION_NONE)
        {
            page_record_action_after_text = action_after_text;
        }
        return PAGE_RECORD_ACTION_NONE;
     }
     
     status = TJC_Text_RequestText(hmi,PAGE_RECORD_INPUT_OBJECT);
     
     if(status == TJC_OK)
     {
        page_record_text_request_pending = 1;
         page_record_action_after_text = action_after_text;
         return PAGE_RECORD_ACTION_NONE;
     }
     return action_after_text;
 }
 
 
 /* ======================== 触摸事件 ======================== */
static PageRecord_Action_t PageRecord_HandleTouch(TJC_HMI *hmi,const TJC_Event *event)
{
    if((hmi == NULL) || (event == NULL))
    {
        return PAGE_RECORD_ACTION_NONE;
    }
    if(event->touch_event != PAGE_RECORD_TOUCH_RELEASE)
    {
        return PAGE_RECORD_ACTION_NONE;
    }        
    //////////PAGE2
    if(event->page_id == PAGE_RECORD_INPUT_PAGE_ID)
    {
        if(event->component_id == PAGE_RECORD_HISTORY_ID)
        {
            /*
             * 始终在离开page2之前读取g0。
             *
             * 不能依赖g0或键盘页面额外发送触摸/页面事件来判断文本
             * 是否变化，否则屏幕没有配置这些事件时，上电后的第一次输入
             * 会被误认为已经同步，page4只能显示空记录。
             */
            return PageRecord_RequestInput(hmi,PAGE_RECORD_ACTION_SHOW_HISTORY);
        }
        if(event->component_id == PAGE_RECORD_BACK_ID)
        {
            return PageRecord_RequestInput(hmi,PAGE_RECORD_ACTION_RETURN_HOME);
        }
    }
    
    /////////////PAGE4
    if((event->page_id == PAGE_RECORD_HISTORY_PAGE_ID) && (event->component_id == PAGE_HISTORY_BACK_ID))
    {
        return PAGE_RECORD_ACTION_RETURN_HOME;
    }
    return PAGE_RECORD_ACTION_NONE;
}



/* ======================== 对外接口 ======================== */

void PageRecord_Init(void)
{
    uint8_t record_count;
    uint8_t index;
    
    (void)memset(page_record_history,0,sizeof(page_record_history));
    
    page_record_text_request_pending = 0;
    page_record_action_after_text = PAGE_RECORD_ACTION_NONE;
    page_record_storage_error = 0;
    
    if(RecordStorage_Init() == 0)
    {
        page_record_storage_error = 1;
        return;
    }
    
    record_count = RecordStorage_GetRecentCount();
    
    if(record_count > PAGE_RECORD_HISTORY_COUNT)
    {
        record_count = PAGE_RECORD_HISTORY_COUNT;
    }
    
    for(index = 0;index < record_count;++index)
    {
        if(RecordStorage_ReadRecent(index,page_record_history[index],sizeof(page_record_history[index])) == 0)
        {
            page_record_storage_error = 1;
            break;
        }
    }
}

void PageRecord_OnDisplayReset(void)
{
    page_record_text_request_pending = 0;
    page_record_action_after_text = PAGE_RECORD_ACTION_NONE;
    
}

void PageRecord_OnPageChanged(TJC_HMI *hmi,uint8_t previous_page,uint8_t current_page)
{
    (void)previous_page;

    if(current_page == PAGE_RECORD_HISTORY_PAGE_ID)
    {
        PageRecord_ShowHistory(hmi);
    }
}

PageRecord_Action_t PageRecord_HandleEvent(TJC_HMI *hmi,const TJC_Event *event)
{
    PageRecord_Action_t action;

    if ((hmi == NULL) || (event == NULL))
    {
        return PAGE_RECORD_ACTION_NONE;
    }

    if (event->type == TJC_EVENT_TOUCH)
    {
        return PageRecord_HandleTouch(hmi,event);
    }

    if ((event->type != TJC_EVENT_STRING) || (page_record_text_request_pending == 0))
    {
        return PAGE_RECORD_ACTION_NONE;
    }

    /*
     * 只有收到有效的新文字时才追加记录。保存后清空g0，避免再次
     * 进入page2而未输入新内容时，把上一条文字重复保存和轮换。
     */
    if(PageRecord_StoreText(event->data,event->data_len) != 0)
    {
        (void)TJC_Text_SetText(hmi,PAGE_RECORD_INPUT_OBJECT,"");
    }
    page_record_text_request_pending = 0;
    action = page_record_action_after_text;
    page_record_action_after_text = PAGE_RECORD_ACTION_NONE;

    return action;
}

void PageRecord_ShowHistory(TJC_HMI *hmi)
{
    if (hmi == NULL)
    {
        return;
    }

    (void)TJC_Text_SetText(hmi,PAGE_HISTORY_TEXT1_OBJECT,page_record_history[0]);

    (void)TJC_Text_SetText(hmi,PAGE_HISTORY_TEXT2_OBJECT,page_record_history[1]);

    (void)TJC_Text_SetText(hmi,PAGE_HISTORY_TEXT3_OBJECT,page_record_history[2]);

    (void)TJC_Text_SetText(hmi,PAGE_HISTORY_TEXT4_OBJECT,page_record_history[3]);
}

uint8_t PageRecord_HasStorageError(void)
{
    return page_record_storage_error;
}

