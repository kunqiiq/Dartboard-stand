/*
 * 飞镖盘TJC/Nextion串口屏管理
 *
 * 使用USART1：
 *
 * TX = PA9
 * RX = PB7
 * 波特率 = 115200
 *
 * 页面：
 *
 * page5 = 上电启动页（页面ID=0）
 * page0 = 首页（页面ID=1）
 * page1 = 发射控制（页面ID=2）
 * page2 = 记录页面（页面ID=3）
 * page3 = 调平测试向导（页面ID=4）
 * keybdA = 在线输入键盘（页面ID=5）
 * page4 = 最近四条记录（页面ID=6）
 *
 * page0按钮：
 *
 * b_balance  ID=1 -> page3
 * b_record   ID=2 -> page2
 * b_fire     ID=3 -> page1
 * b_auto     ID=4 -> 启动自动调平，保留在page0
 */

#include "dartboard_hmi.h"

#include "bsp_uart.h"
#include "balance_app.h"
#include "page_balance.h"
#include "page_fire.h"
#include "page_record.h"
#include "tjc_hmi_stm32_hal.h"
#include "usart.h"

/* ======================== 配置参数 ======================== */

#define DARTBOARD_HMI_RX_BUFFER_SIZE          64

/*
 * 如果屏幕上电时尚未准备好，
 * 500 ms后再次发送page5命令。
 */
#define DARTBOARD_HMI_BOOT_RETRY_MS           500

/* ======================== 页面ID ======================== */

#define DARTBOARD_HMI_PAGE_BOOT               0
#define DARTBOARD_HMI_PAGE_HOME               1
#define DARTBOARD_HMI_PAGE_FIRE               2
#define DARTBOARD_HMI_PAGE_RECORD             PAGE_RECORD_INPUT_PAGE_ID
#define DARTBOARD_HMI_PAGE_BALANCE            4
#define DARTBOARD_HMI_PAGE_KEYBOARD           PAGE_RECORD_KEYBOARD_PAGE_ID
#define DARTBOARD_HMI_PAGE_HISTORY            PAGE_RECORD_HISTORY_PAGE_ID

/* ======================== page0按钮ID ======================== */

#define DARTBOARD_HMI_BUTTON_BALANCE_ID       1
#define DARTBOARD_HMI_BUTTON_RECORD_ID        2
#define DARTBOARD_HMI_BUTTON_FIRE_ID          3
#define DARTBOARD_HMI_BUTTON_AUTO_BALANCE_ID  4

/* TJC标准触摸事件 */
#define DARTBOARD_HMI_TOUCH_RELEASE           0
#define DARTBOARD_HMI_TOUCH_PRESS             1

/* ======================== 内部状态 ======================== */

/* TJC协议和STM32 HAL适配器 */
static TJC_HMI_HAL hmi_adapter;

/* USART1 DMA接收缓冲区 */
static uint8_t
    uart1_rx_buffer[DARTBOARD_HMI_RX_BUFFER_SIZE];

/* 当前屏幕页面 */
static uint8_t current_page =
    DARTBOARD_HMI_PAGE_BOOT;

/* 屏幕启动页面是否已经确认 */
static uint8_t boot_page_confirmed;

/* USART1接收是否需要重新启动 */
static volatile uint8_t
    uart1_rx_restart_required;

/*
 * USART1错误后，在主循环中停止page3测试。
 *
 * 不在UART中断回调里直接停止电机，
 * 避免在中断上下文中发送CAN数据。
 */
static volatile uint8_t
    page_balance_stop_required;

/* USART1异常时在主循环中立即停止发射页执行机构。 */
static volatile uint8_t
    page_fire_stop_required;

/* 初始化时刻 */
static uint32_t init_tick;

/* ======================== USART1接收 ======================== */

static HAL_StatusTypeDef
DartboardHMI_StartReceive(void)
{
    HAL_StatusTypeDef status;

    status = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart1,
        uart1_rx_buffer,
        (uint16_t)sizeof(uart1_rx_buffer));

    if ((status == HAL_OK) &&
        (huart1.hdmarx != NULL))
    {
        /*
         * 禁止DMA半传输中断。
         *
         * 串口屏协议使用空闲中断确定一次接收长度，
         * 不需要半传输事件。
         */
        __HAL_DMA_DISABLE_IT(
            huart1.hdmarx,
            DMA_IT_HT);
    }

    return status;
}

/* ======================== 页面切换 ======================== */

static void DartboardHMI_SetFirePageText(TJC_HMI *hmi)
{
    if(hmi == NULL)
    {
        return;
    }
    (void)TJC_Text_SetText(hmi,"t0","发射界面");
}

static void DartboardHMI_ShowPage(TJC_HMI *hmi,uint8_t page_id)
{
    TJC_Status status;

    if (hmi == NULL)
    {
        return;
    }

    status = TJC_ERR_ARGUMENT;

    switch (page_id)
    {
        case DARTBOARD_HMI_PAGE_BOOT:
        {
            status = TJC_HMI_ShowPage(hmi, "page5");
            break;
        }

        case DARTBOARD_HMI_PAGE_HOME:
        {
            status = TJC_HMI_ShowPage(hmi, "page0");
            break;
        }

        case DARTBOARD_HMI_PAGE_FIRE:
        {
            status = TJC_HMI_ShowPage(hmi, "page1");
            if(status == TJC_OK)
            {
                DartboardHMI_SetFirePageText(hmi);
            }
            break;
        }

        case DARTBOARD_HMI_PAGE_RECORD:
        {
            status = TJC_HMI_ShowPage(hmi, "page2");
            break;
        }

        case DARTBOARD_HMI_PAGE_BALANCE:
        {
            status = TJC_HMI_ShowPage(hmi, "page3");
            break;
        }

        case DARTBOARD_HMI_PAGE_HISTORY:
        {
            status = TJC_HMI_ShowPage(hmi, "page4");
            if (status == TJC_OK)
            {
                PageRecord_ShowHistory(hmi);
            }
            break;
        }

        default:
        {
            break;
        }
    }

    /*
     * 只有发送页面切换命令成功后，
     * 才更新MCU保存的页面状态。
     */
    if (status == TJC_OK)
    {
        uint8_t previous_page = current_page;

        current_page = page_id;

        if ((previous_page == DARTBOARD_HMI_PAGE_FIRE) && (page_id != DARTBOARD_HMI_PAGE_FIRE))
        {
            PageFire_Leave();
        }
        else if ((previous_page != DARTBOARD_HMI_PAGE_FIRE) && (page_id == DARTBOARD_HMI_PAGE_FIRE))
        {
            PageFire_Enter();
        }
    }
}

/* ======================== 页面报告处理 ======================== */

static void DartboardHMI_HandlePageReport(
    TJC_HMI *hmi,
    uint8_t reported_page)
{
    uint8_t previous_page;

    if (reported_page > DARTBOARD_HMI_PAGE_HISTORY)
    {
        return;
    }

    previous_page = current_page;
    current_page = reported_page;

    PageRecord_OnPageChanged(
        hmi,
        previous_page,
        reported_page);

    /*
     * 如果屏幕通过自己的页面指令直接离开page3，
     * 而不是通过b_back离开，也必须停止调平测试。
     */
    if ((previous_page ==
         DARTBOARD_HMI_PAGE_BALANCE) && (reported_page != DARTBOARD_HMI_PAGE_BALANCE))
    {
        if (PageBalance_IsActive() != 0U)
        {
            PageBalance_Leave();
        }
    }

    if ((previous_page == DARTBOARD_HMI_PAGE_FIRE) && (reported_page != DARTBOARD_HMI_PAGE_FIRE))
    {
        PageFire_Leave();
    }

    if ((previous_page != DARTBOARD_HMI_PAGE_FIRE) && (reported_page == DARTBOARD_HMI_PAGE_FIRE))
    {
        PageFire_Enter();
    }

    /*
     * 支持屏幕内部指令直接进入page3。
     *
     * PageBalance_Enter内部有重复进入保护，
     * 不会因为页面报告而重复重置测试向导。
     */
    if (reported_page == DARTBOARD_HMI_PAGE_BALANCE)
    {
        PageBalance_Enter(hmi);
    }
}

/* ======================== page0按钮处理 ======================== */

static void DartboardHMI_HandleHomeTouch(TJC_HMI *hmi,const TJC_Event *event)
{
    if ((hmi == NULL) ||
        (event == NULL))
    {
        return;
    }

    if (event->type != TJC_EVENT_TOUCH)
    {
        return;
    }

    if (event->page_id != DARTBOARD_HMI_PAGE_HOME)
    {
        return;
    }

    /*
     * 首页按钮统一在松开时执行页面跳转，
     * 防止按下和松开各执行一次。
     */
    if (event->touch_event != DARTBOARD_HMI_TOUCH_RELEASE)
    {
        return;
    }

    switch (event->component_id)
    {
        /*
         * page0：b_balance，ID=1
         * 进入page3手动调平测试向导。
         */
        case DARTBOARD_HMI_BUTTON_BALANCE_ID:
        {
            DartboardHMI_ShowPage(hmi,DARTBOARD_HMI_PAGE_BALANCE);

            /*
             * 进入page3后：
             *
             * - 停止自动调平；
             * - 切换到MANUAL模式；
             * - 初始化测试向导；
             * - 准备刷新t_step和t_txt。
             */
            PageBalance_Enter(hmi);
            break;
        }

        /*
         * page0：b_record，ID=2
         */
        case DARTBOARD_HMI_BUTTON_RECORD_ID:
        {
            DartboardHMI_ShowPage(hmi,DARTBOARD_HMI_PAGE_RECORD);
            break;
        }

        /*
         * page0：b_fire，ID=3
         */
        case DARTBOARD_HMI_BUTTON_FIRE_ID:
        {
            DartboardHMI_ShowPage(hmi,DARTBOARD_HMI_PAGE_FIRE);
            break;
        }

        /*
         * page0：b_auto，ID=4
         * 预留给独立的自动调平按钮，松开时启动自动调平。
         */
        case DARTBOARD_HMI_BUTTON_AUTO_BALANCE_ID:
        {
            (void)BalanceApp_StartAuto();
            break;
        }

        default:
        {
            break;
        }
    }
}

/* ======================== TJC事件总入口 ======================== */

static void DartboardHMI_HandleEvent(TJC_HMI *hmi,const TJC_Event *event,void *user)
{
    PageBalance_Action_t balance_action;
    PageFire_Action_t fire_action;
    PageRecord_Action_t record_action;

    (void)user;

    if ((hmi == NULL) ||
        (event == NULL))
    {
        return;
    }

    /*
     * 屏幕启动或复位完成。
     *
     * 如果屏幕在手动测试过程中发生复位，
     * 为安全起见立即终止page3测试。
     */
    if (event->type == TJC_EVENT_READY)
    {
        if (PageBalance_IsActive() != 0)
        {
            PageBalance_Leave();
        }

        PageFire_Leave();

        PageRecord_OnDisplayReset();

        DartboardHMI_ShowPage(hmi,DARTBOARD_HMI_PAGE_BOOT);

        boot_page_confirmed = 1;
        return;
    }

    /*
     * 处理屏幕主动报告的当前页面ID。
     */
    if (event->type == TJC_EVENT_PAGE_ID)
    {
        DartboardHMI_HandlePageReport(hmi,event->page_id);
        return;
    }

    /*
     * page1事件转交给发射页面。
     *
     * 自定义帧本身不包含页面ID，因此先由本层按当前页面拦截；
     * 触摸事件同时校验事件携带的page_id。
     */
    fire_action = PAGE_FIRE_ACTION_NONE;

    if ((current_page == DARTBOARD_HMI_PAGE_FIRE) || ((event->type == TJC_EVENT_TOUCH) && (event->page_id == DARTBOARD_HMI_PAGE_FIRE)))
    {
        fire_action = PageFire_HandleEvent(hmi, event);
    }

    if (fire_action == PAGE_FIRE_ACTION_RETURN_HOME)
    {
        DartboardHMI_ShowPage(hmi,DARTBOARD_HMI_PAGE_HOME);
        return;
    }

    /* page2、page4触摸事件以及g0文本返回事件。 */
    record_action = PageRecord_HandleEvent(hmi, event);

    if (record_action == PAGE_RECORD_ACTION_RETURN_HOME)
    {
        DartboardHMI_ShowPage(hmi,DARTBOARD_HMI_PAGE_HOME);
        return;
    }

    if (record_action ==PAGE_RECORD_ACTION_SHOW_HISTORY)
    {
        DartboardHMI_ShowPage(hmi,DARTBOARD_HMI_PAGE_HISTORY);
        return;
    }

    /*
     * page3事件转交给调平测试页面。
     */
    balance_action = PageBalance_HandleEvent(hmi, event);

    /*
     * page3中的b_back已经先执行：
     *
     * - PageBalance_Leave；
     * - BalanceApp_Stop；
     * - 四个调平电机输出归零。
     *
     * 这里再统一执行页面跳转。
     */
    if (balance_action == PAGE_BALANCE_ACTION_RETURN_HOME)
    {
        DartboardHMI_ShowPage(hmi,DARTBOARD_HMI_PAGE_HOME);
        return;
    }

    /*
     * 处理首页的三个页面选择按钮。
     */
    DartboardHMI_HandleHomeTouch(hmi,event);
}

/* ======================== BSP UART回调 ======================== */

/*
 * 该函数由bsp_uart.c的
 * HAL_UARTEx_RxEventCallback统一调用。
 *
 * 注意：
 * 这里通常运行在中断上下文中，
 * 只复制数据和设置标志，不执行复杂业务。
 */
static void DartboardHMI_UART1RxCallback(
    uint16_t size)
{
    size_t accepted;

    if (size > (uint16_t)sizeof(uart1_rx_buffer))
    {
        size = (uint16_t)sizeof(uart1_rx_buffer);
    }

    accepted = TJC_HMI_HAL_QueueRxFromISR(&hmi_adapter,uart1_rx_buffer,(size_t)size);

    /*
     * 队列满时TJC适配器内部会记录溢出次数。
     * 此处不在中断中打印信息。
     */
    (void)accepted;

    /*
     * ReceiveToIdle DMA使用NORMAL模式，
     * 每次接收完成后都需要重新启动。
     */
    if (DartboardHMI_StartReceive() != HAL_OK)
    {
        uart1_rx_restart_required = 1;

        /*
         * 如果当前正在page3测试，
         * 在主循环中执行安全停止。
         */
        page_balance_stop_required = 1;
        page_fire_stop_required = 1;
    }
}

/*
 * 该函数由bsp_uart.c的
 * HAL_UART_ErrorCallback统一调用。
 */
static void DartboardHMI_UART1ErrorCallback(void)
{
    uart1_rx_restart_required = 1;
    page_balance_stop_required = 1;
    page_fire_stop_required = 1;
}

/* ======================== 初始化 ======================== */

void DartboardHMI_Init(void)
{
    TJC_HMI *hmi;

    /*
     * 初始化TJC HAL适配器。
     *
     * 发送超时时间设置为100 ms。
     */
    TJC_HMI_HAL_Init(&hmi_adapter,&huart1,100U);

    hmi = TJC_HMI_HAL_Core(&hmi_adapter);

    /*
     * 注册所有TJC协议事件的总回调。
     */
    TJC_HMI_SetEventCallback(hmi,DartboardHMI_HandleEvent,NULL);

    /*
     * 向公共BSP UART层注册USART1回调。
     *
     * 工程中只能由bsp_uart.c实现：
     *
     * HAL_UARTEx_RxEventCallback
     * HAL_UART_ErrorCallback
     *
     * 其他模块不能重复定义这两个HAL回调。
     */
    BSP_UART_RegisterRxCallback(UART_ID_USART1,DartboardHMI_UART1RxCallback);

    BSP_UART_RegisterErrorCallback(UART_ID_USART1,DartboardHMI_UART1ErrorCallback);

    uart1_rx_restart_required = (DartboardHMI_StartReceive() == HAL_OK) ?0 :1;

    page_balance_stop_required = 0;
    page_fire_stop_required = 0;

    PageRecord_Init();

    current_page =
        DARTBOARD_HMI_PAGE_BOOT;

    boot_page_confirmed = 0;

    init_tick = HAL_GetTick();

    /*
     * 先立即发送一次page5。
     * 如果屏幕还在启动，500 ms后任务中会再次发送。
     */
    DartboardHMI_ShowPage(hmi,DARTBOARD_HMI_PAGE_BOOT);
}

/* ======================== 周期任务 ======================== */

void DartboardHMI_Task(void)
{
    TJC_HMI *hmi;

    hmi = TJC_HMI_HAL_Core(&hmi_adapter);

    /*
     * USART1通信错误后，
     * 在主循环而不是中断回调中停止page3测试。
     */
    if (page_balance_stop_required != 0)
    {
        page_balance_stop_required = 0;

        if (PageBalance_IsActive() != 0)
        {
            PageBalance_Leave();

            DartboardHMI_ShowPage(hmi,DARTBOARD_HMI_PAGE_HOME);
        }
    }

    if (page_fire_stop_required != 0)
    {
        page_fire_stop_required = 0;
        PageFire_Leave();
    }

    /*
     * 尝试恢复USART1 DMA接收。
     */
    if (uart1_rx_restart_required != 0)
    {
        (void)HAL_UART_AbortReceive(&huart1);

        if (DartboardHMI_StartReceive() == HAL_OK)
        {
            uart1_rx_restart_required = 0;
        }
    }

    /*
     * 从接收环形缓冲区取出数据，
     * 解析完整TJC协议帧并分发事件。
     */
    (void)TJC_HMI_HAL_Process(&hmi_adapter);

    /*
     * 如果没有收到屏幕READY事件，
     * 初始化500 ms后再次发送page5。
     */
    if ((boot_page_confirmed == 0) &&((uint32_t)(HAL_GetTick() - init_tick) >= DARTBOARD_HMI_BOOT_RETRY_MS))
    {
        DartboardHMI_ShowPage(hmi, DARTBOARD_HMI_PAGE_BOOT);

        boot_page_confirmed = 1;
    }
}

/* ======================== 状态查询 ======================== */

uint8_t DartboardHMI_GetCurrentPage(void)
{
    return current_page;
}
