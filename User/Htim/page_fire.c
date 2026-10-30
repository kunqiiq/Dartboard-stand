#include "page_fire.h"
#include "main.h"
#include "motor_service.h"
#include "servo.h"
#include <stdbool.h>
#include <stdint.h>

//调试变量
volatile uint32_t fire_3508_rx_count = 0;
volatile uint32_t fire_6020_rx_count = 0;

volatile uint32_t fire_3508_nonzero_count = 0;
volatile uint32_t fire_6020_nonzero_count = 0;

volatile float fire_3508_rpm_debug = 0.0f;
volatile float fire_6020_rpm_debug = 0.0f;

volatile float fire_3508_last_nonzero_rpm = 0.0f;
volatile float fire_6020_last_nonzero_rpm = 0.0f;

volatile uint32_t fire_bad_3508_cmd_count = 0;

/* 发射页面ID */
#define PAGE_FIRE_PAGE_ID                 2
/* 控件ID */
#define PAGE_FIRE_BACK_ID                 2
#define PAGE_FIRE_SERVO_ID                3
#define PAGE_FIRE_2006_FRONT_ID           4
#define PAGE_FIRE_2006_REVERSE_ID         5
#define PAGE_FIRE_3508_FRONT_ID           7
#define PAGE_FIRE_6020_FRONT_ID           8
#define PAGE_FIRE_3508_REVERSE_ID         9
#define PAGE_FIRE_6020_REVERSE_ID         10
/* 标准触摸状态 */
#define PAGE_FIRE_TOUCH_RELEASE           0
#define PAGE_FIRE_TOUCH_PRESS             1
/* 自定义协议：只保留心跳 */
#define PAGE_FIRE_FRAME_HEADER            0xA5
#define PAGE_FIRE_CMD_HEARTBEAT           0x7F
/* 安全参数 */
#define PAGE_FIRE_LINK_TIMEOUT_MS         300
/* 电机固定转速 */
#define PAGE_FIRE_2006_SPEED_RPM          35.0f
#define PAGE_FIRE_3508_SPEED_RPM          300.0f
#define PAGE_FIRE_6020_SPEED_RPM          300.0f
/* 舵机角度 */
#define PAGE_FIRE_SERVO_IDLE_ANGLE        120.0f
#define PAGE_FIRE_SERVO_FIRE_ANGLE        180.0f

#define PAGE_FIRE_SERVO_FIRE_ANGLE  180.0f

typedef struct
{
    float target_3508_rpm;
    float target_6020_rpm;
    float servo_angle;

    /* M2006按钮状态 */
    bool forward_pressed;
    bool reverse_pressed;

    /* M3508按钮状态 */
    bool front_3508_pressed;
    bool reverse_3508_pressed;

    /* GM6020按钮状态 */
    bool front_6020_pressed;
    bool reverse_6020_pressed;

    /* 屏幕通信状态 */
    bool link_received;
    bool failsafe_active;

    uint32_t last_link_tick;

} PageFireControl_t;

static PageFireControl_t page_fire;


/**
 * @brief 更新串口屏通信状态
 */
static void PageFire_UpdateLink(void)
{
    page_fire.last_link_tick = HAL_GetTick();
    page_fire.link_received = true;
    page_fire.failsafe_active = false;
}


/**
 * @brief 应用M2006正反转按钮状态
 */
static void PageFire_Apply2006(void)
{
    RM_Motor_t *motor;

    motor = motor_nodes[MOTOR_SERVICE_NODE_2006].motor;

    /*
     * 只有一个方向按钮按下时才运行。
     * 两个都未按下或两个同时按下时停止。
     */
    if (page_fire.forward_pressed &&
        !page_fire.reverse_pressed)
    {
        MotorService_SetSpeedTarget(
            motor,
            PAGE_FIRE_2006_SPEED_RPM
        );

        MotorService_SetEnabled(
            motor,
            true
        );
    }
    else if (page_fire.reverse_pressed &&
             !page_fire.forward_pressed)
    {
        MotorService_SetSpeedTarget(
            motor,
            -PAGE_FIRE_2006_SPEED_RPM
        );

        MotorService_SetEnabled(
            motor,
            true
        );
    }
    else
    {
        MotorService_SetSpeedTarget(
            motor,
            0.0f
        );

        /*
         * 保持原来的M2006停止逻辑：
         * 松开按钮后失能电机。
         */
        MotorService_SetEnabled(
            motor,
            false
        );
    }
}


/**
 * @brief 应用M3508正反转按钮状态
 */
static void PageFire_Apply3508(void)
{
    RM_Motor_t *motor;
    PID_TypeDef *pid;
    float target_rpm;

    motor = motor_nodes[MOTOR_SERVICE_NODE_3508].motor;
    pid = motor_nodes[MOTOR_SERVICE_NODE_3508].pid;
    target_rpm = 0.0f;

    if (page_fire.front_3508_pressed &&
        !page_fire.reverse_3508_pressed)
    {
        target_rpm = PAGE_FIRE_3508_SPEED_RPM;
    }
    else if (page_fire.reverse_3508_pressed &&
             !page_fire.front_3508_pressed)
    {
        target_rpm = -PAGE_FIRE_3508_SPEED_RPM;
    }
    else
    {
        target_rpm = 0.0f;
    }

    page_fire.target_3508_rpm = target_rpm;
    fire_3508_rpm_debug = target_rpm;

    if (target_rpm != 0.0f)
    {
        fire_3508_nonzero_count++;
        fire_3508_last_nonzero_rpm = target_rpm;
    }

    MotorService_SetSpeedTarget(
        motor,
        target_rpm
    );

    if (target_rpm == 0.0f)
    {
        /*
         * 松开按钮时清除历史积分，
         * 避免之前的积分继续产生输出。
         */
        PID_Reset(pid);
    }

    /*
     * 目标为0时仍然保持使能，
     * 让速度PID主动制动到0 RPM。
     */
    MotorService_SetEnabled(
        motor,
        true
    );
}


/**
 * @brief 应用GM6020正反转按钮状态
 */
static void PageFire_Apply6020(void)
{
    RM_Motor_t *motor;
    PID_TypeDef *pid;
    float target_rpm;

    motor = motor_nodes[MOTOR_SERVICE_NODE_6020].motor;
    pid = motor_nodes[MOTOR_SERVICE_NODE_6020].pid;
    target_rpm = 0.0f;

    if (page_fire.front_6020_pressed &&
        !page_fire.reverse_6020_pressed)
    {
        target_rpm = PAGE_FIRE_6020_SPEED_RPM;
    }
    else if (page_fire.reverse_6020_pressed &&
             !page_fire.front_6020_pressed)
    {
        target_rpm = -PAGE_FIRE_6020_SPEED_RPM;
    }
    else
    {
        target_rpm = 0.0f;
    }

    page_fire.target_6020_rpm = target_rpm;
    fire_6020_rpm_debug = target_rpm;

    if (target_rpm != 0.0f)
    {
        fire_6020_nonzero_count++;
        fire_6020_last_nonzero_rpm = target_rpm;
    }

    MotorService_SetSpeedTarget(
        motor,
        target_rpm
    );

    if (target_rpm == 0.0f)
    {
        /*
         * 松开按钮时清除历史积分。
         */
        PID_Reset(pid);
    }

    /*
     * 目标为0时保持使能，
     * 让速度PID主动制动到0 RPM。
     */
    MotorService_SetEnabled(
        motor,
        true
    );
}


/**
 * @brief 停止发射页面所有执行机构
 */
void PageFire_StopAll(void)
{
    page_fire.target_3508_rpm = 0.0f;
    page_fire.target_6020_rpm = 0.0f;
    page_fire.servo_angle = PAGE_FIRE_SERVO_IDLE_ANGLE;

    page_fire.forward_pressed = false;
    page_fire.reverse_pressed = false;

    page_fire.front_3508_pressed = false;
    page_fire.reverse_3508_pressed = false;

    page_fire.front_6020_pressed = false;
    page_fire.reverse_6020_pressed = false;

    fire_3508_rpm_debug = 0.0f;
    fire_6020_rpm_debug = 0.0f;

    MotorService_SetSpeedTarget(
        motor_nodes[MOTOR_SERVICE_NODE_3508].motor,
        0.0f
    );

    MotorService_SetSpeedTarget(
        motor_nodes[MOTOR_SERVICE_NODE_6020].motor,
        0.0f
    );

    MotorService_SetSpeedTarget(
        motor_nodes[MOTOR_SERVICE_NODE_2006].motor,
        0.0f
    );

    /*
     * 退出页面或通信超时时，彻底失能所有电机。
     */
    MotorService_SetEnabled(
        motor_nodes[MOTOR_SERVICE_NODE_3508].motor,
        false
    );

    MotorService_SetEnabled(
        motor_nodes[MOTOR_SERVICE_NODE_6020].motor,
        false
    );

    MotorService_SetEnabled(
        motor_nodes[MOTOR_SERVICE_NODE_2006].motor,
        false
    );

    SERVO_SetAngle(
        &send_dart,
        PAGE_FIRE_SERVO_IDLE_ANGLE
    );
}


/**
 * @brief 初始化发射页面
 */
void PageFire_Init(void)
{
    page_fire.target_3508_rpm = 0.0f;
    page_fire.target_6020_rpm = 0.0f;
    page_fire.servo_angle = PAGE_FIRE_SERVO_IDLE_ANGLE;

    page_fire.forward_pressed = false;
    page_fire.reverse_pressed = false;

    page_fire.front_3508_pressed = false;
    page_fire.reverse_3508_pressed = false;

    page_fire.front_6020_pressed = false;
    page_fire.reverse_6020_pressed = false;

    page_fire.link_received = false;
    page_fire.failsafe_active = true;
    page_fire.last_link_tick = HAL_GetTick();

    PageFire_StopAll();
}


/**
 * @brief 进入发射页面
 */
void PageFire_Enter(void)
{
    PageFire_StopAll();

    page_fire.link_received = false;
    page_fire.failsafe_active = true;
    page_fire.last_link_tick = HAL_GetTick();
}


/**
 * @brief 离开发射页面
 */
void PageFire_Leave(void)
{
    PageFire_StopAll();

    page_fire.link_received = false;
    page_fire.failsafe_active = true;
}


/**
 * @brief 处理发射页面触摸事件
 */
static PageFire_Action_t PageFire_HandleTouch(
    const TJC_Event *event
)
{
    if (event == NULL)
    {
        return PAGE_FIRE_ACTION_NONE;
    }

    if (event->page_id != PAGE_FIRE_PAGE_ID)
    {
        return PAGE_FIRE_ACTION_NONE;
    }

    if ((event->touch_event != PAGE_FIRE_TOUCH_PRESS) &&
        (event->touch_event != PAGE_FIRE_TOUCH_RELEASE))
    {
        return PAGE_FIRE_ACTION_NONE;
    }

    PageFire_UpdateLink();

    switch (event->component_id)
    {
        /*
         * 退出按钮：b_back，ID 2
         */
        case PAGE_FIRE_BACK_ID:
        {
            if (event->touch_event == PAGE_FIRE_TOUCH_RELEASE)
            {
                /*
                 * 返回首页前停止所有机构。
                 */
                PageFire_Leave();

                return PAGE_FIRE_ACTION_RETURN_HOME;
            }

            break;
        }

        /*
         * 发射舵机按钮：b_servo，ID 3
         */
        case PAGE_FIRE_SERVO_ID:
        {
            if (event->touch_event == PAGE_FIRE_TOUCH_PRESS)
            {
                page_fire.servo_angle =
                    PAGE_FIRE_SERVO_FIRE_ANGLE;
            }
            else
            {
                page_fire.servo_angle =
                    PAGE_FIRE_SERVO_IDLE_ANGLE;
            }

            SERVO_SetAngle(
                &send_dart,
                page_fire.servo_angle
            );

            break;
        }

        /*
         * M2006正转按钮：b_forward，ID 4
         */
        case PAGE_FIRE_2006_FRONT_ID:
        {
            if (event->touch_event == PAGE_FIRE_TOUCH_PRESS)
            {
                page_fire.forward_pressed = true;
                page_fire.reverse_pressed = false;
            }
            else
            {
                page_fire.forward_pressed = false;
            }

            PageFire_Apply2006();

            break;
        }

        /*
         * M2006反转按钮：b_reverse，ID 5
         */
        case PAGE_FIRE_2006_REVERSE_ID:
        {
            if (event->touch_event == PAGE_FIRE_TOUCH_PRESS)
            {
                page_fire.reverse_pressed = true;
                page_fire.forward_pressed = false;
            }
            else
            {
                page_fire.reverse_pressed = false;
            }

            PageFire_Apply2006();

            break;
        }

        /*
         * M3508正转按钮：b_3508_front，ID 7
         */
        case PAGE_FIRE_3508_FRONT_ID:
        {
            fire_3508_rx_count++;

            if (event->touch_event == PAGE_FIRE_TOUCH_PRESS)
            {
                page_fire.front_3508_pressed = true;
                page_fire.reverse_3508_pressed = false;
            }
            else
            {
                page_fire.front_3508_pressed = false;
            }

            PageFire_Apply3508();

            break;
        }

        /*
         * GM6020正转按钮：b_6020_front，ID 8
         */
        case PAGE_FIRE_6020_FRONT_ID:
        {
            fire_6020_rx_count++;

            if (event->touch_event == PAGE_FIRE_TOUCH_PRESS)
            {
                page_fire.front_6020_pressed = true;
                page_fire.reverse_6020_pressed = false;
            }
            else
            {
                page_fire.front_6020_pressed = false;
            }

            PageFire_Apply6020();

            break;
        }

        /*
         * M3508反转按钮：b_3508_reverse，ID 9
         */
        case PAGE_FIRE_3508_REVERSE_ID:
        {
            fire_3508_rx_count++;

            if (event->touch_event == PAGE_FIRE_TOUCH_PRESS)
            {
                page_fire.reverse_3508_pressed = true;
                page_fire.front_3508_pressed = false;
            }
            else
            {
                page_fire.reverse_3508_pressed = false;
            }

            PageFire_Apply3508();

            break;
        }

        /*
         * GM6020反转按钮：b_6020_reverse，ID 10
         */
        case PAGE_FIRE_6020_REVERSE_ID:
        {
            fire_6020_rx_count++;

            if (event->touch_event == PAGE_FIRE_TOUCH_PRESS)
            {
                page_fire.reverse_6020_pressed = true;
                page_fire.front_6020_pressed = false;
            }
            else
            {
                page_fire.reverse_6020_pressed = false;
            }

            PageFire_Apply6020();

            break;
        }

        default:
        {
            break;
        }
    }

    return PAGE_FIRE_ACTION_NONE;
}


/**
 * @brief 处理自定义数据
 *
 * 两个滑块已经删除，因此这里只保留心跳处理。
 */
static void PageFire_HandleCustom(
    const TJC_Event *event
)
{
    if ((event == NULL) ||
        (event->code != PAGE_FIRE_FRAME_HEADER) ||
        (event->data == NULL) ||
        (event->data_len < 1U))
    {
        return;
    }

    if (event->data[0] == PAGE_FIRE_CMD_HEARTBEAT)
    {
        PageFire_UpdateLink();
    }
}


/**
 * @brief 发射页面统一事件入口
 */
PageFire_Action_t PageFire_HandleEvent(
    TJC_HMI *hmi,
    const TJC_Event *event
)
{
    if ((hmi == NULL) || (event == NULL))
    {
        return PAGE_FIRE_ACTION_NONE;
    }

    if (event->type == TJC_EVENT_TOUCH)
    {
        return PageFire_HandleTouch(event);
    }

    if (event->type == TJC_EVENT_CUSTOM)
    {
        PageFire_HandleCustom(event);
    }

    return PAGE_FIRE_ACTION_NONE;
}


/**
 * @brief 发射页面周期任务
 */
void PageFire_Task(void)
{
    uint32_t now;

    now = HAL_GetTick();

    /*
     * 未收到心跳或超过300ms没有通信时：
     * - 停止所有电机；
     * - 电机失能；
     * - 舵机返回120度。
     */
    if ((!page_fire.link_received) ||
        ((uint32_t)(now - page_fire.last_link_tick) >
         PAGE_FIRE_LINK_TIMEOUT_MS))
    {
        if (!page_fire.failsafe_active)
        {
            PageFire_StopAll();
            page_fire.failsafe_active = true;
        }
    }
}

