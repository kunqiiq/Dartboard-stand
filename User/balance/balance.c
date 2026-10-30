/*
接收IMU输出的俯仰角，横滚角和角速度，通过四个电机升降
*/
#include "balance.h"
#include <math.h>

#define MOTOR_FL 0
#define MOTOR_FR 1
#define MOTOR_BL 2
#define MOTOR_BR 3

#define BALANCE_DT                      0.001f

#define ZERO_DOWN_SPEED                 100.0f      //找底速度
#define ZERO_ROLLBACK_SPEED             100.0f      //回退速度
#define ZERO_ROLLBACK_ECD               8192L       //回退位置
#define ZERO_HOLD_COUNT                 100         //触底/回退后保持周期数

#define LEVEL_SPEED_LIMIT               200.0f      //调平最大速度
#define LEVEL_DESCEND_SPEED             50.0f       //整体下降速度
#define MOTOR_CURRENT_LIMIT             2000.0f     //电流限幅
#define LIMIT_DEBOUNCE_COUNT             8          //开关消抖次数

#define STALL_TARGET_SPEED_MIN           30.0f      //判断堵转时至少达到的目标速度
#define STALL_ACTUAL_SPEED_MAX           5.0f       
#define STALL_CURRENT_MIN                1200       //判断堵转时的电流
#define STALL_CONFIRM_COUNT              300        //堵转需满足300次

#define UPPER_LIMIT_MARGIN_ECD           256L       //靠近机械上限的时候保留的安全余量

#define PITCH_TOLERANCE                 0.01f       //pitch允许误差
#define ROLL_TOLERANCE                  0.01f       //roll允许误差
#define PITCH_LEVEL_OFFSET              0.0f        //pitch补偿
#define ROLL_LEVEL_OFFSET               0.0f        //roll补偿
#define LEVEL_SPEED_DEADBAND            5.0f        //速度死区
#define LEVEL_ACTUAL_SPEED_MAX          8.0f        //稳定时电机实际转速上限
#define LEVEL_ANGULAR_RATE_MAX          0.03f       //稳定时平台角速度上限(rad/s)
#define LEVEL_STABLE_COUNT              500         //稳定判断次数
#define MOTOR_MAX_TEMPERATURE_C          80U

#define MOTOR_ECD_RANGE                 8192
#define MOTOR_ECD_HALF_RANGE            4096


/*
 * 根据实际机构行程填写。
 * 当前先设置约 20 圈电机编码器。
 */
#define UPPER_SOFT_LIMIT_ECD             (20L * 8192L)

//超时限制
#define MOTOR_OFFLINE_TIMEOUT_MS         100
#define FIND_BOTTOM_TIMEOUT_MS         15000
#define ROLLBACK_TIMEOUT_MS            10000
#define LEVEL_TIMEOUT_MS               20000
#define LEVEL_DESCEND_TIMEOUT_MS       20000
#define REFERENCE_DESCEND_TIMEOUT_MS   15000
#define THREE_AXIS_LEVEL_TIMEOUT_MS    20000

#define AXIS_TEST_SPEED_LIMIT          20.0f
#define AXIS_TEST_DURATION_MAX_MS      1000

#define NO_REFERENCE_AXIS                0xFF

typedef struct
{
    Balance_State_t state;
    Balance_Error_t error;
    Balance_Command_t command;      //外部下达的指令
    Balance_Result_t result;        //执行结果

    BalancePID_t pitch_pid;
    BalancePID_t roll_pid;
    BalancePID_t speed_pid[4];

    int32_t total_ecd[4];
    int16_t last_ecd[4];
    int32_t zero_ecd[4];
    int32_t pending_zero_ecd[4];      //等待确认的零点

    uint8_t encoder_initialized[4];     //是否已经碰到底部限位
    uint8_t zero_found[4];              //是否找到零点
    uint16_t zero_hold_count[4];        
    uint8_t rollback_done[4];
    uint8_t zero_valid;
    uint8_t bottom_limit[4];            //消抖后的限位
    uint8_t bottom_limit_count[4];
    uint16_t stall_count[4];            //堵转次数

    float pitch;
    float roll;
    float pitch_rate;
    float roll_rate;

    int16_t motor_current[4];   //最终发送给电机的电流
    uint8_t reference_axis;     //整体下降阶段选出的参考支撑轴
    uint32_t stable_count;      //姿态连续稳定的次数
    uint32_t state_start_tick;  //当前状态开始的时间
    
    uint8_t axis_test_axis;
    float axis_test_speed;
    uint32_t axis_test_duration_ms;
    Balance_AxisTestResult_t axis_test_result;
    uint8_t motor_link_was_ready;
    
} Balance_Controller_t;

static Balance_Controller_t balance;

//电机方向
static const float motor_direction[4] = {1.0f,1.0f,1.0f,1.0f};

static const float pitch_sign[4] = {1.0f,1.0f,-1.0f,-1.0f};     //前升后降
    
static const float roll_sign[4] = {1.0f,-1.0f,1.0f,-1.0f};      //左升右降

static uint8_t Balance_SendCurrents(int16_t fl, int16_t fr,
                                    int16_t bl, int16_t br)
{
    const int16_t currents[BALANCE_MOTOR_COUNT] = {fl, fr, bl, br};
    return BalanceMotor_SetCurrents(currents);
}

//状态进入时间
static void Balance_EnterState(Balance_State_t state)
{
    balance.state = state;
    balance.state_start_tick = HAL_GetTick();
    balance.stable_count = 0;
}

//复位
static void Balance_ResetControllers(void)
{
    uint8_t i;

    balance.stable_count = 0;
    balance.reference_axis = NO_REFERENCE_AXIS;
    balance.axis_test_axis = NO_REFERENCE_AXIS;
    balance.axis_test_speed = 0.0f;
    balance.axis_test_duration_ms = 0;
    
    BalancePID_Set_Integral_Error(&balance.pitch_pid, 0.0f);
    BalancePID_Set_Integral_Error(&balance.roll_pid, 0.0f);
    for (i = 0; i < BALANCE_MOTOR_COUNT; i++)
    {
        BalancePID_Set_Integral_Error(&balance.speed_pid[i], 0.0f);
        balance.zero_found[i] = 0;
        balance.zero_hold_count[i] = 0;
        balance.rollback_done[i] = 0;
        balance.pending_zero_ecd[i] = 0;
        balance.stall_count[i] = 0;
    }
}

static void Balance_InvalidatePositionReference(void)
{
    uint8_t i;

    balance.zero_valid = 0U;
    balance.reference_axis = NO_REFERENCE_AXIS;

    for (i = 0U; i < BALANCE_MOTOR_COUNT; i++)
    {
        balance.encoder_initialized[i] = 0U;
        balance.total_ecd[i] = 0;
        balance.last_ecd[i] = 0;
        balance.zero_ecd[i] = 0;
        balance.pending_zero_ecd[i] = 0;
    }
}

//状态超时
static uint8_t Balance_StateTimeout( uint32_t timeout)
{
    return ((HAL_GetTick() - balance.state_start_tick) >= timeout) ? 1 : 0;
}

//下限开关读取：低电平有效
static uint8_t Balance_ReadBottomLimitRaw(uint8_t index)
{
    GPIO_PinState state;

    switch (index)
    {
        case MOTOR_FL:
            state = HAL_GPIO_ReadPin(LIMIT_FL_GPIO_Port,LIMIT_FL_Pin);
            break;

        case MOTOR_FR:
            state = HAL_GPIO_ReadPin(LIMIT_FR_GPIO_Port,LIMIT_FR_Pin);
            break;

        case MOTOR_BL:
            state = HAL_GPIO_ReadPin(LIMIT_BL_GPIO_Port,LIMIT_BL_Pin);
            break;

        case MOTOR_BR:
            state = HAL_GPIO_ReadPin(LIMIT_BR_GPIO_Port,LIMIT_BR_Pin);
            break;

        default:
            return 0;
    }

    return (state == GPIO_PIN_RESET) ? 1 : 0;
}

//限位开关消抖
static void Balance_UpdateBottomLimits(void)
{
    uint8_t i;

    for (i = 0; i < BALANCE_MOTOR_COUNT; i++)
    {
        if (Balance_ReadBottomLimitRaw(i) != 0)
        {
            if (balance.bottom_limit_count[i] < LIMIT_DEBOUNCE_COUNT)
            {
                balance.bottom_limit_count[i]++;
            }
            if (balance.bottom_limit_count[i] >= LIMIT_DEBOUNCE_COUNT)
            {
                balance.bottom_limit[i] = 1;
            }
        }
        else
        {
            balance.bottom_limit_count[i] = 0;
            balance.bottom_limit[i] = 0;
        }
    }
}


static uint8_t Balance_ReadBottomLimit(uint8_t index)
{
    return (index < BALANCE_MOTOR_COUNT) ? balance.bottom_limit[index] : 0;
}

//电流限幅
static int16_t Balance_LimitCurrent(float current)
{
    if (current > MOTOR_CURRENT_LIMIT)
    {
        current = MOTOR_CURRENT_LIMIT;
    }
    else if (current < -MOTOR_CURRENT_LIMIT)
    {
        current = -MOTOR_CURRENT_LIMIT;
    }

    return (int16_t)current;
}

//电机在线检测
static uint8_t Balance_MotorsOnline(void)
{
    uint8_t i;
    uint32_t now;

    now = HAL_GetTick();
    BalanceMotor_UpdateTimeout();

    for (i = 0; i < 4; i++)
    {
        const BalanceMotor_Feedback_t *motor;

        motor = BalanceMotor_GetFeedback(i);

        if ((motor == NULL) || (motor->online == 0U) ||
            ((uint32_t)(now - motor->last_update_tick) > MOTOR_OFFLINE_TIMEOUT_MS))
        {
            return 0;
        }
    }

    return 1;
}

//多圈编码器累计
static void Balance_UpdateEncoder(void)
{
    uint8_t i;

    for (i = 0; i < 4; i++)
    {
        const BalanceMotor_Feedback_t *motor;
        int32_t difference;

        motor = BalanceMotor_GetFeedback(i);
        if (motor == NULL)
        {
            continue;
        }

        if (balance.encoder_initialized[i] == 0)
        {
            balance.last_ecd[i] = (int16_t)motor->angle;

            balance.total_ecd[i] = 0;
            balance.encoder_initialized[i] = 1;
            continue;
        }

        difference = (int32_t)motor->angle - (int32_t)balance.last_ecd[i];

        if (difference > MOTOR_ECD_HALF_RANGE)
        {
            difference -= MOTOR_ECD_RANGE;
        }
        else if (difference < -MOTOR_ECD_HALF_RANGE)
        {
            difference += MOTOR_ECD_RANGE;
        }

        balance.total_ecd[i] += difference;
        balance.last_ecd[i] = (int16_t)motor->angle;
    }
}

//当前升高量:编码器计数形式的高度
static int32_t Balance_GetHeight(uint8_t index)
{
    return (int32_t)( (balance.total_ecd[index] - balance.zero_ecd[index]) * motor_direction[index]);
}

//下限开关都找到
static uint8_t Balance_AllZeroFound(void)
{
    uint8_t i;

    for (i = 0; i < 4; i++)
    {
        if (balance.zero_found[i] == 0)
        {
            return 0;
        }
    }
    return 1;
}

//回退
static uint8_t Balance_AllRollbackDone(void)
{
    uint8_t i;

    for (i = 0; i < 4; i++)
    {
        if (balance.rollback_done[i] == 0)
        {
            return 0;
        }
    }

    return 1;
}

//电机速度闭环
static uint8_t Balance_SpeedControl( const float target_speed[4])
{
    uint8_t i;
    //电机在线
    if (Balance_MotorsOnline() == 0)
    {
        Balance_Stop( BALANCE_ERROR_MOTOR_OFFLINE);

        return 0;
    }

    for (i = 0; i < 4; i++)
    {
        const BalanceMotor_Feedback_t *motor;
        float physical_speed;
        int32_t height;

        motor = BalanceMotor_GetFeedback(i);
        if (motor == NULL)
        {
            Balance_Stop(BALANCE_ERROR_MOTOR_OFFLINE);
            return 0U;
        }
        if (motor->temperature >= MOTOR_MAX_TEMPERATURE_C)
        {
            Balance_Stop(BALANCE_ERROR_MOTOR_OVERHEAT);
            return 0U;
        }
        physical_speed = target_speed[i];
        height = Balance_GetHeight(i);
        
        //下限保护
        if ((Balance_ReadBottomLimit(i) != 0) && (physical_speed < 0.0f))
        {
            physical_speed = 0.0f;
            BalancePID_Set_Integral_Error(&balance.speed_pid[i], 0.0f);
        }
        //软件上限保护
        if ((balance.zero_valid != 0) && (height >= (UPPER_SOFT_LIMIT_ECD - UPPER_LIMIT_MARGIN_ECD)) && (physical_speed > 0.0f))
        {
            physical_speed = 0.0f;
            BalancePID_Set_Integral_Error(&balance.speed_pid[i], 0.0f);
        }

        if ((balance.zero_valid != 0) && (height > (UPPER_SOFT_LIMIT_ECD + UPPER_LIMIT_MARGIN_ECD)))
        {
            Balance_Stop(BALANCE_ERROR_UPPER_LIMIT);
            return 0;
        }

        BalancePID_Set_Target(&balance.speed_pid[i],physical_speed *motor_direction[i]);
        BalancePID_Set_Now(&balance.speed_pid[i],(float)motor->speed_rpm);
        BalancePID_Adjust(&balance.speed_pid[i]);
        //限幅
        balance.motor_current[i] = Balance_LimitCurrent( BalancePID_Get_Out(&balance.speed_pid[i]));

        if ((Math_Abs_Float(physical_speed) >= STALL_TARGET_SPEED_MIN) &&
            (Math_Abs_Float((float)motor->speed_rpm) <= STALL_ACTUAL_SPEED_MAX) &&
            (Math_Abs_Float((float)motor->current) >= STALL_CURRENT_MIN))
        {
            if (balance.stall_count[i] < STALL_CONFIRM_COUNT)
            {
                balance.stall_count[i]++;
            }
            if (balance.stall_count[i] >= STALL_CONFIRM_COUNT)
            {
                Balance_Stop(BALANCE_ERROR_MOTOR_STALL);
                return 0;
            }
        }
        else
        {
            balance.stall_count[i] = 0;
        }
    }

    if (Balance_SendCurrents(
        balance.motor_current[MOTOR_FL],
        balance.motor_current[MOTOR_FR],
        balance.motor_current[MOTOR_BL],
        balance.motor_current[MOTOR_BR]) == 0)
    {
        Balance_Stop(BALANCE_ERROR_CAN_TX);
        return 0;
    }

    return 1;
}

//姿态调平速度计算
static uint8_t Balance_CalculateLevelSpeed(float target_speed[4],const uint8_t allowed[4])
{
    float pitch_output;
    float roll_output;
    float pp = 0.0f;
    float pr = 0.0f;
    float rr = 0.0f;
    float determinant;
    float pitch_factor;
    float roll_factor;
    uint8_t i;

    BalancePID_Set_Target(&balance.pitch_pid,PITCH_LEVEL_OFFSET);
    BalancePID_Set_Now(&balance.pitch_pid,balance.pitch);
    BalancePID_Adjust(&balance.pitch_pid);
    pitch_output =BalancePID_Get_Out(&balance.pitch_pid);
    
    BalancePID_Set_Target(&balance.roll_pid,ROLL_LEVEL_OFFSET);
    BalancePID_Set_Now(&balance.roll_pid,balance.roll);
    BalancePID_Adjust(&balance.roll_pid);
    roll_output =BalancePID_Get_Out(&balance.roll_pid);

    //额外增加微分阻尼：抑制平台振荡，根据实际情况改动
    pitch_output -= 5.0f * balance.pitch_rate;
    roll_output -= 5.0f * balance.roll_rate;

    if ((!isfinite(pitch_output)) || (!isfinite(roll_output)))
    {
        return 0U;
    }

    for (i = 0; i < BALANCE_MOTOR_COUNT; i++)
    {
        if (allowed[i] != 0)
        {
            pp += pitch_sign[i] * pitch_sign[i];
            pr += pitch_sign[i] * roll_sign[i];
            rr += roll_sign[i] * roll_sign[i];
        }
    }
    
    determinant = pp * rr - pr * pr;
    
    if ((!isfinite(determinant)) || (determinant <= 0.000001f))
    {
        return 0;
    }
    pitch_factor = (rr * pitch_output - pr * roll_output) / determinant;
    roll_factor = (-pr * pitch_output + pp * roll_output) / determinant;

    for (i = 0; i < 4; i++)
    {
        if (allowed[i] == 0)
        {
            target_speed[i] = 0.0f;
            continue;
        }

        target_speed[i] = pitch_sign[i] * pitch_factor + roll_sign[i] * roll_factor;

        if (!isfinite(target_speed[i]))
        {
            return 0;
        }

        if (target_speed[i] >LEVEL_SPEED_LIMIT)
        {
            target_speed[i] =LEVEL_SPEED_LIMIT;
        }
        else if (target_speed[i] <-LEVEL_SPEED_LIMIT)
        {
            target_speed[i] =-LEVEL_SPEED_LIMIT;
        }
    }

    return 1;
}

//水平稳定判断
static uint8_t Balance_IsLevelStable(const float speed[4],const uint8_t allowed[4])
{
    uint8_t i;

    if (Math_Abs_Float(balance.pitch - PITCH_LEVEL_OFFSET) >PITCH_TOLERANCE)
    {
        return 0;
    }

    if (Math_Abs_Float(balance.roll - ROLL_LEVEL_OFFSET) >ROLL_TOLERANCE)
    {
        return 0;
    }

    if ((Math_Abs_Float(balance.pitch_rate) > LEVEL_ANGULAR_RATE_MAX) ||
        (Math_Abs_Float(balance.roll_rate) > LEVEL_ANGULAR_RATE_MAX))
    {
        return 0U;
    }

    for (i = 0; i < 4; i++)
    {
        if ((allowed[i] != 0) &&(Math_Abs_Float(speed[i]) >LEVEL_SPEED_DEADBAND))
        {
            return 0;
        }


        if (allowed[i] != 0U)
        {
            const BalanceMotor_Feedback_t *motor =
                BalanceMotor_GetFeedback(i);

            if ((motor == NULL) ||
                (Math_Abs_Float((float)motor->speed_rpm) >
                 LEVEL_ACTUAL_SPEED_MAX))
            {
                return 0U;
            }
        }
    }

    return 1;
}

//寻找四根轴高度最低的作为参考轴
static uint8_t Balance_SelectLowestAxis(void)
{
    uint8_t i;
    uint8_t reference;
    int32_t min_height;
    int32_t height;

    reference = MOTOR_FL;
    min_height = Balance_GetHeight(MOTOR_FL);

    for (i = 1; i < 4; i++)
    {
        height = Balance_GetHeight(i);

        if (height < min_height)
        {
            min_height = height;
            reference = i;
        }
    }

    return reference;
}

//寻找零点限位
static void Balance_FindBottom(void)
{
    float speed[4];
    uint8_t i;

    if (Balance_StateTimeout(FIND_BOTTOM_TIMEOUT_MS) != 0)
    {
        Balance_Stop(BALANCE_ERROR_FIND_BOTTOM_TIMEOUT);
        return;
    }

    for (i = 0; i < 4; i++)
    {
        if (Balance_ReadBottomLimit(i) != 0)
        {
            speed[i] = 0.0f;

            if (balance.zero_found[i] == 0)
            {
                balance.zero_found[i] = 1;
                balance.pending_zero_ecd[i] = balance.total_ecd[i];
                balance.zero_hold_count[i] = 0;
            }
            else if (balance.zero_hold_count[i] < ZERO_HOLD_COUNT)
            {
                balance.zero_hold_count[i]++;
            }
        }
        else
        {
            /* 释放开关会撤销没有锁定状态的判断 */
            if ((balance.zero_found[i] != 0) && (balance.zero_hold_count[i] < ZERO_HOLD_COUNT))
            {
                balance.zero_found[i] = 0;
                balance.zero_hold_count[i] = 0;
            }
            speed[i] = (balance.zero_found[i] != 0) ? 0.0f : -ZERO_DOWN_SPEED;
        }
    }

    if (Balance_SpeedControl(speed) == 0)
    {
        return;
    }

    if (Balance_AllZeroFound() != 0)
    {
        uint8_t all_held = 1;
        for (i = 0U; i < BALANCE_MOTOR_COUNT; i++)
        {
            if (balance.zero_hold_count[i] < ZERO_HOLD_COUNT)
            {
                all_held = 0;
                break;
            }
        }
        if (all_held != 0)
        {
            for (i = 0; i < BALANCE_MOTOR_COUNT; i++)
            {
                balance.zero_ecd[i] = balance.pending_zero_ecd[i];
            }
            balance.zero_valid = 1;
            Balance_EnterState(BALANCE_STATE_ROLLBACK);
        }
    }
}

//回退过程
static void Balance_Rollback(void)
{
    float speed[4];
    uint8_t i;

    if (Balance_StateTimeout(ROLLBACK_TIMEOUT_MS) != 0)
    {
        Balance_Stop(BALANCE_ERROR_ROLLBACK_TIMEOUT);
        return;
    }

    for (i = 0; i < 4; i++)
    {
        if (Balance_GetHeight(i) >=ZERO_ROLLBACK_ECD)
        {
            if (Balance_ReadBottomLimit(i) != 0U)
            {
                Balance_Stop(BALANCE_ERROR_LIMIT_CONFLICT);
                return;
            }
            speed[i] = 0.0f;
            balance.rollback_done[i] = 1;
        }
        else
        {
            speed[i] =ZERO_ROLLBACK_SPEED;
        }
    }

    if (Balance_SpeedControl(speed) == 0)
    {
        return;
    }

    if (Balance_AllRollbackDone() != 0)
    {
        balance.stable_count++;
        if (balance.stable_count >= ZERO_HOLD_COUNT)
        {
            balance.result = BALANCE_RESULT_SUCCEEDED;
            Balance_EnterState(BALANCE_STATE_HOLD);
        }
    }
    else
    {
        balance.stable_count = 0;
    }
}

//自动调平过程
static void Balance_LevelAdjust(void)
{
    float speed[4];
    uint8_t allowed[4] = {1, 1, 1, 1};

    if (Balance_StateTimeout(LEVEL_TIMEOUT_MS) != 0)
    {
        Balance_Stop(BALANCE_ERROR_LEVEL_TIMEOUT);
        return;
    }

    if (Balance_CalculateLevelSpeed(speed, allowed) == 0)
    {
        Balance_Stop(BALANCE_ERROR_GEOMETRY);
        return;
    }

    if (Balance_SpeedControl(speed) == 0)
    {
        return;
    }

    if (Balance_IsLevelStable(speed,allowed) != 0)
    {
        balance.stable_count++;
    }
    else
    {
        balance.stable_count = 0;
    }

    if (balance.stable_count >=LEVEL_STABLE_COUNT)
    {
        if(balance.command == BALANCE_COMMAND_LEVEL_ONLY)
      {
         balance.result = BALANCE_RESULT_SUCCEEDED;
         Balance_EnterState(BALANCE_STATE_HOLD);
      }
      else
      {
        balance.reference_axis =Balance_SelectLowestAxis();
        Balance_EnterState(BALANCE_STATE_LEVEL_DESCEND);   
      }
        

    }
}

//整体下降
static void Balance_LevelDescend(void)
{
    float speed[4];
    uint8_t allowed[4] = {1, 1, 1, 1};
    uint8_t i;

    if (Balance_StateTimeout(LEVEL_DESCEND_TIMEOUT_MS) != 0)
    {
        Balance_Stop(BALANCE_ERROR_LEVEL_DESCEND_TIMEOUT);
        return;
    }

    if (Balance_CalculateLevelSpeed(speed, allowed) == 0)
    {
        Balance_Stop(BALANCE_ERROR_GEOMETRY);
        return;
    }

    for (i = 0; i < 4; i++)
    {
        speed[i] -= LEVEL_DESCEND_SPEED;
    }

        // 参考轴到达下限位，进入三轴调平
    if (Balance_ReadBottomLimit(balance.reference_axis) != 0)
    {
        (void)Balance_SendCurrents(0, 0, 0, 0);
        Balance_EnterState(BALANCE_STATE_THREE_AXIS_LEVEL);
        return;
    }

        //其他轴比参考轴提前触底， 改为只让参考轴下降
    for (i = 0; i < 4; i++)
    {
        if ((i != balance.reference_axis) &&(Balance_ReadBottomLimit(i) != 0))
        {
            (void)Balance_SendCurrents(0, 0, 0, 0);
            Balance_EnterState(BALANCE_STATE_REFERENCE_DESCEND);
            return;
        }
    }

    (void)Balance_SpeedControl(speed);
}

//参考轴独自下降
static void Balance_ReferenceDescend(void)
{
    float speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    if (Balance_StateTimeout(REFERENCE_DESCEND_TIMEOUT_MS) != 0)
    {
        Balance_Stop(BALANCE_ERROR_REFERENCE_DESCEND_TIMEOUT);
        return;
    }

    if (balance.reference_axis >=BALANCE_MOTOR_COUNT)
    {
        Balance_Stop(BALANCE_ERROR_REFERENCE_DESCEND_TIMEOUT);
        return;
    }

        // 参考轴到达下限位
    if (Balance_ReadBottomLimit(balance.reference_axis) != 0)
    {
        (void)Balance_SendCurrents(0, 0, 0, 0);
        Balance_EnterState(BALANCE_STATE_THREE_AXIS_LEVEL);
        return;
    }

        //只让参考轴向下运动
    speed[balance.reference_axis] =-LEVEL_DESCEND_SPEED;

    (void)Balance_SpeedControl(speed);
}


//三轴调平
static void Balance_ThreeAxisLevel(void)
{
    float speed[4];
    uint8_t allowed[4] = {1, 1, 1, 1};

    if (Balance_StateTimeout(THREE_AXIS_LEVEL_TIMEOUT_MS) != 0)
    {
        Balance_Stop(BALANCE_ERROR_THREE_AXIS_TIMEOUT);
        return;
    }

    if (balance.reference_axis >=BALANCE_MOTOR_COUNT)
    {
        Balance_Stop(BALANCE_ERROR_THREE_AXIS_TIMEOUT);
        return;
    }

        //已触底的参考轴不再参与调平
    allowed[balance.reference_axis] = 0;

    if (Balance_CalculateLevelSpeed(speed, allowed) == 0)
    {
        Balance_Stop(BALANCE_ERROR_GEOMETRY);
        return;
    }

    speed[balance.reference_axis] = 0.0f;

    if (Balance_SpeedControl(speed) == 0)
    {
        return;
    }

    if (Balance_IsLevelStable(speed,allowed) != 0)
    {
        balance.stable_count++;
    }
    else
    {
        balance.stable_count = 0;
    }

    if (balance.stable_count >=LEVEL_STABLE_COUNT)
    {
        balance.result = BALANCE_RESULT_SUCCEEDED;
        Balance_EnterState(BALANCE_STATE_HOLD);
    }
}

static void Balance_AxisTestUpdate(void)
{
    float speed[4] = {0.0f,0.0f,0.0f,0.0f};
    if(balance.axis_test_axis >= BALANCE_MOTOR_COUNT)
    {
        balance.axis_test_result = BALANCE_AXIS_TEST_INVALID;
        Balance_Stop(BALANCE_ERROR_INVALID_STATE);
        return;
    }
    
    if((HAL_GetTick() - balance.state_start_tick) >= balance.axis_test_duration_ms)
    {
        (void)Balance_SendCurrents(0,0,0,0);
        balance.axis_test_result = BALANCE_AXIS_TEST_SUCCEEDED;
        balance.result = BALANCE_RESULT_SUCCEEDED;
        Balance_EnterState(BALANCE_STATE_HOLD);
        return;
    }
    
    if((Balance_ReadBottomLimit(balance.axis_test_axis) != 0) && 
        (balance.axis_test_speed < 0.0f))
    {
        (void)Balance_SendCurrents(0,0,0,0);
        balance.axis_test_result = BALANCE_AXIS_TEST_LIMITED;
        balance.result = BALANCE_RESULT_FAILED;
        Balance_EnterState(BALANCE_STATE_HOLD);
        return;
    }
    
    speed[balance.axis_test_axis] = balance.axis_test_speed;
    if(Balance_SpeedControl(speed) == 0)
    {
        if(balance.error == BALANCE_ERROR_MOTOR_OFFLINE)
            balance.axis_test_result = BALANCE_AXIS_TEST_OFFLINE;
        else if(balance.error == BALANCE_ERROR_MOTOR_STALL)
            balance.axis_test_result = BALANCE_AXIS_TEST_STALLED;
        else
            balance.axis_test_result = BALANCE_AXIS_TEST_INVALID;
    }
}

//初始化
void Balance_Init(void)
{
    uint8_t i;

    balance.error = BALANCE_ERROR_NONE;
    balance.command = BALANCE_COMMAND_IDLE;
    balance.result = BALANCE_RESULT_IDLE;
    balance.reference_axis =NO_REFERENCE_AXIS;
    balance.zero_valid = 0;
    
    balance.axis_test_axis = NO_REFERENCE_AXIS;
    balance.axis_test_speed = 0.0f;
    balance.axis_test_duration_ms = 0;
    balance.axis_test_result = BALANCE_AXIS_TEST_IDLE;
    balance.motor_link_was_ready = 0U;

    balance.pitch = 0.0f;
    balance.roll = 0.0f;
    balance.pitch_rate = 0.0f;
    balance.roll_rate = 0.0f;

    for (i = 0U; i < 4; i++)
    {
        balance.total_ecd[i] = 0;
        balance.last_ecd[i] = 0;
        balance.zero_ecd[i] = 0;
        balance.pending_zero_ecd[i] = 0;

        balance.encoder_initialized[i] = 0;
        balance.zero_found[i] = 0;
        balance.zero_hold_count[i] = 0;
        balance.rollback_done[i] = 0;
        balance.motor_current[i] = 0;
        balance.bottom_limit[i] = 0;
        balance.bottom_limit_count[i] = 0;
        balance.stall_count[i] = 0;

        BalancePID_Init(
            &balance.speed_pid[i],
            5.0f,
            0.05f,
            0.0f,
            0.0f,
            1000.0f,
            MOTOR_CURRENT_LIMIT,
            BALANCE_DT,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            BALANCE_PID_D_FIRST_DISABLE);
    }

    BalancePID_Init(
        &balance.pitch_pid,
        100.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        LEVEL_SPEED_LIMIT,
        BALANCE_DT,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        BALANCE_PID_D_FIRST_DISABLE);

    BalancePID_Init(
        &balance.roll_pid,
        100.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        LEVEL_SPEED_LIMIT,
        BALANCE_DT,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        BALANCE_PID_D_FIRST_DISABLE);

    Balance_EnterState(BALANCE_STATE_WAIT_READY);

    Balance_SendCurrents(0, 0, 0, 0);
}

void Balance_SetCommand(Balance_Command_t command)
{
    if(balance.axis_test_result == BALANCE_AXIS_TEST_RUNNING)
    {
        balance.axis_test_result = BALANCE_AXIS_TEST_ABORTED;
    }
    
    (void)Balance_SendCurrents(0, 0, 0, 0);
    Balance_ResetControllers();
    balance.command = command;
    balance.error = BALANCE_ERROR_NONE;

    if (command == BALANCE_COMMAND_IDLE)
    {
        balance.result = BALANCE_RESULT_IDLE;
        Balance_EnterState(BALANCE_STATE_HOLD);
    }
    else if (command == BALANCE_COMMAND_ABORT)
    {
        balance.result = BALANCE_RESULT_ABORTED;
        Balance_EnterState(BALANCE_STATE_HOLD);
    }
    else if (command == BALANCE_COMMAND_ZERO_CALIBRATE)
    {
        balance.result = BALANCE_RESULT_RUNNING;
        Balance_EnterState(BALANCE_STATE_WAIT_READY);
    }
    else if ((command == BALANCE_COMMAND_LEVEL_ONLY) || (command == BALANCE_COMMAND_LEVEL))
    {
        if (balance.zero_valid == 0)
        {
            balance.result = BALANCE_RESULT_FAILED;
            balance.error = BALANCE_ERROR_NO_VALID_ZERO;
            Balance_EnterState(BALANCE_STATE_ERROR);
        }
        else
        {
            balance.result = BALANCE_RESULT_RUNNING;
            Balance_EnterState(BALANCE_STATE_LEVEL_ADJUST);
        }
    }
    else
    {
        balance.result = BALANCE_RESULT_FAILED;
        balance.error = BALANCE_ERROR_INVALID_STATE;
        Balance_EnterState(BALANCE_STATE_ERROR);
    }
}

//状态更新
void Balance_Update(float pitch,float roll,float pitch_rate,float roll_rate,uint8_t imu_valid)
{
    uint8_t motors_online;
    
    Balance_UpdateBottomLimits();
    motors_online = Balance_MotorsOnline();
    if(motors_online != 0)
    {
        balance.motor_link_was_ready = 1U;
        Balance_UpdateEncoder();
    }
    else if(balance.motor_link_was_ready != 0U)
    {
        /*
         * 离线期间电机可能滑动或编码器跨圈。不再信任旧的
         * 多圈位置与零点，恢复后必须重新找底。
         */
        balance.motor_link_was_ready = 0U;
        Balance_InvalidatePositionReference();
    }
    
    /* 空闲指令不依赖实时传感器 */
    if ((balance.state == BALANCE_STATE_HOLD) ||
        (balance.state == BALANCE_STATE_ERROR))
    {
        (void)Balance_SendCurrents(0, 0, 0, 0);
        return;
    }
    
    if((balance.state == BALANCE_STATE_LEVEL_ADJUST) ||
        (balance.state == BALANCE_STATE_LEVEL_DESCEND) ||
          (balance.state == BALANCE_STATE_THREE_AXIS_LEVEL))
    {
        if((imu_valid == 0) || (!isfinite(pitch)) || (!isfinite(roll)) ||
            (!isfinite(pitch_rate)) || (!isfinite(roll_rate)))
        {
            Balance_Stop((imu_valid == 0) ? BALANCE_ERROR_IMU_OFFLINE : BALANCE_ERROR_NONFINITE);
            return;
        }
        balance.pitch = pitch;
        balance.roll = roll;
        balance.pitch_rate = pitch_rate;
        balance.roll_rate = roll_rate;
    }
    
    if(balance.state == BALANCE_STATE_WAIT_READY)
    {
        Balance_SendCurrents(0,0,0,0);
        if(balance.command != BALANCE_COMMAND_ZERO_CALIBRATE)
        {
            return;
        }
        if(motors_online != 0)
        {
            if((balance.encoder_initialized[0] != 0) &&
               (balance.encoder_initialized[1] != 0) &&
               (balance.encoder_initialized[2] != 0) &&
               (balance.encoder_initialized[3] != 0) )
            {
                Balance_EnterState(BALANCE_STATE_FIND_BOTTOM);
            }
        }
        return;
    }

    if (Balance_MotorsOnline() == 0)
    {
        Balance_Stop(BALANCE_ERROR_MOTOR_OFFLINE);
        return;
    }

//    Balance_UpdateEncoder();

    switch (balance.state)
    {
        case BALANCE_STATE_FIND_BOTTOM:
            Balance_FindBottom();
            break;

        case BALANCE_STATE_ROLLBACK:
            Balance_Rollback();
            break;

        case BALANCE_STATE_LEVEL_ADJUST:
            Balance_LevelAdjust();
            break;

        case BALANCE_STATE_LEVEL_DESCEND:
            Balance_LevelDescend();
            break;

        case BALANCE_STATE_REFERENCE_DESCEND:
            Balance_ReferenceDescend();
            break;

        case BALANCE_STATE_THREE_AXIS_LEVEL:
            Balance_ThreeAxisLevel();
            break;
        
        case BALANCE_STATE_AXIS_TEST:
            Balance_AxisTestUpdate();
            break;

        case BALANCE_STATE_HOLD:
            Balance_SendCurrents(0, 0, 0, 0);
            break;

        default:
            Balance_Stop(BALANCE_ERROR_INVALID_STATE);
            break;
    }
}


void Balance_Stop(Balance_Error_t error)
{
    balance.error = error;
    balance.result = BALANCE_RESULT_FAILED;
    balance.state = BALANCE_STATE_ERROR;

    Balance_SendCurrents(0, 0, 0, 0);
}


Balance_State_t Balance_GetState(void)
{
    return balance.state;
}


Balance_Error_t Balance_GetError(void)
{
    return balance.error;
}

Balance_Command_t Balance_GetCommand(void)
{
    return balance.command;
}

Balance_Result_t Balance_GetResult(void)
{
    return balance.result;
}

uint8_t Balance_ZeroValid(void)
{
    return balance.zero_valid;
}


uint8_t Balance_StartAxisTest(uint8_t axis,float target_speed,uint32_t duration_ms)
{
    if((axis >= BALANCE_MOTOR_COUNT) || (!isfinite(target_speed)) ||
        (target_speed == 0.0f) || (duration_ms == 0) || 
        (duration_ms > AXIS_TEST_DURATION_MAX_MS))
    {
        balance.axis_test_result = BALANCE_AXIS_TEST_INVALID;
        return 0;
    }
    
    if(Balance_MotorsOnline() == 0)
    {
        balance.axis_test_result = BALANCE_AXIS_TEST_OFFLINE;
        return 0;
    }
    
    if(target_speed > AXIS_TEST_SPEED_LIMIT)
        target_speed = AXIS_TEST_SPEED_LIMIT;
    else if(target_speed < -AXIS_TEST_SPEED_LIMIT)
        target_speed = -AXIS_TEST_SPEED_LIMIT;
    
    Balance_ResetControllers();
    balance.command = BALANCE_COMMAND_IDLE;
    balance.error = BALANCE_ERROR_NONE;
    balance.result = BALANCE_RESULT_RUNNING;
    balance.axis_test_axis = axis;
    balance.axis_test_speed = target_speed;
    balance.axis_test_duration_ms = duration_ms;
    balance.axis_test_result = BALANCE_AXIS_TEST_RUNNING;
    Balance_EnterState(BALANCE_STATE_AXIS_TEST);
    return 1;
}

void Balance_AbortAxisTest(void)
{
    if(balance.axis_test_result == BALANCE_AXIS_TEST_RUNNING)
        balance.axis_test_result = BALANCE_AXIS_TEST_ABORTED;
    (void)Balance_SendCurrents(0,0,0,0);
    balance.result = BALANCE_RESULT_ABORTED;
    Balance_EnterState(BALANCE_STATE_HOLD);
}


uint8_t Balance_AxisTestRunning(void)
{
    return (balance.axis_test_result == BALANCE_AXIS_TEST_RUNNING) ? 1:0;
}

Balance_AxisTestResult_t Balance_GetAxisTestResult(void)
{
    return balance.axis_test_result;
}

uint8_t Balance_GetAxisDiagnostic(uint8_t axis,Balance_AxisDiagnostic_t *diagnostic)
{
    const BalanceMotor_Feedback_t *motor;
    uint32_t now;
    
    if((axis >= BALANCE_MOTOR_COUNT) || (diagnostic == NULL))
        return 0;

    motor = BalanceMotor_GetFeedback(axis);
    now = HAL_GetTick();
    if (motor == NULL)
    {
        return 0U;
    }

    diagnostic->online = ((motor->online != 0U) &&
                          ((uint32_t)(now - motor->last_update_tick) <=
                           MOTOR_OFFLINE_TIMEOUT_MS)) ? 1U : 0U;
    
    diagnostic->bottom_limit_raw = Balance_ReadBottomLimitRaw(axis);
    diagnostic->bottom_limit_active = balance.bottom_limit[axis];
    diagnostic->ecd = motor->angle;
    diagnostic->total_ecd = balance.total_ecd[axis];
    diagnostic->height = balance.zero_valid ? Balance_GetHeight(axis) : 0;
    diagnostic->speed_rpm = motor->speed_rpm;
    diagnostic->given_current = motor->current;
    diagnostic->temperature = motor->temperature;
    return 1;
}

