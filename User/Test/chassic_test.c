/*]
调平系统测试向导
1.can电机在线检查
2.四轴正反方向检查
3.四个下限开关检查
4.IMU数据及方向检查
5.编码器高度方向检查
6.零点标定
7.仅四轴静态调平
8.完整调平流程
9.总结
操作：
 - b_bottom短按：下一步、重试；测试运行中短按则中止
 - b_bottom长按1秒：紧急停止并重置向导
*/

#include "chassic_test.h"
#include "main.h"
#include "balance.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define TEST_PRINT_PERIOD_MS   200
#define TEST_LIMIT_MONITOR_DURATION_MS  5000
#define TEST_IMU_MONITOR_DURATION_MS    6000
#define TEST_AXIS_SPEED   20.0f
#define TEST_AXIS_MOVE_MS  250
#define TEST_AXIS_PAUSE_MS  300

//文字控件字符最大值设置200，与控件相对应
#define TEST_MESSAGE_SIZE   176
#define TEST_RESULT_COUNT   10
#define TEST_AXIS_COUNT     4

typedef enum
{
    TEST_RUN_IDLE = 0,
    TEST_RUN_DIRECTION,
    TEST_RUN_LIMIT,
    TEST_RUN_IMU,
    TEST_RUN_ENCODER,
    TEST_RUN_BALANCE_COMMAND
}Test_Run_t;

typedef enum
{
    TEST_RESULT_NONE = 0,
    TEST_RESULT_RUNNING,
    TEST_RESULT_PASS,
    TEST_RESULT_FAIL,
    TEST_RESULT_ABORTED
}Test_Result_t;

typedef enum
{
    AXIS_PHASE_START_POSITIVE = 0,
    AXIS_PHASE_WAIT_POSITIVE,
    AXIS_PHASE_PAUSE_POSITIVE,
    AXIS_PHASE_START_NEGATIVE,
    AXIS_PHASE_WAIT_NEGATIVE,
    AXIS_PHASE_PAUSE_NEGATIVE
}Axis_Phase_t;


static uint8_t test_step;
static uint8_t test_finished;

static Test_Run_t test_run;
static Test_Result_t test_result[TEST_RESULT_COUNT];

static uint32_t test_start_tick;
static uint32_t test_phase_tick;
static uint32_t test_print_tick;

static uint8_t test_axis;
static Axis_Phase_t test_axis_phase;

static int16_t test_positive_peak[TEST_AXIS_COUNT];
static int16_t test_negative_peak[TEST_AXIS_COUNT];

static int32_t test_encoder_start[TEST_AXIS_COUNT];
static int32_t test_encoder_delta[TEST_AXIS_COUNT];

static uint8_t test_limit_high[TEST_AXIS_COUNT];
static uint8_t test_limit_low[TEST_AXIS_COUNT];

static uint32_t test_imu_valid_count;
static uint32_t test_imu_sample_count;

static char test_message[TEST_MESSAGE_SIZE];
static uint32_t test_message_revision;

/////////////////////////////////////////////////////////文字输出

static void Test_SetMessage(const char *format,...)
{
    va_list args;
    int length;
    int index;
    
    if(format == NULL)
    {
        return;
    }
    
    va_start(args,format);
    length = vsnprintf(test_message,sizeof(test_message),format,args);
    va_end(args);
    
    if(length < 0)
    {
        test_message[0] = '\0';
        return;
    }
    if(length >= (int)sizeof(test_message))
    {
        length = (int)sizeof(test_message) - 1;
    }
    
    //TJC_HMI_SetTextProperty不接受：双引号，OXFF，小于0X20的控制字符
    //因此将换行，回车，和双引号统一替换成空格
    for(index = 0;index < length;index++)
    {
    unsigned char value = (unsigned char)test_message[index];
        
        if((value < 0x20) || (value == (unsigned char)'"') || (value == 0XFF))
        {
            test_message[index] = ' ';
        }
    }
    //删除字符串尾部多余的空格
    while((length > 0) && (test_message[length - 1] == ' '))
    {
        length--;
    }
    test_message[length] = '\0';
    test_message_revision++;
}


///////////////////////////////////////////////////////公共辅助函数
static const char * Test_AxisName(uint8_t axis)
{
    switch(axis)
    {
        case 0:
            return "FL";
        case 1:
            return "FR";
        case 2:
            return "BL";
        case 3:
            return "BR";
        default:
            return "UNKNOW";
    }
}

static const char *Test_StateName(Balance_State_t state)
{
    switch(state)
    {
        case BALANCE_STATE_WAIT_READY:
            return "WAIT_READY";
        case BALANCE_STATE_FIND_BOTTOM:
            return "FIND_BOTTOM";
        case BALANCE_STATE_ROLLBACK:
            return "ROLLBACK";
        case BALANCE_STATE_LEVEL_ADJUST:
            return "LEVEL_ADJUST";
        case BALANCE_STATE_LEVEL_DESCEND:
            return "LEVEL_DESCEND";
        case BALANCE_STATE_REFERENCE_DESCEND:
            return "REFERENCE_DESCEND";
        case BALANCE_STATE_THREE_AXIS_LEVEL:
            return "THREE_AXIS_LEVEL";
        case BALANCE_STATE_AXIS_TEST:
            return "AXIS_TEST";
        case BALANCE_STATE_HOLD:
            return "HOLD";
        case BALANCE_STATE_ERROR:
            return "ERROR";
        default:
            return "UNKNOW";
    }
}

static void Test_SetResult(uint8_t step,Test_Result_t result)
{
    if(step < TEST_RESULT_COUNT)
    {
        test_result[step] = result;
    }
}


static void Test_ResetAxisMeasurements(void)
{
    uint8_t axis;
    
    for(axis = 0;axis < TEST_AXIS_COUNT;axis++)
    {
        test_positive_peak[axis] = 0;
        test_negative_peak[axis] = 0;
        
        test_encoder_start[axis] = 0;
        test_encoder_delta[axis] = 0;
    }
}

///////////////////////////////////////////////////////////////////////停止与故障
void ChassisTest_ForceStop(void)
{
    if(Balance_AxisTestRunning() != 0)
    {
        Balance_AbortAxisTest();
    }
    Balance_SetCommand(BALANCE_COMMAND_ABORT);
    test_run = TEST_RUN_IDLE;
}

 void ChassisTest_ReportFatal(const char *message)
 {
    ChassisTest_ForceStop();
    Test_SetMessage("[FATAL]%s",(message != NULL)?message : "UNKNOW");
}
 
/////////////////////////////////////////////////////////步骤1：CAN检查
static uint8_t Test_AllCanOnline(void)
{
    uint8_t axis;
    uint8_t all_online = 1;
    
    for(axis = 0;axis < TEST_AXIS_COUNT;axis++)
    {
        Balance_AxisDiagnostic_t diagnostic;
        memset(&diagnostic,0,sizeof(diagnostic));
        
        if(Balance_GetAxisDiagnostic(axis,&diagnostic) == 0)
        {
            all_online = 0;
            continue;
        }
        if(diagnostic.online == 0)
        {
            all_online = 0;
        }
    }
    return all_online;
}

static void Test_StartCan(void)
{
    if(Test_AllCanOnline() != 0)
    {
        Test_SetResult(1,TEST_RESULT_PASS);
        Test_SetMessage(
        "Setp1 PASS: CAN ISs 0x201~0x204 online."
        "Confirm FL FR BL BR mapping,then press b_bottom."
        );
    }
    else
    {
        Test_SetResult(1,TEST_RESULT_FAIL);
        Test_SetMessage(
        "Step1 FAIL:one or more CAN motors offline."
        "Check CAN2,power and IDs,then press b_bottom to retry."
        );
    }
}

///////////////////////////////////////////////////////////步骤2：方向检查
static void Test_AxisFailure(uint8_t step,const char *reason)
{
    Balance_AbortAxisTest();
    test_run = TEST_RUN_IDLE;
    
    Test_SetMessage(
    "Step %u Fail,axis %s:%s"
    "Press b_bottom to retry.",
    (unsigned)step,Test_AxisName(test_axis),(reason != NULL)?reason:"UNKNOW"
    );
}

static void Test_StartDirection(uint32_t now)
{
    Balance_SetCommand(BALANCE_COMMAND_IDLE);
    Test_ResetAxisMeasurements();
    
    test_axis = 0;
    test_axis_phase = AXIS_PHASE_START_POSITIVE;
    test_phase_tick = now;
    test_print_tick = now;
    test_run = TEST_RUN_DIRECTION;
    Test_SetResult(2,TEST_RESULT_RUNNING);
    Test_SetMessage(
    "Step 2 direction:each axis runs +20 then -20 rpm."
    "Confirm + is UP and - is DOWN.b_bottom = abort."
    );
}


static void Test_UpdateDirection(uint32_t now)
{
    Balance_AxisDiagnostic_t diagnostic;
    Balance_AxisTestResult_t result;
    
    if(test_axis >= TEST_AXIS_COUNT)
    {
        test_run = TEST_RUN_IDLE;
        Test_SetResult(2,TEST_RESULT_PASS);
        
        Test_SetMessage(
        "Step 2 PASS: direction sequence complete."
        "Confirm physical directions, then press b_bottom."
        );
        return;
    }
    
    memset(&diagnostic,0,sizeof(diagnostic));
    
    if(Balance_GetAxisDiagnostic(test_axis,&diagnostic) == 0)
    {
        Test_AxisFailure(2,"diagnostic unavaliable");
        return;
    }
    
    switch(test_axis_phase)
    {
        case AXIS_PHASE_START_POSITIVE:
        {
            test_encoder_start[test_axis] = diagnostic.total_ecd;      
            if(Balance_StartAxisTest(test_axis,TEST_AXIS_SPEED,TEST_AXIS_MOVE_MS) == 0)
            {
                Test_AxisFailure(2,"cannot start positive motion");
                return;
            }
            test_axis_phase = AXIS_PHASE_WAIT_POSITIVE;
        break;
        }
    
        case AXIS_PHASE_WAIT_POSITIVE:
        {
            if(diagnostic.speed_rpm > test_positive_peak[test_axis])
            {
                test_positive_peak[test_axis] = diagnostic.speed_rpm;
            }
            if(Balance_AxisTestRunning() == 0)
            {
                result = Balance_GetAxisTestResult();
                if(result != BALANCE_AXIS_TEST_SUCCEEDED)
                {
                    Test_AxisFailure(2,"positive motion stooped by protection");
                    return;
                }
                test_phase_tick = now;
                test_axis_phase = AXIS_PHASE_PAUSE_POSITIVE;
            }
            break;
        }
        
        case AXIS_PHASE_PAUSE_POSITIVE:
        {
            if((uint32_t)(now - test_phase_tick) >= TEST_AXIS_PAUSE_MS)
            {
                test_axis_phase = AXIS_PHASE_START_NEGATIVE;
            }
            break;
        }
        
        case AXIS_PHASE_START_NEGATIVE:
        {
            if(Balance_StartAxisTest(test_axis,-TEST_AXIS_SPEED,TEST_AXIS_MOVE_MS) == 0)
            {
                Test_AxisFailure(2,"cannot start negative motion");
                return;
            }
            test_axis_phase = AXIS_PHASE_WAIT_NEGATIVE;
            break;
        }
        
        case AXIS_PHASE_WAIT_NEGATIVE:
        {
            if(diagnostic.speed_rpm < test_negative_peak[test_axis])
            {
                test_negative_peak[test_axis] = diagnostic.speed_rpm;
            }
            if(Balance_AxisTestRunning() == 0)
            {
                result = Balance_GetAxisTestResult();
                if(result != BALANCE_AXIS_TEST_SUCCEEDED)
                {
                    Test_AxisFailure(2,"negative motion stopeed by protection");
                    return;
                }
                test_phase_tick = now;
                test_axis_phase = AXIS_PHASE_PAUSE_NEGATIVE;
            }
            break;        
        }
        case AXIS_PHASE_PAUSE_NEGATIVE:
        default:
        {
            if((uint32_t)(now - test_phase_tick) >= TEST_AXIS_PAUSE_MS)
            {
                (void)Balance_GetAxisDiagnostic(test_axis,&diagnostic);
                test_encoder_delta[test_axis] = 
                diagnostic.total_ecd - test_encoder_start[test_axis];
                test_axis++;
                test_axis_phase = AXIS_PHASE_START_POSITIVE;
                
                if(test_axis >= TEST_AXIS_COUNT)
                {
                    test_run = TEST_RUN_IDLE;
                    Test_SetResult(2,TEST_RESULT_PASS);
                    Test_SetMessage("Step 2 PASS:sequence complete."
                    "Confirm +UP and -DOWN,then press b_bottom");
                    return;
                }
            }
            break;
        }
    }
    
    
    if((uint32_t)(now - test_print_tick) >= TEST_PRINT_PERIOD_MS)
    {
        test_print_tick = now;
        Test_SetMessage(
        "Step 2: %s phase=%u rpm =%d ecd=%ld limit=%u",
        Test_AxisName(test_axis),
        (unsigned)test_axis_phase,
        diagnostic.speed_rpm,
        (long)diagnostic.total_ecd,
        (unsigned)diagnostic.bottom_limit_active
        );
    }
    
}


///////////////////////////////////////////////////////////步骤3：限位检查
static void Test_StartLimit(uint32_t now)
{
    uint8_t axis;
    for(axis = 0;axis < TEST_AXIS_COUNT;axis++)
    {
        test_limit_high[axis] = 0;
        test_limit_low[axis] = 0;
    }
    test_start_tick = now;
    test_print_tick = now;
    test_run = TEST_RUN_LIMIT;
    Test_SetResult(3,TEST_RESULT_RUNNING);
    Test_SetMessage(
    "Step 3 LIMIT: release all switches, then press every "
    "bottom switch within 5 seconds."
    );
}

  
static void Test_UpdateLimit(uint32_t now)
{
    uint8_t axis;
    uint8_t pass = 1;
    
    Balance_AxisDiagnostic_t diagnostic[TEST_AXIS_COUNT];
    memset(diagnostic,0,sizeof(diagnostic));
    
    for (axis = 0; axis < TEST_AXIS_COUNT; axis++)
    {
        if (Balance_GetAxisDiagnostic(axis,&diagnostic[axis]) == 0)
        {
            pass = 0;
            continue;
        }

        if (diagnostic[axis].bottom_limit_raw != 0)
        {
            test_limit_high[axis] = 1;
        }
        else
        {
            test_limit_low[axis] = 1;
        }
    }
    
    if((uint32_t)(now - test_print_tick) >= TEST_PRINT_PERIOD_MS)
    {
        test_print_tick = now;
        Test_SetMessage(
            "Step 3: raw=%u%u%u%u active=%u%u%u%u",
            (unsigned)diagnostic[0].bottom_limit_raw,
            (unsigned)diagnostic[1].bottom_limit_raw,
            (unsigned)diagnostic[2].bottom_limit_raw,
            (unsigned)diagnostic[3].bottom_limit_raw,
            (unsigned)diagnostic[0].bottom_limit_active,
            (unsigned)diagnostic[1].bottom_limit_active,
            (unsigned)diagnostic[2].bottom_limit_active,
            (unsigned)diagnostic[3].bottom_limit_active);
    }
    
    
    if((uint32_t)(now - test_start_tick) >= TEST_LIMIT_MONITOR_DURATION_MS)
    {
        uint8_t toggled;
        for(axis = 0;axis < TEST_AXIS_COUNT;axis++)
        {
            toggled = (uint8_t)(test_limit_high[axis]&&test_limit_low[axis]);
            if(toggled == 0)
            {
                pass = 0;
            }
        }
        test_run = TEST_RUN_IDLE;
        if(pass != 0)
        {
            Test_SetResult(3,TEST_RESULT_PASS);
            Test_SetMessage(
            "Step 3 PASS: all four bottom switches toggled."
            "Press b_botto, for IMU test."
            );
        }
        else
        {
            Test_SetResult(3,TEST_RESULT_FAIL);
            Test_SetMessage(
            "Step 3 FAIL:not all switches toggled."
            "Press b_bottom to retry."
            );
        }
    }
    
}


///////////////////////////////////////////////////////////步骤4：IMU检查
static void Test_StartImu(uint32_t now)
{
    test_start_tick = now;
    test_print_tick = now;
    
    test_imu_valid_count = 0;
    test_imu_sample_count = 0;
    
    test_run  = TEST_RUN_IMU;
    Test_SetResult(4,TEST_RESULT_RUNNING);
    
    Test_SetMessage(
    "Step 4 IMU:raise FRONT => pitch positive;"
    "raise LEFT => roll positive.Observe for 6 seconds."
    );
}

static void Test_UpdateImu(uint32_t now,const IMU_Attitude_t *imu,uint8_t imu_valid)
{
    uint8_t pass;
    test_imu_sample_count++;
    if((imu_valid != 0) && (imu != NULL)&&(imu->data_valid != 0))
    {
        test_imu_valid_count++;
    }
    if(((uint32_t)(now - test_print_tick) >= TEST_PRINT_PERIOD_MS) && (imu != NULL))
    {
        test_print_tick = now;
        
        Test_SetMessage(
        "Step 4:pitch=%.4f roll=%.4f"
        "pitchRate=%.3f rollRate=%.3f valid=%u",
        imu->pitch,
        imu->roll,
        imu->pitch_rate,
        imu->roll_rate,
        (unsigned)imu_valid
        );
    }
    
    if((uint32_t)(now - test_start_tick) >= TEST_IMU_MONITOR_DURATION_MS)
    {
        pass = 0;
        if((test_imu_sample_count != 0) && ((test_imu_valid_count * 100 / test_imu_sample_count) >= 95))
        {
            pass = 1;
        }
        test_run = TEST_RUN_IDLE;
        
        if(pass != 0)
        {
            Test_SetResult(4,TEST_RESULT_PASS);
            Test_SetMessage(
            "Step 4 PASS:IMU valid=%lu/%lu."
            "Confirm pitch and roll signs,then press b_bottom.",
            (unsigned long)test_imu_valid_count,
            (unsigned long)test_imu_sample_count
            );
        }
        else
        {
            Test_SetResult(4,TEST_RESULT_FAIL);
            Test_SetMessage(
            "Step 4 FAIL: IMU valid=%lu/%lu."
            "Press b_bottom to retry.",
            (unsigned long)test_imu_valid_count,
            (unsigned long)test_imu_sample_count
            );
        }
    }
}


///////////////////////////////////////////////////////////步骤5：编码器检查
static void Test_StartEncoder(uint32_t now)
{
    uint8_t axis;
    Balance_AxisDiagnostic_t diagnostic;
    
    Balance_SetCommand(BALANCE_COMMAND_IDLE);
    
    test_axis = 0;
    test_axis_phase = AXIS_PHASE_START_POSITIVE;
    
    test_phase_tick = now;
    test_print_tick = now;
    
    for(axis = 0;axis < TEST_AXIS_COUNT;axis++)
    {
        memset(&diagnostic,0,sizeof(diagnostic));
        (void) Balance_GetAxisDiagnostic(axis,&diagnostic);
        test_encoder_start[axis] = diagnostic.total_ecd;
        test_encoder_delta[axis] = 0;
    }
    test_run = TEST_RUN_ENCODER;
    Test_SetResult(5,TEST_RESULT_RUNNING);
    Test_SetMessage(
    "Step 5 ENCODER:each axis receive +UP command."
    "Confirm physical movement UP and encoder delta positive"
    );
}

static void Test_UpdateEncoder(uint32_t now)
{
    Balance_AxisDiagnostic_t diagnostic;
    Balance_AxisTestResult_t result;
    
    if(test_axis >= TEST_AXIS_COUNT)
    {
        test_run = TEST_RUN_IDLE;
        Test_SetResult(5,TEST_RESULT_PASS);
        Test_SetMessage(
        "Step 5 PASS:encodr sequence complete."
        "Confirm every delta is positive,then press b_bottom."
        );
        return;
    }
    
    memset(&diagnostic,0,sizeof(diagnostic));
    
    if(Balance_GetAxisDiagnostic(test_axis,&diagnostic) == 0)
    {
        Test_AxisFailure(5,"diagnostic unavaliable");
        return;
    }
    
    if(test_axis_phase == AXIS_PHASE_START_POSITIVE)
    {
        test_encoder_start[test_axis] = diagnostic.total_ecd;
        if(Balance_StartAxisTest(test_axis,TEST_AXIS_SPEED,TEST_AXIS_MOVE_MS) == 0)
        {
            Test_AxisFailure(5,"cannot start positive motion");
            return;
        }
        test_axis_phase = AXIS_PHASE_WAIT_POSITIVE;
    }
    else if(test_axis_phase == AXIS_PHASE_WAIT_POSITIVE)
    {
        if(Balance_AxisTestRunning() == 0)
        {
            result = Balance_GetAxisTestResult();
            if(result != BALANCE_AXIS_TEST_SUCCEEDED)
            {
                Test_AxisFailure(5,"moton stopped by protection");
                return;
            }
            test_phase_tick = now;
            test_axis_phase = AXIS_PHASE_PAUSE_POSITIVE;
        }
    }
    else
    {
        if((uint32_t)(now - test_phase_tick) >= TEST_AXIS_PAUSE_MS)
        {
            (void)Balance_GetAxisDiagnostic(test_axis,&diagnostic);
            test_encoder_delta[test_axis] = diagnostic.total_ecd - test_encoder_start[test_axis];
            
            Test_SetMessage(
            "Step 5:%s encoder delta=%ld;"
            "physical direction must be UP.",
            Test_AxisName(test_axis),
            (long)test_encoder_delta[test_axis]
            );
            
            test_axis++;
            test_axis_phase = AXIS_PHASE_START_POSITIVE;
            
            if(test_axis >= TEST_AXIS_COUNT)
            {
                test_run = TEST_RUN_IDLE;
                Test_SetResult(5,TEST_RESULT_PASS);
                Test_SetMessage(
                "Step 5 PASS:sequence complete."
                "Confirm all encoder deltas positive,"
                "then press b_bottom."
                );
                return;
            }
        }
    }
    
    if((uint32_t)(now - test_print_tick) >= TEST_PRINT_PERIOD_MS)
    {
        test_print_tick = now;
        Test_SetMessage(
        "sTEP 5: %s rpm=%d total=%ld limit=%u",
        Test_AxisName(test_axis),
        diagnostic.speed_rpm,
        (long)diagnostic.total_ecd,
        (unsigned)diagnostic.bottom_limit_active
        );
    }
}


///////////////////////////////////////////////////////////步骤6~8：调平系统
static void Test_StartBalanceCommand(uint8_t step,Balance_Command_t command,const char *name)
{
    test_run = TEST_RUN_BALANCE_COMMAND;
    test_print_tick = HAL_GetTick();
    Test_SetResult(step,TEST_RESULT_RUNNING);
    
    Test_SetMessage(
    "Step %u %s running."
    "Press b_bottom during motion to abort.",
    (unsigned)step,
        (name != NULL) ? name : "BALANCE"
    );
    
    Balance_SetCommand(command);
    
    if(Balance_GetResult() == BALANCE_RESULT_FAILED)
    {
        test_run = TEST_RUN_IDLE;
        Test_SetResult(step,TEST_RESULT_FAIL);
        
        Test_SetMessage(
        "Step %u cannot start : error=%u zerovailid=%u."
        "Press b_bottom to retry.",
        (unsigned)step,
        (unsigned)Balance_GetError(),
        (unsigned)Balance_ZeroValid()
        );
    }
}

static void Test_UpdateBalanceCommand(uint32_t now)
{
    Balance_Result_t result;
    result = Balance_GetResult();
    
    if((uint32_t)(now - test_print_tick) >= TEST_PRINT_PERIOD_MS)
    {
        test_print_tick = now;
        Test_SetMessage(
        "Step %u: state=%s result=%u error=%u zero=%u",
        (unsigned)test_step,
        Test_StateName(Balance_GetState()),
        (unsigned)result,
        (unsigned)Balance_GetError(),
        (unsigned)Balance_ZeroValid()
        );
    }
    
    if(result == BALANCE_RESULT_SUCCEEDED)
    {
        test_run = TEST_RUN_IDLE;
        Test_SetResult(test_step,TEST_RESULT_PASS);
        Test_SetMessage(
        "Step %u PASS: motors stopped."
        "Press b_bottom for next step.",
        (unsigned)test_step
        );
    }
    else if((result == BALANCE_RESULT_FAILED) || (result == BALANCE_RESULT_ABORTED))
    {
        test_run = TEST_RUN_IDLE;
        Test_SetResult(test_step,TEST_RESULT_FAIL);
        Test_SetMessage(
        "Step %u FAIL/ABORT: error=%u."
        "Press b_botom to retry.",
        (unsigned)test_step,
        (unsigned)Balance_GetError()
        );
    }
}


///////////////////////////////////////////////////////////步骤控制
static void Test_PrintSummary(void)
{
    uint8_t step;
    uint8_t all_passed = 1;
    
    for(step = 1;step <= 8;step++)
    {
        if(test_result[step] != TEST_RESULT_PASS)
        {
            all_passed = 0;
            break;
        }
    }
    test_finished = 1;
    test_step = 9;
    
    ChassisTest_ForceStop();
    
    if(all_passed != 0)
    {
        Test_SetMessage(
         "TEST COMPLETE: steps 1~8 PASS. "
            "All balance motors stopped."
        );
    }
    else
    {
        Test_SetMessage(
         "TEST COMPLETE: one or more steps NOT PASS. "
            "Hold b_bottom for 1 second to reset."
        );
    }
}

static void Test_StartStep(uint32_t now)
{
    switch(test_step)
    {
        case 1:
            Test_StartCan();
        break;
        
        case 2:
            Test_StartDirection(now);
        break;
        
        case 3:
            Test_StartLimit(now);
        break;
        
        case 4:
            Test_StartImu(now);
        break;
        
        case 5:
            Test_StartEncoder(now);
        break;
        
        case 6:
            Test_StartBalanceCommand(6,BALANCE_COMMAND_ZERO_CALIBRATE,"ZERO CALIBRATE");
        break;
        
        case 7:
            Test_StartBalanceCommand(7,BALANCE_COMMAND_LEVEL_ONLY,"FOUR_AXIS LEVEL");
        break;
        
        case 8:
            Test_StartBalanceCommand(8,BALANCE_COMMAND_FULL_LEVEL,"FULL LEVEL");
        break;
        
        default:
        Test_PrintSummary();
        break;
    }
}


///////////////////////////////////////////////////////////串口屏按键接口
void ChassisTest_OnShortPress(uint32_t now)
{
    if(test_finished != 0)
    {
        Test_SetMessage(
        "Wizard already complete."
        "Hold b_bottom for 1 second to reset.");
        return;
    }
    //测试运行过程中短按，立即中止当前动作
    if(test_run != TEST_RUN_IDLE)
    {
        ChassisTest_ForceStop();
        Test_SetResult(test_step,TEST_RESULT_ABORTED);
        Test_SetMessage(
        "Step %u ABORTED"
        "Press b_bottom again to retry.",
        (unsigned)test_step);
        return;
    }
    //当前步骤没有通过，不进入下一项，直接重试当前步骤
    if((test_step != 0) && (test_result[test_step] != TEST_RESULT_PASS))
    {
        Test_SetMessage(
        "Retry step %u.",
        (unsigned)test_step);
        
        Test_StartStep(now);
        return;
    }
    //当前步骤通过或尚未开始，进入下一步骤
    test_step++;
    Test_StartStep(now);
}

void ChassisTest_OnLongPress(void)
{
    int8_t step;
    ChassisTest_ForceStop();
    
    for(step = 0;step < TEST_RESULT_COUNT;step++)
    {
        test_result[step] = TEST_RESULT_NONE;
    }
    test_step = 0;
    test_finished = 0;
    
    Test_ResetAxisMeasurements();
    Test_SetMessage(
    "EMERGENCY STOP:motors zero,wizard resert."
    "Release b_botom,then short press to start step 1.");
}


///////////////////////////////////////////////////////////初始化与周期任务
void ChassisTest_Init(void)
{
    uint8_t step;
    ChassisTest_ForceStop();
    
    test_step = 0;
    test_finished = 0;
    test_run = TEST_RUN_IDLE;
    
    test_start_tick = 0;
    test_phase_tick = 0;
    test_print_tick = 0;
    
    test_axis = 0;
    test_axis_phase = AXIS_PHASE_START_POSITIVE;
    
    test_imu_valid_count = 0;
    test_imu_sample_count = 0;
    
    for(step = 0;step < TEST_RESULT_COUNT;step++)
    {
        test_result[step] = TEST_RESULT_NONE;
    }
    
    memset(test_limit_high,0,sizeof(test_limit_high));
    memset(test_limit_low,0,sizeof(test_limit_low));
    
    Test_ResetAxisMeasurements();
    
    test_message[0] = '\0';
    test_message_revision = 0;
    
    Test_SetMessage(
    "TEST WIZARD READY:short press b_bottom=next/retry;"
    "hold 1 scond=emergency top/reset."
    );
}

void ChassisTest_Update(uint32_t now,const IMU_Attitude_t *imu,uint8_t imu_valid)
{
    switch (test_run)
    {
        case TEST_RUN_DIRECTION:
            Test_UpdateDirection(now);
            break;

        case TEST_RUN_LIMIT:
            Test_UpdateLimit(now);
            break;

        case TEST_RUN_IMU:
            Test_UpdateImu(
                now,
                imu,
                imu_valid);
            break;

        case TEST_RUN_ENCODER:
            Test_UpdateEncoder(now);
            break;

        case TEST_RUN_BALANCE_COMMAND:
            Test_UpdateBalanceCommand(now);
            break;

        case TEST_RUN_IDLE:
        default:
            break;
    }
}

/* ======================== 状态查询接口 ======================== */

uint8_t ChassisTest_IsFinished(void)
{
    return test_finished;
}

uint8_t ChassisTest_AllPassed(void)
{
    uint8_t step;

    if (test_finished == 0)
    {
        return 0;
    }
    for (step = 1; step <= 8; step++)
    {
        if (test_result[step] !=TEST_RESULT_PASS)
        {
            return 0;
        }
    }
    return 1;
}

uint8_t ChassisTest_GetStep(void)
{
    return test_step;
}

uint8_t ChassisTest_IsRunning(void)
{
    return (test_run != TEST_RUN_IDLE) ? 1 : 0;
}

const char *ChassisTest_GetMessage(void)
{
    return test_message;
}

uint32_t ChassisTest_GetMessageRevision(void)
{
    return test_message_revision;
}

