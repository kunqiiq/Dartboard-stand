#include "balance_motor.h"

#include "DJI_motor.h"
#include "can.h"

static RM_Motor_t balance_motors[BALANCE_MOTOR_COUNT];
static BalanceMotor_Feedback_t balance_feedback[BALANCE_MOTOR_COUNT];
static uint8_t balance_motors_initialized;

uint8_t BalanceMotor_Init(void)
{
    uint8_t i;

    if (balance_motors_initialized != 0U)
    {
        return 1U;
    }

    for (i = 0U; i < BALANCE_MOTOR_COUNT; i++)
    {
        balance_motors[i].param.hcan = &hcan2;
        balance_motors[i].param.id = (uint16_t)(0x201U + i);
        balance_motors[i].param.type = RM_MOTOR_M3508;

        if (RM_Motor_Register(&balance_motors[i]) != RM_MOTOR_OK)
        {
            (void)BalanceMotor_Stop();
            return 0U;
        }
    }

    balance_motors_initialized = 1U;
    return BalanceMotor_Stop();
}

void BalanceMotor_UpdateTimeout(void)
{
    RM_Motor_CheckTimeout();
}

const BalanceMotor_Feedback_t *BalanceMotor_GetFeedback(uint8_t index)
{
    RM_MotorSnapshot_t snapshot;

    if ((index >= BALANCE_MOTOR_COUNT) || (balance_motors_initialized == 0U))
    {
        return NULL;
    }

    if (!RM_Motor_GetSnapshot(&balance_motors[index], &snapshot))
    {
        return NULL;
    }

    balance_feedback[index].online = snapshot.online ? 1U : 0U;
    balance_feedback[index].angle = snapshot.feedback.raw_angle;
    balance_feedback[index].speed_rpm = snapshot.feedback.raw_speed;
    balance_feedback[index].current = snapshot.feedback.raw_current;
    balance_feedback[index].temperature = snapshot.feedback.temperature;
    balance_feedback[index].last_update_tick = snapshot.last_update_tick;

    return &balance_feedback[index];
}

uint8_t BalanceMotor_IsOnline(uint8_t index)
{
    if ((index >= BALANCE_MOTOR_COUNT) || (balance_motors_initialized == 0U))
    {
        return 0U;
    }

    return RM_Motor_IsOnline(&balance_motors[index]) ? 1U : 0U;
}

uint8_t BalanceMotor_SetCurrents(const int16_t currents[BALANCE_MOTOR_COUNT])
{
    uint8_t i;

    if ((currents == NULL) || (balance_motors_initialized == 0U))
    {
        return 0U;
    }

    for (i = 0U; i < BALANCE_MOTOR_COUNT; i++)
    {
        if (RM_Motor_SetCurrent(&balance_motors[i], currents[i]) != RM_MOTOR_OK)
        {
            return 0U;
        }
    }

    return (RM_Motor_Flush(&hcan2) == RM_MOTOR_OK) ? 1U : 0U;
}

uint8_t BalanceMotor_Stop(void)
{
    int16_t currents[BALANCE_MOTOR_COUNT] = {0, 0, 0, 0};

    if (balance_motors_initialized == 0U)
    {
        return 0U;
    }

    return BalanceMotor_SetCurrents(currents);
}
