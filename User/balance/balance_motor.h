#ifndef BALANCE_MOTOR_H
#define BALANCE_MOTOR_H

#include "main.h"
#include <stdint.h>

#define BALANCE_MOTOR_COUNT 4U

typedef struct
{
    uint8_t online;
    uint16_t angle;
    int16_t speed_rpm;
    int16_t current;
    uint8_t temperature;
    uint32_t last_update_tick;
} BalanceMotor_Feedback_t;

/* Register four M3508 motors (0x201~0x204) on CAN2. */
uint8_t BalanceMotor_Init(void);

/* Refresh online flags maintained by the common RM motor driver. */
void BalanceMotor_UpdateTimeout(void);

const BalanceMotor_Feedback_t *BalanceMotor_GetFeedback(uint8_t index);
uint8_t BalanceMotor_IsOnline(uint8_t index);

/* Write one complete CAN2 0x200 current frame. Returns 1 on success. */
uint8_t BalanceMotor_SetCurrents(const int16_t currents[BALANCE_MOTOR_COUNT]);
uint8_t BalanceMotor_Stop(void);

#endif /* BALANCE_MOTOR_H */
