#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_pwm.h"

/**
 * @brief 
 */
typedef struct {
   BSP_PWM_Channel_t pwm_ch;  //PWM通道号
   float min_duty;             //最小占空比
   float max_duty;             //最大占空比
} SERVO_t;

/**
 * @brief  初始化舵机
 * @param  servo 传入舵机结构体实例指针
 * @retval BSP_OK / BSP_ERR
 */

int8_t SERVO_Init(SERVO_t *servo);

/**
 * @brief  设置舵机角度
 * @param  servo
 * @param  angle 角度值
 * @retval BSP_OK / BSP_ERR
 */
int8_t SERVO_SetAngle(SERVO_t *servo, float angle);

#ifdef __cplusplus
}
#endif

