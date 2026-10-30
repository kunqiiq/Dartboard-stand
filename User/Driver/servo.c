#include "servo.h"
#include "bsp_pwm.h"


#define SERVO_MIN_DUTY  0.025f   
#define SERVO_MAX_DUTY  0.125f   


/**
 * @brief  初始化舵机
 * @param  servo 传入舵机结构体实例指针
 * @retval BSP_OK / BSP_ERR
 */

int8_t SERVO_Init(SERVO_t *servo) {
    if (servo == NULL) return HAL_ERROR;
    return BSP_PWM_Start(servo->pwm_ch);
}

int8_t SERVO_SetAngle(SERVO_t *servo, float angle) {
    if (servo == NULL) return HAL_ERROR;
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    float duty = servo->min_duty + (angle / 180.0f) * (servo->max_duty - servo->min_duty);

    return BSP_PWM_SetComp(servo->pwm_ch, duty);
}

