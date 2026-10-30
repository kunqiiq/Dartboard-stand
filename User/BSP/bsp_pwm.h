/* TIM1 由 CubeMX 配置为 50 Hz 舵机 PWM。 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"
#include <stdint.h>

/* PWM通道 */


typedef enum {
	/*在此处注册PWM通带BSP枚举实例*/
	  BSP_PWM_TIM1_CH1,
  BSP_PWM_TIM1_CH2,
  BSP_PWM_TIM1_CH3,
  BSP_PWM_TIM1_CH4,
	/* 用户 BSP_PWM 开始 */
	/* 用户 BSP_PWM 结束 */
	
  BSP_PWM_NUM,
  BSP_PWM_ERR,
} BSP_PWM_Channel_t;



/*对外暴露函数声明*/
HAL_StatusTypeDef BSP_PWM_Start(BSP_PWM_Channel_t ch);
HAL_StatusTypeDef BSP_PWM_SetComp(BSP_PWM_Channel_t ch, float duty_cycle);

#ifdef __cplusplus
}
#endif

