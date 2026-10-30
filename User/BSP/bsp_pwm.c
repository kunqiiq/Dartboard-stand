/* Includes ----------------------------------------------------------------- */
#include "bsp_pwm.h"


typedef struct {
  TIM_HandleTypeDef *tim; //指向定时器指针 
  uint16_t channel; 	//指向定时器PWM的输出通道
} BSP_PWM_Config_t;


/* 内部保存PWM通道数的内存 -------------------------------------------------------- */
static const BSP_PWM_Config_t PWM_Map[BSP_PWM_NUM] = {
	/* 需要在此为每个通道分配定时器句柄和对应通道 */
	  [BSP_PWM_TIM1_CH1] = {&htim1, TIM_CHANNEL_1},
  [BSP_PWM_TIM1_CH2] = {&htim1, TIM_CHANNEL_2},
  [BSP_PWM_TIM1_CH3] = {&htim1, TIM_CHANNEL_3},
  [BSP_PWM_TIM1_CH4] = {&htim1, TIM_CHANNEL_4},
};


/* 启动指定通道的PWM输出 */
HAL_StatusTypeDef BSP_PWM_Start(BSP_PWM_Channel_t ch) {
  if (ch >= BSP_PWM_NUM) return HAL_ERROR;
  
	//Hal库的使能函数 主要是这个 其他的部分用作是检查
  return HAL_TIM_PWM_Start(PWM_Map[ch].tim, PWM_Map[ch].channel);
}

/* 设定指定通道的占空比
duty_cycle 输出范围是0~1.0f
*/
HAL_StatusTypeDef BSP_PWM_SetComp(BSP_PWM_Channel_t ch, float duty_cycle) {
  if (ch >= BSP_PWM_NUM) return HAL_ERROR;
  
  if (duty_cycle > 1.0f) {
    duty_cycle = 1.0f;
  }
  if (duty_cycle < 0.0f) {
    duty_cycle = 0.0f;
  }
  // 获取ARR值（周期值）
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(PWM_Map[ch].tim);
  
  // 计算比较值：CCR = duty_cycle * (ARR + 1)
  uint32_t ccr = (uint32_t)(duty_cycle * (arr + 1));
  
  __HAL_TIM_SET_COMPARE(PWM_Map[ch].tim, PWM_Map[ch].channel, ccr);

  return HAL_OK;
}
