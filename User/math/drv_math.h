/**
 * @file drv_math.h
 * @author yssickjgd 1345578933@qq.com
 * @brief 一些数学
 * @version 0.1
 * @date 2023-05-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef DRV_MATH_H
#define DRV_MATH_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <math.h>
#include <float.h>

/* Exported macros -----------------------------------------------------------*/

//圆周率PI
#define PI (3.14159265f)

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

//大小端转换
void Math_Endian_Reverse_16(void *Address);
void Math_Endian_Copy_Reverse_16(const void *Source, void *Destination);
void Math_Endian_Reverse_32(void *Address);
void Math_Endian_Copy_Reverse_32(const void *Source, void *Destination);

//求和
uint8_t Math_Sum_8(uint8_t *Address, uint32_t Length);
uint16_t Math_Sum_16(uint16_t *Address, uint32_t Length);
uint32_t Math_Sum_32(uint32_t *Address, uint32_t Length);

//滤波器相关函数
float Math_Sinc(float x);

/**
 * @brief 限幅函数
 *(PID时积分限幅/输出限幅)
 * @tparam Type
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 */
void Math_Constrain_Float(float *x, float min, float max);
/**
 * @brief 求绝对值
 *（在PID计算中根据   误差的绝对值大小   进行分离，超过某个值的时候，就分离掉，不参与计算）
 * @tparam Type
 * @param x 传入数据
 * @return Type x的绝对值
 */
float Math_Abs_Float(float x);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
