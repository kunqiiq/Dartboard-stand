#ifndef __IMU_ATTITUDE_H__
#define __IMU_ATTITUDE_H__

#include "main.h"

typedef struct
{
    float pitch;
    float roll;
    //角速度
    float pitch_rate;
    float roll_rate;
    //姿态四元数
    float q0;   //实部
    float q1;   //x轴旋分量
    float q2;   //y轴旋转分量
    float q3;   //z轴旋转分量
    
    uint32_t last_update_tick;
    uint32_t update_count;
    uint32_t reject_count;
    float accel_norm;
    
    uint8_t initialized;
    uint8_t data_valid;
    uint8_t accel_feedback_used;

}IMU_Attitude_t;


void IMU_Attitude_Init(void);
uint8_t IMU_Attitude_Update(float dt);

const IMU_Attitude_t *IMU_Attitude_GetData(void);
void IMU_Attitude_Reset(void);


#endif


