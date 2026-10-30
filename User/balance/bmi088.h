#ifndef __BMI088_H__
#define __BMI088_H__

#include "main.h"
#include <stdint.h>

typedef struct
{
    //加速度
    float accel_x;
    float accel_y;
    float accel_z;
    //陀螺仪
    float gryo_x;   
    float gryo_y;
    float gryo_z;
    //零偏补偿
    float gryo_offset_x;
    float gryo_offset_y;
    float gryo_offset_z;
    
    uint32_t last_update_tick;
    
    uint8_t initialized; //初始化标志
    uint8_t data_valid; //数据有效标志
    
    uint8_t calibration_valid;
    uint8_t calibrating;
    uint8_t accel_chip_id;
    uint8_t gryo_chip_id;
    uint32_t update_count;
    uint32_t spi_error_count;
    
}BMI088_Data_t;

uint8_t BMI088_Init(void);
//实时更新数据
uint8_t BMI088_Update(void);
//测量零偏
uint8_t BMI088_CalibrateGryo(void);
const BMI088_Data_t *BMI088_GetData(void);

#endif

