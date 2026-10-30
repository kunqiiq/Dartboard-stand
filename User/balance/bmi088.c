/* 
*BMI088六轴惯性传感器的对外接口，
保存三轴加速度，角速度，陀螺仪零偏和驱动状态
*/

#include "bmi088.h"
#include "iwdg.h"
#include "balance.h"
#include "spi.h"
#include <math.h>

#define BMI088_ACCEL_CHIP_ID_REG        0X00
#define BMI088_ACCEL_CHIP_ID_VALUE      0X1E
#define BNI088_ACCEL_X_LSB_REG          0X12
#define BMI088_ACCEL_CONF_REG           0X40
#define BMI088_ACCEL_RANGE_REG          0X41
#define BMI088_ACCEL_PWR_CONF_REG       0X7C
#define BMI088_ACCEL_PWR_CTRL_REG       0X7D
#define BMI088_ACCEL_SOFTRESET_REG      0X7E

#define BMI088_GRYO_CHIP_ID_REG         0X00
#define BMI088_GRYO_CHIP_ID_VALUE       0X0F
#define BMI088_GRYO_X_LSB_REG           0X02  
#define BMI088_GRYO_RANGE_REG           0X0F
#define BMI088_GRYO_BANDWIDTH_REG       0X10
#define BMI088_GRYO_LPW1_REG            0X11
#define BMI088_GRYO_SOFERESET_REG       0x14

#define BNI088_ACCEL_SCALE     (9.80665f / 5461.3333f)      //加速度量程换算 ±6g m/s²
#define BMI088_GRYO_SCALE        (0.0010652644f)            //陀螺仪量程换算 ±2000°/s， 正负34.91rad/s，  2000/32768 * π/180

#define BMI088_SPI_TIMEOUT      2             //超时时间
#define BMI088_GRYO_CALIBRATE_COUNT   1000      //校准次数
#define BMI088_GRYO_CALLBRATE_MIN_OK   960
#define BMI088_GRYO_CALLBRATE_STD_MAX  0.03f    //校准时最大角速度标准差（rad/s）
#define BMI088_GRYO_CALLBRATE_MEAN_MAX 0.10f    //拒绝把持续旋转误当成零偏
#define BMI088_ACCEL_CALIBRATE_NORM_ERR_MAX 1.0f
#define BMI088_ACCEL_CALIBRATE_STD_MAX      0.25f

static BMI088_Data_t bmi088_data;

//拉低或者拉高cs片选
static void BMI088_Accel_Select(void)
{
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port,BMI088_ACC_CS_Pin,GPIO_PIN_RESET);
}

static void BMI088_Accel_Deselect(void)
{
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port,BMI088_ACC_CS_Pin,GPIO_PIN_SET);
}

static void BMI088_Gryo_Select(void)
{
    HAL_GPIO_WritePin(BMI088_GRYO_CS_GPIO_Port,BMI088_GRYO_CS_Pin,GPIO_PIN_RESET);
}

static void BMI088_Gryo_Deselect(void)
{
    HAL_GPIO_WritePin(BMI088_GRYO_CS_GPIO_Port,BMI088_GRYO_CS_Pin,GPIO_PIN_SET);
}

//写寄存器
static uint8_t BMI088_Accel_Write(uint8_t reg,uint8_t value)
{
    uint8_t tx[2];
    
    tx[0] = reg & 0x7f;
    tx[1] = value;
    
    BMI088_Accel_Select();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1,tx,2,BMI088_SPI_TIMEOUT);
    BMI088_Accel_Deselect();
    return (status == HAL_OK) ? 1 : 0;
}

//读寄存器
static uint8_t BMI088_Accel_Read(uint8_t reg)
{
    uint8_t command;
    uint8_t dummy;
    uint8_t value;
    
    command = reg | 0x80;
    dummy = 0xff;
    value = 0;
    
    BMI088_Accel_Select();
    HAL_SPI_Transmit(&hspi1,&command,1,BMI088_SPI_TIMEOUT);
    
    //加速度SPI需要读取一个 dummy byte
    HAL_SPI_TransmitReceive(&hspi1,&dummy,&value,1,BMI088_SPI_TIMEOUT);
    
    HAL_SPI_TransmitReceive(&hspi1,&dummy,&value,1,BMI088_SPI_TIMEOUT);
    
    BMI088_Accel_Deselect();
    return value;
}

//连续读多个寄存器
static uint8_t BMI088_Accel_ReadMultiple(uint8_t reg,uint8_t *buffer,uint16_t length)
{
    uint8_t command;
    uint8_t dummy;
    uint8_t ignored;
    
    command = reg | 0x80;
    dummy = 0xff;
    ignored = 0;
    
    BMI088_Accel_Select();
    if (HAL_SPI_Transmit(&hspi1,&command,1,BMI088_SPI_TIMEOUT) != HAL_OK)
    {
        BMI088_Accel_Deselect();
        return 0;
    }
    if (HAL_SPI_TransmitReceive(&hspi1,&dummy,&ignored,1,BMI088_SPI_TIMEOUT) != HAL_OK)
    {
        BMI088_Accel_Deselect();
        return 0;
    }
    
    while(length > 0)
    {
        if (HAL_SPI_TransmitReceive(&hspi1,&dummy,buffer,1,BMI088_SPI_TIMEOUT) != HAL_OK)
        {
            BMI088_Accel_Deselect();
            return 0;
        }
        buffer++;
        length--;
    }
    BMI088_Accel_Deselect();
    return 1;
}

//写寄存器
static uint8_t BMI088_GRYO_Write(uint8_t reg,uint8_t value)
{
    uint8_t tx[2];
    
    tx[0] = reg & 0x7f;
    tx[1] = value;
    
    BMI088_Gryo_Select();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1,tx,2,BMI088_SPI_TIMEOUT);
    BMI088_Gryo_Deselect();
    return (status == HAL_OK) ? 1 : 0;
}

//读寄存器
static uint8_t BMI088_GRYO_Read(uint8_t reg)
{
    uint8_t command;
    uint8_t dummy;
    uint8_t value;
    
    command = reg | 0x80;
    dummy = 0xff;
    value = 0;
    
    BMI088_Gryo_Select();
    HAL_SPI_Transmit(&hspi1,&command,1,BMI088_SPI_TIMEOUT);
    HAL_SPI_TransmitReceive(&hspi1,&dummy,&value,1,BMI088_SPI_TIMEOUT);
    BMI088_Gryo_Deselect();
    
    return value;
}

//连续读多个寄存器
static uint8_t BMI088_Gryo_ReadMultiple(uint8_t reg,uint8_t *buffer,uint8_t length)
{
    uint8_t command;
    uint8_t dummy;
    
    command = reg | 0x80;
    dummy = 0xff;
    
    BMI088_Gryo_Select();
    if (HAL_SPI_Transmit(&hspi1,&command,1,BMI088_SPI_TIMEOUT) != HAL_OK)
    {
        BMI088_Gryo_Deselect();
        return 0;
    }
    
    while(length > 0)
    {
        if (HAL_SPI_TransmitReceive(&hspi1,&dummy,buffer,1,BMI088_SPI_TIMEOUT) != HAL_OK)
        {
            BMI088_Gryo_Deselect();
            return 0;
        }
        buffer++;
        length--;
    }
    
    BMI088_Gryo_Deselect();
    return 1;
}
//加速度初始化
static uint8_t BMI088_InitAccel(void)
{
    uint8_t chip_id;
    
    BMI088_Accel_Deselect();
    HAL_Delay(10);
    //第一次读取用于切换加速度到SPI模式
    (void)BMI088_Accel_Read(BMI088_ACCEL_CHIP_ID_REG);
    chip_id = BMI088_Accel_Read(BMI088_ACCEL_CHIP_ID_REG);
    bmi088_data.accel_chip_id = chip_id;
    
    if(chip_id != BMI088_ACCEL_CHIP_ID_VALUE)
    {
        return 0;
    }
    
    if (BMI088_Accel_Write(BMI088_ACCEL_SOFTRESET_REG,0XB6) == 0U) return 0U;
    HAL_Delay(50);
    (void)BMI088_Accel_Read(BMI088_ACCEL_CHIP_ID_REG);
    
    //开启加速计
    if (BMI088_Accel_Write(BMI088_ACCEL_PWR_CTRL_REG,0x04) == 0U) return 0U;
    //退出suspend
    if (BMI088_Accel_Write(BMI088_ACCEL_PWR_CONF_REG,0x00) == 0U) return 0U;
    
    HAL_Delay(50);
    
    //800hz,正常带宽模式
    if (BMI088_Accel_Write(BMI088_ACCEL_CONF_REG,0xAB) == 0U) return 0U;
    //±6g
    if (BMI088_Accel_Write(BMI088_ACCEL_RANGE_REG,0X01) == 0U) return 0U;
    
    HAL_Delay(10);
    
    return 1;
}

//初始化陀螺仪
static uint8_t BMI088_InitGryo(void)
{
    uint8_t chip_id;
    
    BMI088_Gryo_Deselect();
    HAL_Delay(10);
    chip_id = BMI088_GRYO_Read(BMI088_GRYO_CHIP_ID_REG);
    bmi088_data.gryo_chip_id = chip_id;
    
    if(chip_id != BMI088_GRYO_CHIP_ID_VALUE)
    {
        return 0;
    }
    
    if (BMI088_GRYO_Write(BMI088_GRYO_SOFERESET_REG,0XB6) == 0U) return 0U;
    HAL_Delay(50);
    
    //正常工作模式
    if (BMI088_GRYO_Write(BMI088_GRYO_LPW1_REG,0X00) == 0U) return 0U;
    //±2000 deg/s
    if (BMI088_GRYO_Write(BMI088_GRYO_RANGE_REG,0X00) == 0U) return 0U;
    //1000hz ODR，约116hz带宽
    if (BMI088_GRYO_Write(BMI088_GRYO_BANDWIDTH_REG,0X02) == 0U) return 0U;
    
    HAL_Delay(10);
    
    return 1;
}

//获取 BMI088 加速度和陀螺仪的数据，转换为实际物理单位
uint8_t BMI088_Update(void)
{
    uint8_t accel_buffer[6];
    uint8_t gryo_buffer[6];
    int16_t accel_raw_x;
    int16_t accel_raw_y;
    int16_t accel_raw_z;
    
    int16_t gryo_raw_x;
    int16_t gryo_raw_y;
    int16_t gryo_raw_z;
    
    //初始化之前不能实时更新数据
    if(bmi088_data.initialized == 0)
    {
        bmi088_data.data_valid = 0;
        return 0;
    }
    
    if ((BMI088_Accel_ReadMultiple(BNI088_ACCEL_X_LSB_REG,accel_buffer,6) == 0U) ||
        (BMI088_Gryo_ReadMultiple(BMI088_GRYO_X_LSB_REG,gryo_buffer,6) == 0U))
    {
        bmi088_data.spi_error_count++;
        bmi088_data.data_valid = 0;
        return 0;
    }
    
    accel_raw_x = (int16_t)((uint16_t)accel_buffer[1] << 8) | accel_buffer[0];
    accel_raw_y = (int16_t)((uint16_t)accel_buffer[3] << 8) | accel_buffer[2];
    accel_raw_z = (int16_t)((uint16_t)accel_buffer[5] << 8) | accel_buffer[4];
    
    gryo_raw_x = (int16_t)((uint16_t)gryo_buffer[1] << 8) | gryo_buffer[0];
    gryo_raw_y = (int16_t)((uint16_t)gryo_buffer[3] << 8) | gryo_buffer[2];
    gryo_raw_z = (int16_t)((uint16_t)gryo_buffer[5] << 8) | gryo_buffer[4];
    
    bmi088_data.accel_x = (float)accel_raw_x * BNI088_ACCEL_SCALE;
    bmi088_data.accel_y = (float)accel_raw_y * BNI088_ACCEL_SCALE;
    bmi088_data.accel_z = (float)accel_raw_z * BNI088_ACCEL_SCALE;
    //顺便减去零偏
    bmi088_data.gryo_x = (float)gryo_raw_x * BMI088_GRYO_SCALE - bmi088_data.gryo_offset_x;
    bmi088_data.gryo_y = (float)gryo_raw_y * BMI088_GRYO_SCALE - bmi088_data.gryo_offset_y;
    bmi088_data.gryo_z = (float)gryo_raw_z * BMI088_GRYO_SCALE - bmi088_data.gryo_offset_z;
    //更新时间
    bmi088_data.last_update_tick = HAL_GetTick();
    bmi088_data.data_valid = 1;
    bmi088_data.update_count++;
    
    return 1;
}

//连续读取多次陀螺仪数据，取平均值作为零点偏差
uint8_t BMI088_CalibrateGryo(void)
{
    uint32_t i;
    uint32_t succes_count = 0;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    float square_x = 0.0f;
    float square_y = 0.0f;
    float square_z = 0.0f;
    float accel_norm_sum = 0.0f;
    float accel_norm_square = 0.0f;
    float mean_x;
    float mean_y;
    float mean_z;
    float variance_x;
    float variance_y;
    float variance_z;
    float accel_norm_mean;
    float accel_norm_variance;
    
    bmi088_data.calibrating = 1;
    bmi088_data.calibration_valid = 0;
    bmi088_data.gryo_offset_x = 0.0f;
    bmi088_data.gryo_offset_y = 0.0f;
    bmi088_data.gryo_offset_z = 0.0f;
    
    //校准期间平台必须完全静止
    for(i = 0;i < BMI088_GRYO_CALIBRATE_COUNT;i++)
    {
        if(BMI088_Update() != 0)
        {
            sum_x += bmi088_data.gryo_x;
            sum_y += bmi088_data.gryo_y;
            sum_z += bmi088_data.gryo_z;
            square_x += bmi088_data.gryo_x * bmi088_data.gryo_x;
            square_y += bmi088_data.gryo_y * bmi088_data.gryo_y;
            square_z += bmi088_data.gryo_z * bmi088_data.gryo_z;

            {
                float accel_norm = sqrtf(
                    bmi088_data.accel_x * bmi088_data.accel_x +
                    bmi088_data.accel_y * bmi088_data.accel_y +
                    bmi088_data.accel_z * bmi088_data.accel_z);

                accel_norm_sum += accel_norm;
                accel_norm_square += accel_norm * accel_norm;
            }
            
            succes_count++;
        }
        //IWDG已经在main中限于bmi088初始化，必须喂狗
        (void)HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(1);
    }
    if (succes_count < BMI088_GRYO_CALLBRATE_MIN_OK)
    {
        bmi088_data.data_valid = 0;
        bmi088_data.calibrating = 0;
        return 0;
    }
    
    mean_x = sum_x / (float)succes_count;
    mean_y = sum_y / (float)succes_count;
    mean_z = sum_z / (float)succes_count;
    variance_x = square_x / (float)succes_count - mean_x * mean_x;
    variance_y = square_y / (float)succes_count - mean_y * mean_y;
    variance_z = square_z / (float)succes_count - mean_z * mean_z;
    accel_norm_mean = accel_norm_sum / (float)succes_count;
    accel_norm_variance = accel_norm_square / (float)succes_count -
                          accel_norm_mean * accel_norm_mean;
    if(variance_x < 0.0f)  variance_x = 0.0f;
    if(variance_y < 0.0f)  variance_y = 0.0f;
    if(variance_z < 0.0f)  variance_z = 0.0f;
    if(accel_norm_variance < 0.0f) accel_norm_variance = 0.0f;
    
    if((!isfinite(mean_x)) || (!isfinite(mean_y)) || (!isfinite(mean_z)) ||
        (sqrtf(variance_x) > BMI088_GRYO_CALLBRATE_STD_MAX) ||
        (sqrtf(variance_y) > BMI088_GRYO_CALLBRATE_STD_MAX) ||
        (sqrtf(variance_z) > BMI088_GRYO_CALLBRATE_STD_MAX) ||
        (fabsf(mean_x) > BMI088_GRYO_CALLBRATE_MEAN_MAX) ||
        (fabsf(mean_y) > BMI088_GRYO_CALLBRATE_MEAN_MAX) ||
        (fabsf(mean_z) > BMI088_GRYO_CALLBRATE_MEAN_MAX) ||
        (fabsf(accel_norm_mean - 9.80665f) >
         BMI088_ACCEL_CALIBRATE_NORM_ERR_MAX) ||
        (sqrtf(accel_norm_variance) > BMI088_ACCEL_CALIBRATE_STD_MAX))
    {
        bmi088_data.data_valid = 0;
        bmi088_data.calibrating = 0;
        return 0;
    }
    
    bmi088_data.gryo_offset_x = mean_x;
    bmi088_data.gryo_offset_y = mean_y;
    bmi088_data.gryo_offset_z = mean_z;
    bmi088_data.calibration_valid = 1;
    bmi088_data.calibrating = 0;
    return 1;
}

uint8_t BMI088_Init(void)
{
    bmi088_data.initialized = 0;
    bmi088_data.data_valid = 0;
    
    bmi088_data.calibration_valid = 0;
    bmi088_data.calibrating = 0;
    bmi088_data.accel_chip_id = 0;
    bmi088_data.gryo_chip_id = 0;
    bmi088_data.update_count = 0;
    bmi088_data.spi_error_count = 0;
    
    //拉高片选标志
    BMI088_Accel_Deselect();
    BMI088_Gryo_Deselect();
    
    HAL_Delay(10);
    
    if(BMI088_InitAccel() == 0)
    {
        return 0;
    }
    if(BMI088_InitGryo() == 0)
    {
        return 0;
    }
    
    bmi088_data.initialized = 1;
    //读取数据
    if(BMI088_Update() == 0)
    {
        bmi088_data.initialized = 0;
        return 0;
    }
    //零偏校准
    if (BMI088_CalibrateGryo() == 0)
    {
        bmi088_data.initialized = 0;
        return 0U;
    }

    if (bmi088_data.data_valid == 0)
    {
        bmi088_data.initialized = 0;
        return 0U;
    }
    
    return 1;
}

const BMI088_Data_t *BMI088_GetData(void)
{
    return &bmi088_data;
}

