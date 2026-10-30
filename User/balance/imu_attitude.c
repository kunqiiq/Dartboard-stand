/*
    实现了一个基于Mahony思想的IMU姿态解算：
    BMI088加速度计提供“重力方向”，陀螺仪提供“短时间旋转速度”
    用加速度计修正陀螺仪积分漂移，用四元数保存空间姿态，
    最终输出俯仰角pitch，横滚角roll以及对应的角速度
*/
#include "imu_attitude.h"
#include "main.h"
#include "bmi088.h"
#include <math.h>

#define IMU_MAHONY_KP            2.0f       //加速度反馈的比例增益，根据实际进行修改
#define IMU_GRAVITY              9.80665f
#define IMU_ACCEL_TOLERANCE      3.0f
#define IMU_ACCEL_NORM_MIN       1.0e-6f
#define IMU_QUATERNION_NORM_MIN  1.0e-12f
#define IMU_DT_MIN               0.0001f
#define IMU_DT_MAX               0.02f

//映射：0=X,1=Y,2=Z  
#define IMU_BODY_X_AXIS         0
#define IMU_BODY_Y_AXIS         1
#define IMU_BODY_Z_AXIS         2
#define IMU_BODY_X_SIGN         1.0f
#define IMU_BODY_Y_SIGN         1.0f
#define IMU_BODY_Z_SIGN         1.0f

//正向俯仰-前部更高，正向横滚-左侧更高
#define IMU_PITCH_OUTPUT_SIGN   -1.0f
#define IMU_ROLL_OUTPUT_SIGN    1.0f

static IMU_Attitude_t attitude;

static float IMU_SelectAxis(float x,float y,float z,uint8_t axis)
{
    if(axis == 0)
        return x;
    if(axis == 1)
        return y;
    return z;
}

static void IMU_ResetQuaternion(void)
{
    attitude.q0 = 1.0f;
    attitude.q1 = 0.0f;
    attitude.q2 = 0.0f;
    attitude.q3 = 0.0f;
}

void IMU_Attitude_Reset(void)
{
    attitude.pitch = 0.0f;
    attitude.roll = 0.0f;
    attitude.pitch_rate = 0.0f;
    attitude.roll_rate = 0.0f;
    attitude.accel_norm = 0.0f;
    attitude.data_valid = 0.0f;
    attitude.accel_feedback_used = 0;
    IMU_ResetQuaternion();
}

//平方根倒数：用于向量归一化
static float InvSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

void IMU_Attitude_Init(void)
{
    attitude.initialized = 1;
    attitude.update_count = 0;
    attitude.reject_count = 0;
    attitude.last_update_tick = HAL_GetTick();
    IMU_Attitude_Reset();
}


//dt：两次姿态更新之间的时间：s
uint8_t IMU_Attitude_Update(float dt)
{
    const BMI088_Data_t *imu;
    float sensor_accel[3];
    float sensor_gyro[3];
    
    float ax,ay,az;     //原始加速度
    float gx,gy,gz;     //原始角速度
    float measured_gx,measured_gy;
    
    float accel_norm;                       //加速度模长
    float reciprocal_norm;                  //加速度模长的倒数
    float gravity_x,gravity_y,gravity_z;    //当前四元数估计出来的重力方向
    float error_x,error_y,error_z;          //重力计算误差
    float q0,q1,q2,q3;                      //当前四元数
    float q_dot0,q_dot1,q_dot2,q_dot3;      //四元数对时间的导数
    float sin_pitch;                        //计算pitch的中间量
    
    if((attitude.initialized == 0) ||
       (!isfinite(dt)) || (dt <= 0.0f) || (dt > IMU_DT_MAX))
    {
        attitude.data_valid = 0;
        attitude.accel_feedback_used = 0;
        attitude.reject_count++;
        return 0;
    }
    
    imu = BMI088_GetData();
    
    if ((imu == NULL) || (imu->initialized == 0) ||
        (imu->data_valid == 0) || (imu->calibration_valid == 0))
    {
        attitude.data_valid = 0;
        attitude.accel_feedback_used = 0U;
        attitude.reject_count++;
        return 0U;
    }

    sensor_accel[0] = imu->accel_x;
    sensor_accel[1] = imu->accel_y;
    sensor_accel[2] = imu->accel_z;
    sensor_gyro[0] = imu->gryo_x;
    sensor_gyro[1] = imu->gryo_y;
    sensor_gyro[2] = imu->gryo_z;
    
    ax = IMU_BODY_X_SIGN * IMU_SelectAxis(sensor_accel[0], sensor_accel[1],sensor_accel[2], IMU_BODY_X_AXIS);
    ay = IMU_BODY_Y_SIGN * IMU_SelectAxis(sensor_accel[0], sensor_accel[1],sensor_accel[2], IMU_BODY_Y_AXIS);
    az = IMU_BODY_Z_SIGN * IMU_SelectAxis(sensor_accel[0], sensor_accel[1],sensor_accel[2], IMU_BODY_Z_AXIS);
    gx = IMU_BODY_X_SIGN * IMU_SelectAxis(sensor_gyro[0], sensor_gyro[1],sensor_gyro[2], IMU_BODY_X_AXIS);
    gy = IMU_BODY_Y_SIGN * IMU_SelectAxis(sensor_gyro[0], sensor_gyro[1],sensor_gyro[2], IMU_BODY_Y_AXIS);
    gz = IMU_BODY_Z_SIGN * IMU_SelectAxis(sensor_gyro[0], sensor_gyro[1],sensor_gyro[2], IMU_BODY_Z_AXIS);
    measured_gx = gx;
    measured_gy = gy;

    if ((!isfinite(ax)) || (!isfinite(ay)) || (!isfinite(az)) ||
        (!isfinite(gx)) || (!isfinite(gy)) || (!isfinite(gz)))
    {
        attitude.data_valid = 0;
        attitude.accel_feedback_used = 0;
        attitude.reject_count++;
        return 0;
    }

    accel_norm = sqrtf(ax*ax + ay*ay + az*az);
    attitude.accel_norm = accel_norm;
    attitude.accel_feedback_used = 0;
    
    q0 = attitude.q0;
    q1 = attitude.q1;
    q2 = attitude.q2;
    q3 = attitude.q3;
    
    //有效的时候进入加速度反馈修正，加速度异常时直接使用陀螺仪积分
    if (isfinite(accel_norm) && (accel_norm > IMU_ACCEL_NORM_MIN) &&
        (fabsf(accel_norm - IMU_GRAVITY) <= IMU_ACCEL_TOLERANCE))
    {
        //归一化后三轴加速夫成为单位方向向量
        reciprocal_norm = 1.0f / accel_norm;
        ax *= reciprocal_norm;
        ay *= reciprocal_norm;
        az *= reciprocal_norm;
        //估计重力方向（四元数预测）
        gravity_x = 2.0f * (q1*q3 - q0*q2);
        gravity_y = 2.0f * (q0*q1 + q2*q3);
        gravity_z = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        //方向误差：两个单位向量的叉乘
        error_x = ay * gravity_z - az * gravity_y;
        error_y = az * gravity_x - ax * gravity_z;
        error_z = ax * gravity_y - ay * gravity_x;
        //修正陀螺仪角速度
        gx += IMU_MAHONY_KP *error_x;
        gy += IMU_MAHONY_KP *error_y;
        gz += IMU_MAHONY_KP *error_z;
        attitude.accel_feedback_used = 1;
    }
            
            //四元数微分：q̇ = 1/2 × q ⊗ ω
            q_dot0 = 0.5f * (-q1 * gx - q2 * gy - q3 *gz);
            q_dot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
            q_dot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
            q_dot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
            //四元数积分：一阶欧拉积分
            q0 += q_dot0 * dt;
            q1 += q_dot1 * dt;
            q2 += q_dot2 * dt;
            q3 += q_dot3 * dt;
            //四元数归一化
            accel_norm = q0*q0 + q1*q1 + q2*q2 + q3*q3;
            if ((!isfinite(accel_norm)) ||
                (accel_norm < IMU_QUATERNION_NORM_MIN))
            {
                IMU_ResetQuaternion();
                attitude.data_valid = 0;
                attitude.accel_feedback_used = 0;
                attitude.reject_count++;
                return 0;
            }
            
            reciprocal_norm = InvSqrt(accel_norm);
            attitude.q0 = q0 * reciprocal_norm;
            attitude.q1 = q1 * reciprocal_norm;
            attitude.q2 = q2 * reciprocal_norm;
            attitude.q3 = q3 * reciprocal_norm;
            
            q0 = attitude.q0;
            q1 = attitude.q1;
            q2 = attitude.q2;
            q3 = attitude.q3;
            //四元数转换为俯仰角
            sin_pitch = 2.0f *(q0 * q2 - q3 *q1);
    
    if(sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if(sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }
            
            // 与参考的机械方向一致
            attitude.pitch = IMU_PITCH_OUTPUT_SIGN * asinf(sin_pitch);
                //计算横滚角
            attitude.roll = IMU_ROLL_OUTPUT_SIGN * 
                               atan2f (2.0f* (q0 * q1 + q2 * q3),
                                1.0f - 2.0f * (q1 *q1 + q2 * q2));
            
            attitude.pitch_rate = IMU_PITCH_OUTPUT_SIGN * measured_gy;
            attitude.roll_rate = IMU_ROLL_OUTPUT_SIGN * measured_gx;
    
            if ((!isfinite(attitude.pitch)) || (!isfinite(attitude.roll)) ||
            (!isfinite(attitude.pitch_rate)) || (!isfinite(attitude.roll_rate)))
        {
            attitude.data_valid = 0U;
            attitude.reject_count++;
            return 0U;
        }
    
                //输出俯仰角和横滚角
            attitude.last_update_tick = HAL_GetTick();
            attitude.data_valid = 1;
            attitude.update_count++;
            
            return 1;
}


const IMU_Attitude_t *IMU_Attitude_GetData(void)
{
    return &attitude;
}

