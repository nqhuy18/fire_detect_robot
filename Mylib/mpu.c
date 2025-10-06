// mpu.c
#include "mpu.h"

#define MPU6050_ADDR        0xD0        // b?n dang dùng 8-bit addr này
#define ACCEL_XOUT_H_REG    0x3B
#define GYRO_CONFIG_REG     0x1B
#define ACCEL_CONFIG_REG    0x1C
#define SMPLRT_DIV_REG      0x19
#define PWR_MGMT_1_REG      0x6B
#define WHO_AM_I_REG        0x75

extern I2C_HandleTypeDef hi2c1;

// --- bi?n dùng cho IT streaming ---
static volatile uint8_t imu_buf[14];
static volatile float   yaw_deg = 0.0f;      // yaw tích luy
static volatile float   gyro_bias_z = 0.0f;  // bias Gz (deg/s)
static uint32_t         last_tick_ms = 0;

// scale theo c?u hình FS_SEL=0 (±250 dps => 131 LSB/dps)
#define GYRO_SENS_131  (131.0f)

// Init g?c c?a b?n (d?c WHO_AM_I, config reg...) gi? nguyên:
void MPU6050_init(void)
{
    uint8_t check, data;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &check, 1, 1000);
    if (check == 104)
    {
        data = 0x00; 
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

        data = 0x07; 
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

        data = 0x00; 
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

        data = 0x00; 
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    }
}

void MPU6050_GyroZ_BiasCalib(uint16_t n_samples)
{
    int32_t sum = 0;
    uint8_t gbuf[6];

    for (uint16_t i = 0; i < n_samples; i++)
    {
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, I2C_MEMADD_SIZE_8BIT, gbuf, 6, 100);
        int16_t gz_raw = (int16_t)((gbuf[4] << 8) | gbuf[5]);
        sum += gz_raw;
        HAL_Delay(2); 
    }

    float gz_avg_raw = (float)sum / (float)n_samples;
    gyro_bias_z = gz_avg_raw / GYRO_SENS_131; 
}


void MPU6050_StartYaw_IT(void)
{
    last_tick_ms = HAL_GetTick();
    HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_ADDR,
                        ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT,
                        (uint8_t*)imu_buf, 14);
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1)
    {
        
        int16_t gz_raw = (int16_t)((imu_buf[12] << 8) | imu_buf[13]);
        float   gz_dps = (gz_raw / GYRO_SENS_131) - gyro_bias_z;

        uint32_t now = HAL_GetTick();
        float dt = (now - last_tick_ms) * 0.001f; 
        last_tick_ms = now;

        yaw_deg += gz_dps * dt;

       
        if (yaw_deg > 180.0f)      yaw_deg -= 360.0f;
        else if (yaw_deg < -180.0f) yaw_deg += 360.0f;

      
        HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_ADDR,
                            ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT,
                            (uint8_t*)imu_buf, 14);
    }
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1)
    {
        HAL_I2C_DeInit(&hi2c1);
        HAL_I2C_Init(&hi2c1);
        
        last_tick_ms = HAL_GetTick();
        HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_ADDR,
                            ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT,
                            (uint8_t*)imu_buf, 14);
    }
}

float MPU6050_GetYawDeg(void)
{
    return yaw_deg;
}
