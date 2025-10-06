// mpu.h
#ifndef MPU_H
#define MPU_H
#include "stm32f4xx_hal.h"   // ho?c "stm32f1xx_hal.h" dúng dòng b?n dùng

void  MPU6050_init(void);
void  MPU6050_StartYaw_IT(void);          // b?t d?u d?c chu?i b?ng interrupt
void  MPU6050_GyroZ_BiasCalib(uint16_t n_samples);  // calib bias Gz
float MPU6050_GetYawDeg(void);            // l?y yaw (d?)

#endif
