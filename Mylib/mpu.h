#include "stm32f4xx_hal.h"
#define   RTD   57.2957
void MPU6050_init(void); //Initialize the MPU 
void MPU6050_Read_Accel (float *Ax, float *Ay, float *Az); //Read MPU Accelerator 
void MPU6050_Read_Gyro (float *Gx, float *Gy, float *Gz); //Read MPU Gyroscope
void filter(float *Ax, float *Ay, float *Az,float *Gx, float *Gy, float *Gz,float* pitch , float* roll,float* yaw);