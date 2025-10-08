
#include "stm32f4xx_hal.h"
#define   RTD   57.2957
void MPU6050_init(void); //Initialize the MPU
void MPU6050_Read_Accel (float *Ax, float *Ay, float *Az); //Read MPU Accelerator
void MPU6050_Read_Gyro (float *Gx, float *Gy, float *Gz); //Read MPU Gyroscope
void filter(float *Ax, float *Ay, float *Az,float *Gx, float *Gy, float *Gz,float* pitch , float* roll,float* yaw, float dt);
void MPU6050ReadG(void);
void MPU6050ReadA(void);
void filter1(float AX,float AY,float AZ,float GX,float GY,float GZ);
extern float AX,AY,AZ,GX,GY,GZ;
extern float pitch;
extern float roll ;
extern float yaw ;
