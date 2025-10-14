#ifndef __MPU6050_H__
#define __MPU6050_H__

/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>
//#include "i2c.h"
#include "stm32f4xx_hal.h"
// MPU6050 structure

#define RAD_TO_DEG 57.295779513082320876798154814105
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;
    double Gz_bias;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
    double Yaw;
} MPU6050_t;

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

void MPU6050_Init(void);

void MPU6050_Read_Accel(MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(MPU6050_t *DataStruct);

void MPU6050_Read_Temp(MPU6050_t *DataStruct);

void MPU6050_Read_All(MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
void MPU6050_CalibGz(MPU6050_t *DataStruct, uint16_t samples);
void I2C_HardReset(I2C_HandleTypeDef *hi2c);
#endif
