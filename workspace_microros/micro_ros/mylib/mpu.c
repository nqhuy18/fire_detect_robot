
#include "math.h"
#include "mpu.h"
#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
float pitchA,rollA,pitchG	,rollG;
//float Ax, Ay, Az, Gx, Gy, Gz;
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

float pitch;
float roll ;
float yaw ;
float AX,AY,AZ,GX,GY,GZ;

int16_t ax = 0 , ay = 0 , az = 0 , gx =0 , gy = 0 , gz = 0;
void MPU6050_init(void)
{
	uint8_t check,data;
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1 , 1000);
	if (check == 104)
	{
		//Power management register write all 0's to wake up sensor
		data = 0;
		HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
		//Set data rate of 1KHz by writing SMPRT_DIV register
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
		//Writing both register with 0 to set full scale range
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);

		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
	}

}

//Function with multiple return using pointer

void MPU6050_Read_Accel (float* Ax, float* Ay, float* Az)
{
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	//Adding 2 BYTES into 16 bit integer
	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	*Ax = Accel_X_RAW*100/16384.0;
	*Ay = Accel_Y_RAW*100/16384.0;
	*Az = Accel_Z_RAW*100/16384.0;
}

void MPU6050_Read_Gyro(float* Gx, float* Gy, float* Gz)
{
    uint8_t Rec_Data[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    // Correctly assign raw data values for each axis
    Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    *Gx = Gyro_X_RAW / 131.0;
    *Gy = Gyro_Y_RAW / 131.0;
    *Gz = Gyro_Z_RAW / 131.0;
}

void filter(float* Ax, float* Ay, float* Az , float* Gx, float* Gy, float* Gz,float* pitch,float* roll,float* yaw, float dt){
	 pitchG = *pitch +*Gx*dt;
	 rollG = *roll + *Gy*dt;

	 pitchA = atan2(*Ay,sqrt(*Ax**Ax+*Az**Az))*RTD;
	 rollA = atan2(*Ax,sqrt(*Ay**Ay+*Az**Az))*RTD;

	*pitch = 0.98*pitchG + 0.02*pitchA;
	*roll = 0.98*rollG + 0.02*rollA;
	*yaw = *yaw + *Gz * dt;

}


void MPU6050ReadG(void){
  uint8_t dataG[6];

  HAL_I2C_Mem_Read(&hi2c1 ,MPU6050_ADDR,0x43 ,1 , dataG,6,1000);
  gx = (int16_t)(dataG[0] << 8 | dataG[1]);
  gy = (int16_t)(dataG[2] << 8 | dataG[3]);
  gz = (int16_t)(dataG[4] << 8 | dataG[5]);
  GX = (float)gx /131.0;
  GY = (float)gy /131.0;
  GZ = (float)gz /131.0;
}
void MPU6050ReadA(void){
  uint8_t dataA[6];
  HAL_I2C_Mem_Read(&hi2c1 ,MPU6050_ADDR,0x3B ,1 , dataA,6,1000);
  ax = (int16_t)(dataA[0] << 8 | dataA[1]);
  ay = (int16_t)(dataA[2] << 8 | dataA[3]);
  az = (int16_t)(dataA[4] << 8 | dataA[5]);
  AX = (float)ax /16384.0;
  AY = (float)ay /16384.0;
  AZ = (float)az /16384.0;
}
void filter1(float AX,float AY,float AZ,float GX,float GY,float GZ){
  float pitchG = pitch +GX*(1000/1000000.0f);
  float rollG = roll +GY*(1000/1000000.0f);
  float yawG   = yaw  + GZ *(1000/1000000.0f);

  float pitchA = atan2(AY,sqrt(AX*AX+AZ*AZ))*RTD;
  float rollA = atan2(AX,sqrt(AY*AY+AZ*AZ))*RTD;

  pitch = 0.98*pitchG + 0.02*pitchA;
  roll = 0.98*rollG + 0.02*rollA;
  yaw = yawG;
}








