#include "Motor.h"
#include <math.h>

	double encoder_progress = 0;
	double encoder_output = 0;
	double encoder_target = 0;
	uint16_t count = 0;
	uint32_t prevtime = 0;
	float vL, vR;
#define OFFSET 0.05f
extern double vl_cur_mps, vr_cur_mps;
	// Init Motor
	void Motor_Init(Motor *_motor,
						Motor_id id,
						GPIO_TypeDef *_IN1_Port, uint16_t _IN1_Pin,
						GPIO_TypeDef *_IN2_Port, uint16_t _IN2_Pin,
						TIM_HandleTypeDef *htim_pwm, uint32_t Channel,
						TIM_HandleTypeDef *htim_encoder,
						float kp, float ki, float kd)
		{
			// G?n ID
			_motor->id = id;

			// Luu th?ng tin ch?n di?u khi?n chi?u
			_motor->IN1_Port = _IN1_Port;
			_motor->IN1_Pin  = _IN1_Pin;
			_motor->IN2_Port = _IN2_Port;
			_motor->IN2_Pin  = _IN2_Pin;

			// PWM
			_motor->htim_pwm = htim_pwm;
			_motor->Channel  = Channel;


			// Encoder
			_motor->htim_encoder = htim_encoder;
			_motor->encoder_count = 0;
			_motor->prev_count    = 0;
			_motor->delta_count   = 0;
			_motor->progress_count = 0;

			// PID
			_motor->kp = kp;
			_motor->ki = ki;
			_motor->kd = kd;

			// Speed
			_motor->cur_speed    = 0.0;
			_motor->target_speed = 0.0;
			_motor->Pid_output = 0.0;

			// Start PWM
			HAL_TIM_PWM_Start(_motor->htim_pwm, _motor->Channel);

			// Kh?i d?ng Encoder
			HAL_TIM_Encoder_Start(_motor->htim_encoder, TIM_CHANNEL_ALL);
		}
		
void Motor_UpdateSpeed(Motor *_motor, float new);
void Motor_GetSpeed(Motor *_motor)
	{
		_motor->encoder_count = __HAL_TIM_GET_COUNTER(_motor->htim_encoder);
		_motor->delta_count = _motor->encoder_count - _motor->prev_count;
    int sign = (_motor->id == LEFT) ? 1 : -1;
		_motor->cur_speed = (sign)*( (double)(_motor->delta_count)/ ENCODER_PPR) * (60.0 / (10 / 1000.0));
		_motor->prev_count = _motor->encoder_count;

	}
	
	
void Motor_SetPwm(Motor *motor)
	{
		if(motor->Pid_output > 999)
		{
			motor->Pid_output = 999;
		}
		else if(motor->Pid_output < -999)
		{
			motor->Pid_output = -999;
		}
	if(motor->id == RIGHT)
		{
		if (motor->target_speed == 0 && fabs(vr_cur_mps) < OFFSET)
		{
			motor->Pid_output = 0;
		}
			if(motor->Pid_output > 0)
			{
				HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
			}
			else if(motor->Pid_output < 0)
			{
				HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
			}
			motor->htim_pwm->Instance->CCR2 =(uint32_t)((fabs)(motor->Pid_output) );
			}
		else if (motor->id == LEFT)
		{
			if (motor->target_speed == 0  && fabs(vl_cur_mps) < OFFSET)
			{
				motor->Pid_output = 0;
			}
			if(motor->Pid_output > 0)
			{
				HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin,GPIO_PIN_SET);
			}
			else if(motor->Pid_output < 0)
			{
				HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
			}
			motor->htim_pwm->Instance->CCR1 =(uint32_t)((fabs)(motor->Pid_output) );

		}


	}
void Motor_SetTarget(Motor*motor, double target)
	{
		motor->target_speed = target;
	}

void Drive_VW(Motor *mL, Motor *mR, float v_mps, float w_radps)
{
    const float two_pi_R   = 2.0f * 3.1415926f * WHEEL_RADIUS_M;

     vL = (2 * v_mps - w_radps * TRACK_WIDTH_M) / (2) ;   // [m/s]
     vR = (2 * v_mps + w_radps * TRACK_WIDTH_M) / (2);   // [m/s]

    double rpsL = (double)(vL / two_pi_R); // [rps]
    double rpsR = (double)(vR / two_pi_R); // [rps]

    if (rpsL >  MOTOR_RPS_MAX) rpsL =  MOTOR_RPS_MAX;
    if (rpsL < -MOTOR_RPS_MAX) rpsL = -MOTOR_RPS_MAX;
    if (rpsR >  MOTOR_RPS_MAX) rpsR =  MOTOR_RPS_MAX;
    if (rpsR < -MOTOR_RPS_MAX) rpsR = -MOTOR_RPS_MAX;

    Motor_SetTarget(mL, 60*rpsL); // [rpm]
    Motor_SetTarget(mR, 60*rpsR); // [rpm]
}
