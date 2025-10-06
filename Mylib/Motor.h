
	#ifndef MOTOR_H_
	#define MOTOR_H_



	#include "stdbool.h"
	#include "pid.h"

#define MOTOR_RPS_MAX    (330.0f/60.0f) /* ~5.5 rps n?u motor ~330rpm */


#define ENCODER_PPR    1400



#define WHEEL_RADIUS_M   0.034f   



#define TRACK_WIDTH_M    0.30f    




	typedef enum
	{
		LEFT,
		RIGHT
	}Motor_id;


	typedef enum {
    DIR_CW = 0,         
    DIR_CCW
} MotorDir;

	typedef enum {
    ST_IDLE = 0,
    ST_MOVE,
    ST_TURN_LEFT,
    ST_TURN_RIGHT,
    ST_TURN_BACK,
    ST_COOL_DOWN
} MotionState;

	typedef struct {
    MotionState state;

    bool   active; 
    float  v_max;       /* [m/s]    t?c d? t?i da */
    float  v_min;       /* [m/s]    t?c d? t?i thi?u */
    float  v_cmd;       /* [m/s]    l?nh v?n t?c t?c th?i (linear) */
} Motion;

	typedef struct
	{
			Motor_id id;

			//Direct
			GPIO_TypeDef *IN1_Port;
			uint16_t IN1_Pin;
			GPIO_TypeDef *IN2_Port;
			uint16_t IN2_Pin;

			// Encoder
			TIM_HandleTypeDef *htim_encoder;
			int16_t encoder_count;
			int16_t prev_count;
			int16_t delta_count;
			int16_t progress_count;

			// PID
			float kp;
			float ki;
			float kd;

			// PWM output
			TIM_HandleTypeDef *htim_pwm;
			uint32_t Channel;

			//Velocity Display
			double cur_speed;	// input pwwm

			// Velocity Aim
			double target_speed; // setpoint pwwm

			// Adjust value from PID
			double Pid_output;


	} Motor;


	extern double encoder_progress;
	extern double encoder_output;
	extern double encoder_target;

	// Init Motor
	void Motor_Init(Motor *_motor,
							Motor_id id,
									GPIO_TypeDef *_IN1_Port, uint16_t _IN1_Pin,
									GPIO_TypeDef *_IN2_Port, uint16_t _IN2_Pin,
									TIM_HandleTypeDef *htim_pwm, uint32_t Channel,
									TIM_HandleTypeDef *htim_encoder,
					float kp, float ki, float kd);
	//Measurement
	void Motor_UpdateSpeed(Motor *_motor, float new);
	void Motor_GetSpeed(Motor *motor);

	//Control

	void Motor_SetDir(Motor *motor);
	void PWM_limit(Motor *motor);
	void Motor_SetPwm(Motor *motor);
	void Motor_stop(Motor *motor);
	void Motor_SetTarget(Motor*motor, double target);

	//----------------------Advance Control


	void Move_forward(Motor *_motorL, Motor *_motorR);
	void Move_backward(Motor *_motorL, Motor *_motorR);
	void Move_Left(Motor *_motorL, Motor *_motorR);
	void Move_Right(Motor *_motorL, Motor *_motorR);
	void Move_Left_enhanced(Motor *_motorL, Motor*_motorR, float accelerate);
  void Drive_VW(Motor *_motorL, Motor *_motorR, float v_mps, float w_radps);








	#endif 
