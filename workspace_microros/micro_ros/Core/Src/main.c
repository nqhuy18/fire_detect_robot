/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <math.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
/* USER CODE END PM */

/* Private variables
 *  ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ADD  0xD0
#define RTD  57.2957
geometry_msgs__msg__Twist msg_cmd_vel;
double v, omega;
double R = 5, L = 20;
double vl, vr;

int16_t ax = 0 , ay = 0 , az = 0 , gx =0 , gy = 0 , gz = 0;
float AX,AY,AZ,GX,GY,GZ;

double pitch = 0,roll = 0, yaw = 0;

void cmd_vel_callback(const void * msgin)
{
  // Cast received message to used type
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

   v = msg->linear.x;
   omega = msg->angular.z;

   vl = (2 * v - omega * L) / (2 * R);
   vr = (2 * v + omega * L) / (2 * R);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void MPU6050Init(void){
  uint8_t check;
  uint8_t mdata;
  HAL_I2C_Mem_Read(&hi2c2,ADD, 0x75 ,1, &check,1,1000);
  if(check == 0x68){
    mdata = 0x00;
    HAL_I2C_Mem_Write(&hi2c2,ADD,0x6B,1,&mdata,1,1000);
    mdata = 0x07;
    HAL_I2C_Mem_Write(&hi2c2,ADD,0x19,1,&mdata,1,1000);
    mdata = 0x00;
    HAL_I2C_Mem_Write(&hi2c2,ADD,0x1B,1,&mdata,1,1000);
    mdata = 0x00;
    HAL_I2C_Mem_Write(&hi2c2,ADD,0x1C,1,&mdata,1,1000);
  }
}


void MPU6050ReadG(void){
  uint8_t dataG[6];

  HAL_I2C_Mem_Read(&hi2c2 ,ADD,0x43 ,1 , dataG,6,1000);
  gx = (int16_t)(dataG[0] << 8 | dataG[1]);
  gy = (int16_t)(dataG[2] << 8 | dataG[3]);
  gz = (int16_t)(dataG[4] << 8 | dataG[5]);
  GX = (float)gx /131.0;
  GY = (float)gy /131.0;
  GZ = (float)gz /131.0;

}
void MPU6050ReadA(void){
  uint8_t dataA[6];
  HAL_I2C_Mem_Read(&hi2c2 ,ADD,0x3B ,1 , dataA,6,1000);
  ax = (int16_t)(dataA[0] << 8 | dataA[1]);
  ay = (int16_t)(dataA[2] << 8 | dataA[3]);
  az = (int16_t)(dataA[4] << 8 | dataA[5]);
  AX = (float)ax /16384.0;
  AY = (float)ay /16384.0;
  AZ = (float)az /16384.0;
}
void filter(float AX,float AY,float AZ,float GX,float GY,float GZ){
  float pitchG = pitch +GX*(1000/1000000.0f);
  float rollG = roll +GY*(1000/1000000.0f);

  float pitchA = atan2(AY,sqrt(AX*AX+AZ*AZ));
  float rollA = atan2(AX,sqrt(AY*AY+AZ*AZ));

  pitch = 0.98*pitchG + 0.02*pitchA ;
  roll = 0.98*rollG + 0.02*rollA;
  yaw = yaw   + GZ *(1000/1000000.0f);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  MPU6050Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  MPU6050ReadA();
//	  MPU6050ReadG();
//	  filter(AX, AY, AZ, GX, GY,GZ);
//	  vTaskDelay(pdMS_TO_TICKS(10));
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);


rcl_publisher_t odom_pub, tf_pub;
rcl_subscription_t subscriber;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__TransformStamped tf;
tf2_msgs__msg__TFMessage tf_msg;
double x_pos = 0, y_pos = 0, z_pos = 0;
double vx = 1;
double a_x = 0, v_yaw = 0;
geometry_msgs__msg__Quaternion euler_to_quaternion(double roll, double pitch, double yaw) {
    geometry_msgs__msg__Quaternion q;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
		//Get time actual from agent ros to mcu
        uint64_t time_ns = rmw_uros_epoch_nanos();
        odom_msg.header.stamp.sec     = time_ns / 1000000000ULL;
        odom_msg.header.stamp.nanosec = time_ns % 1000000000ULL;

        geometry_msgs__msg__Quaternion q = euler_to_quaternion(roll, pitch, 0);
        // update data /odom

        x_pos = x_pos;
        odom_msg.pose.pose.position.x = x_pos;
        odom_msg.pose.pose.position.y = y_pos;
        odom_msg.pose.pose.position.z = z_pos;
        odom_msg.pose.pose.orientation = q;
        odom_msg.twist.twist.linear.x = 0.05;
        odom_msg.twist.twist.angular.z = 0.0;


        tf.header.stamp.sec = time_ns / 1000000000ULL;
        tf.header.stamp.nanosec = time_ns % 1000000000ULL;

        tf.transform.translation.x = x_pos;
        tf.transform.translation.y = y_pos;
        tf.transform.translation.z = z_pos;

        tf.transform.rotation = q;

        tf_msg.transforms.data = &tf;
        tf_msg.transforms.size = 1;
        tf_msg.transforms.capacity = 1;
		RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
		RCSOFTCHECK(rcl_publish(&tf_pub, &tf_msg, NULL));
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

	  // micro-ROS configuration

	  rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart2,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);

	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	      printf("Error on default allocators (line %d)\n", __LINE__);
	  }

	 // Initialize micro-ROS allocator
	 allocator = rcl_get_default_allocator();

	 // create init_options
	 rclc_support_init(&support, 0, NULL, &allocator);

	 //create node_sub
	 rclc_node_init_default(&node, "stm32_node","", &support);

	  // create publisher
	 rclc_publisher_init_default(
	    &odom_pub,
	    &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
	    "/odom_data");


	//create subscriber
	rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"/cmd_vel");

	rclc_publisher_init_default(
	    &tf_pub,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
	    "/tf");
	// create timer
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&executor, &support.context, 2, &allocator);

	// add subscriber callback to the executor
	rclc_executor_add_subscription(&executor, &subscriber, &msg_cmd_vel, &cmd_vel_callback, ON_NEW_DATA);
	// add time for executor
	rclc_executor_add_timer(&executor, &timer);

    if (rmw_uros_sync_session(10) != RMW_RET_OK) {
        printf("Time sync failed\n");
    }

    // init data odom
    odom_msg.header.frame_id.data = "odom";
    odom_msg.child_frame_id.data  = "base_link";

    tf.header.frame_id.data = "odom";
    tf.child_frame_id.data = "base_link";
	while(1) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		MPU6050ReadA();
		MPU6050ReadG();
		filter(AX, AY, AZ, GX, GY,GZ);
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		vTaskDelay(pdMS_TO_TICKS(1));
	}
  /* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
