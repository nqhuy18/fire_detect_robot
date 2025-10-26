#ifndef PUBLISH_H_
#define PUBLISH_H_
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
#include <sensor_msgs/msg/imu.h>

#include <Motor.h>
#include <mpu6050.h>
#include <pid.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define DEG_TO_RAD 0.01745329252
extern double x_pos, y_pos, z_pos;
extern double vx, vy;
extern double v_yaw;
extern double vl_cur_mps, vr_cur_mps, v_cur_mps;
extern double vr_cur, vl_cur;
extern double roll, pitch, yaw;
extern int cnt_pub, cnt_imu, cnt_control;
extern double vl, vr;
extern double v_mps, omega;
extern Motor Left_motor;
extern Motor Right_motor;
extern PID_TypeDef RPID;
extern PID_TypeDef LPID;
extern MPU6050_t MPU6050;
typedef struct Velocity {
    double vx;      // m/s
    double vy;      // m/s
    double v_yaw;   // rad/s
} Velocity;

extern rcl_publisher_t odom_pub, tf_pub, imu_pub;
extern rcl_subscription_t subscriber;

extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;
extern nav_msgs__msg__Odometry odom_msg;
extern sensor_msgs__msg__Imu imu_msg;
extern geometry_msgs__msg__TransformStamped tf;
extern tf2_msgs__msg__TFMessage tf_msg;
extern geometry_msgs__msg__Twist msg_cmd_vel;

geometry_msgs__msg__Quaternion euler_to_quaternion(double roll, double pitch, double yaw);
Velocity convertVrVlYaw(double vl_cur, double vr_cur, double yaw, double L);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void cmd_vel_callback(const void * msgin);

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

#endif
