#include "publish.h"


double x_pos = 0, y_pos = 0, z_pos = 0;
double vx = 0, vy = 0;
double v_yaw = 0;
double vl_cur_mps = 0, vr_cur_mps = 0, v_cur_mps = 0;
double vr_cur, vl_cur;
double roll, pitch, yaw;
int cnt_pub = 0, cnt_imu = 0, cnt_control = 0;

double v_mps, omega;
double vl, vr;
#define ODOM_COV 	0.005
#define IMU_COV 0.005;

MPU6050_t MPU6050;
rcl_publisher_t odom_pub;
rcl_publisher_t tf_pub;
rcl_publisher_t imu_pub;
rcl_subscription_t subscriber;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__TransformStamped tf;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__Twist msg_cmd_vel;

geometry_msgs__msg__Quaternion euler_to_quaternion(double roll, double pitch, double yaw) {
    geometry_msgs__msg__Quaternion q;

    yaw = yaw / RAD_TO_DEG;
    roll = roll / RAD_TO_DEG;
    pitch = pitch / RAD_TO_DEG;

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


// Differential Drive Kinematic Model
Velocity vel;
Velocity convertVrVlYaw(double vl_cur, double vr_cur, double yaw, double L) {
    vl_cur_mps = vl_cur * ((2.0f * 3.1415926f * WHEEL_RADIUS_M)) / 60;   // rpm -> mps
    vr_cur_mps = vr_cur * ((2.0f * 3.1415926f * WHEEL_RADIUS_M)) / 60;   // rpm -> mps

    yaw = yaw / RAD_TO_DEG; // yaw (rad)
    v_cur_mps = (vl_cur_mps + vr_cur_mps) / 2.0;   // mps

    vel.vx = v_cur_mps * cos(yaw);
    vel.vy = v_cur_mps * sin(yaw);
    vel.v_yaw = - (vr_cur_mps - vl_cur_mps) / L;    // rad/s

    return vel;
}
double dt;
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	if (timer != NULL) {
		//Get time actual from agent ros to mcu
		static uint64_t last_time_ns = 0;

        uint64_t time_ns = rmw_uros_epoch_nanos();
        if (last_time_ns == 0) {
                last_time_ns = time_ns;
                return;
        }
        odom_msg.header.stamp.sec     = time_ns / 1000000000ULL;
        odom_msg.header.stamp.nanosec = time_ns % 1000000000ULL;

        roll = MPU6050.KalmanAngleX;
        pitch = MPU6050.KalmanAngleY;
        yaw = MPU6050.Yaw;
        geometry_msgs__msg__Quaternion q = euler_to_quaternion(0, 0, yaw);


        Velocity vel = convertVrVlYaw(vr_cur, vl_cur, yaw, TRACK_WIDTH_M); // vr_cur: rpm
        dt = (time_ns - last_time_ns) / 1e9;
        last_time_ns = time_ns;

        odom_msg.twist.twist.linear.x = v_cur_mps;
        odom_msg.twist.twist.linear.y = 0.00;
        odom_msg.twist.twist.angular.z = vel.v_yaw;

        odom_msg.twist.covariance[0] = ODOM_COV; // vx
        odom_msg.twist.covariance[7] = ODOM_COV;   //vy
        odom_msg.twist.covariance[14] = 1e6;
        odom_msg.twist.covariance[21] = 1e6;
        odom_msg.twist.covariance[28] = 1e6;
        odom_msg.twist.covariance[35] = ODOM_COV;  // v_yaw

        imu_msg.header.stamp.sec = time_ns / 1000000000ULL;
        imu_msg.header.stamp.nanosec = time_ns % 1000000000ULL;
        imu_msg.header.frame_id.data = "base_link";

        imu_msg.orientation = q; 								// yaw
        imu_msg.angular_velocity.z = MPU6050.Gz * DEG_TO_RAD;   // v_yaw
        imu_msg.linear_acceleration.x = MPU6050.Ax * 9.80665;  // a_x m/s²

        imu_msg.orientation_covariance[0] = 1e6;
        imu_msg.orientation_covariance[4] = 1e6;
        imu_msg.orientation_covariance[8] = IMU_COV; // yaw

        imu_msg.angular_velocity_covariance[0] = 1e6;
        imu_msg.angular_velocity_covariance[4] = 1e6;
        imu_msg.angular_velocity_covariance[8] = IMU_COV; //v_yaw

        imu_msg.linear_acceleration_covariance[0] = IMU_COV;  //a_x
        imu_msg.linear_acceleration_covariance[4] = 1e6;
        imu_msg.linear_acceleration_covariance[8] = 1e6;


        RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
		RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
	}
}

void cmd_vel_callback(const void * msgin)
{
  // Cast received message to used type
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

   v_mps = msg->linear.x;
   omega = msg->angular.z;

   vl = (2 * v_mps - omega * TRACK_WIDTH_M) / 2; // m/s
   vr = (2 * v_mps + omega * TRACK_WIDTH_M) / 2; // m/s
}
