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

MPU6050_t MPU6050;
rcl_publisher_t odom_pub;
rcl_publisher_t tf_pub;
rcl_subscription_t subscriber;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
nav_msgs__msg__Odometry odom_msg;
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
Velocity convertVrVlYaw(double vl_cur, double vr_cur, double yaw, double L) {
    Velocity vel;

    yaw = yaw / RAD_TO_DEG; // yaw (rad)
    v_cur_mps = (vl_cur_mps + vr_cur_mps) / 2.0;   // mps

    vel.vx = v_cur_mps * cos(yaw);
    vel.vy = v_cur_mps * sin(yaw);
    vel.v_yaw = (vr_cur_mps - vl_cur_mps) / L;    // rad/s

    return vel;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	if (timer != NULL) {
		//Get time actual from agent ros to mcu
		static uint64_t last_time_ns = 0;
        uint64_t time_ns = rmw_uros_epoch_nanos();
        odom_msg.header.stamp.sec     = time_ns / 1000000000ULL;
        odom_msg.header.stamp.nanosec = time_ns % 1000000000ULL;

        roll = MPU6050.KalmanAngleX;
        pitch = MPU6050.KalmanAngleY;
        yaw = MPU6050.Yaw;
        geometry_msgs__msg__Quaternion q = euler_to_quaternion(0, 0, yaw);


        Velocity vel = convertVrVlYaw(vr_cur, vl_cur, yaw, TRACK_WIDTH_M); // vr_cur: rpm
        // update data /odom

        double dt = (time_ns - last_time_ns) / 1e9;
        last_time_ns = time_ns;

        x_pos = x_pos + vel.vx * dt;
        y_pos = y_pos + vel.vy * dt;

        odom_msg.pose.pose.position.x = x_pos;
        odom_msg.pose.pose.position.y = y_pos;
        odom_msg.pose.pose.position.z = z_pos;
        odom_msg.pose.pose.orientation =	 q;

        odom_msg.twist.twist.linear.x = v_cur_mps;
        odom_msg.twist.twist.linear.y = 0.00;
        odom_msg.twist.twist.angular.z = vel.v_yaw;


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

void cmd_vel_callback(const void * msgin)
{
  // Cast received message to used type
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

   v_mps = msg->linear.x;
   omega = msg->angular.z;

   vl = (2 * v_mps - omega * TRACK_WIDTH_M) / 2; // m/s
   vr = (2 * v_mps + omega * TRACK_WIDTH_M) / 2; // m/s
}
