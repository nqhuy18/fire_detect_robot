#include <chrono>
#include <memory>
#include <cmath>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class OdomPublisherNoisy : public rclcpp::Node
{
public:
  OdomPublisherNoisy()
  : Node("odom_publisher_noisy"),
    x_(0.0), y_(0.0), th_(0.0),
    vx_(0.0), vy_(0.0), vth_(0.0)   
  {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel/odometry", 50);

    // Timer 30 Hz
    auto period = std::chrono::milliseconds(1000 / 20);
    timer_ = this->create_wall_timer(period, std::bind(&OdomPublisherNoisy::timer_callback, this));

    last_time_ = this->now();

    // Gaussian noise generator (mean=0, stddev=0.02m)
    std::random_device rd;
    gen_ = std::mt19937(rd());
    noise_dist_ = std::normal_distribution<double>(0.0, 0.1);
  }

private:
  void timer_callback()
  {
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    // simulate straight-line motion
    x_ += vx_ * dt;
    y_ += vy_ * dt;
    th_ += vth_ * dt;

    // add noise
    double noisy_x = x_;
    double noisy_y = y_ ;
    double noisy_th = th_+ noise_dist_(gen_);

    // orientation quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, noisy_th);
    geometry_msgs::msg::Quaternion odom_quat;
    odom_quat.x = q.x();
    odom_quat.y = q.y();
    odom_quat.z = q.z();
    odom_quat.w = q.w();

    // build odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = noisy_x;
    odom.pose.pose.position.y = noisy_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.linear.y = vy_;
    odom.twist.twist.angular.z = vth_;

    odom_pub_->publish(odom);

    last_time_ = current_time;
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double x_, y_, th_;
  double vx_, vy_, vth_;
  rclcpp::Time last_time_;

  // random generator for Gaussian noise
  std::mt19937 gen_;
  std::normal_distribution<double> noise_dist_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisherNoisy>());
  rclcpp::shutdown();
  return 0;
}
