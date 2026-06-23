#include <algorithm>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "bunker_msgs/msg/bunker_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"

class OdometerNode : public rclcpp::Node
{
public:
  OdometerNode()
  : Node("odometer"),
    x_(0.0),
    y_(0.0),
    yaw_(0.0),
    linear_vel_(0.0),
    angular_vel_(0.0),
    initialized_time_(false),
    got_initial_pose_(false)
  {
    kv_left_ = this->declare_parameter<double>("kv_left", 0.00117);
    kv_right_ = this->declare_parameter<double>("kv_right", 0.00117);
    kw_ = this->declare_parameter<double>("kw", 0.00167);
    angular_bias_ = this->declare_parameter<double>("angular_bias", 0.0);
    rpm_diff_deadband_ = this->declare_parameter<double>("rpm_diff_deadband", 5.0);

    invert_left_ = this->declare_parameter<bool>("invert_left", false);
    invert_right_ = this->declare_parameter<bool>("invert_right", false);
    swap_left_right_ = this->declare_parameter<bool>("swap_left_right", false);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/project_odom", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_init_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&OdometerNode::odomInitCallback, this, std::placeholders::_1));

    status_sub_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>(
      "/bunker_status",
      10,
      std::bind(&OdometerNode::statusCallback, this, std::placeholders::_1));

    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reset",
      std::bind(
        &OdometerNode::resetCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      this->get_logger(),
      "Odometer started. kv_left=%.6f, kv_right=%.6f, kw=%.6f, angular_bias=%.6f, deadband=%.2f",
      kv_left_,
      kv_right_,
      kw_,
      angular_bias_,
      rpm_diff_deadband_);
  }

private:
  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }

    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }

    return angle;
  }

  void odomInitCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (got_initial_pose_) {
      return;
    }

    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    yaw_ = normalizeAngle(yaw);
    got_initial_pose_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Initial pose from /odom: x=%.3f y=%.3f yaw=%.3f",
      x_,
      y_,
      yaw_);
  }

  bool extractLeftRightRpm(
    const bunker_msgs::msg::BunkerStatus::SharedPtr msg,
    double & left_rpm,
    double & right_rpm)
  {
    bool got_right = false;
    bool got_left = false;

    right_rpm = 0.0;
    left_rpm = 0.0;

    for (const auto & motor : msg->actuator_states) {
      const double rpm = static_cast<double>(motor.rpm);

      if (std::abs(rpm) < 1e-6) {
        continue;
      }

      if (motor.motor_id == 0 && !got_right) {
        right_rpm = rpm;
        got_right = true;
      } else if (motor.motor_id == 1 && !got_left) {
        left_rpm = rpm;
        got_left = true;
      }
    }

    if (swap_left_right_) {
      std::swap(left_rpm, right_rpm);
    }

    if (invert_left_) {
      left_rpm = -left_rpm;
    }

    if (invert_right_) {
      right_rpm = -right_rpm;
    }

    return got_left && got_right;
  }

  void statusCallback(const bunker_msgs::msg::BunkerStatus::SharedPtr msg)
  {
    if (!got_initial_pose_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Waiting for first /odom message to initialize pose...");
      return;
    }

    const rclcpp::Time current_time = this->now();

    if (!initialized_time_) {
      last_time_ = current_time;
      initialized_time_ = true;

      publishOdometry(current_time);
      publishTF(current_time);

      RCLCPP_INFO(this->get_logger(), "First /bunker_status received. Starting integration.");
      return;
    }

    const double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

    double left_rpm = 0.0;
    double right_rpm = 0.0;

    if (!extractLeftRightRpm(msg, left_rpm, right_rpm)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Failed to extract valid left/right RPM.");
      return;
    }

    const double v_left = kv_left_ * left_rpm;
    const double v_right = kv_right_ * right_rpm;
    const double rpm_diff = right_rpm - left_rpm;

    linear_vel_ = (v_right + v_left) / 2.0;

    if (std::abs(rpm_diff) < rpm_diff_deadband_) {
      angular_vel_ = 0.0;
    } else {
      angular_vel_ = kw_ * rpm_diff + angular_bias_;
    }

    const double new_yaw = yaw_ + angular_vel_ * dt;

    if (std::abs(angular_vel_) < 1e-6) {
      x_ += linear_vel_ * std::cos(yaw_) * dt;
      y_ += linear_vel_ * std::sin(yaw_) * dt;
    } else {
      const double radius = linear_vel_ / angular_vel_;

      x_ += radius * (std::sin(new_yaw) - std::sin(yaw_));
      y_ -= radius * (std::cos(new_yaw) - std::cos(yaw_));
    }

    yaw_ = normalizeAngle(new_yaw);

    publishOdometry(current_time);
    publishTF(current_time);
  }

  void publishOdometry(const rclcpp::Time & current_time)
  {
    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link2";

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);

    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = linear_vel_;
    odom_msg.twist.twist.angular.z = angular_vel_;

    odom_pub_->publish(odom_msg);
  }

  void publishTF(const rclcpp::Time & current_time)
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link2";

    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);

    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;

    linear_vel_ = 0.0;
    angular_vel_ = 0.0;

    initialized_time_ = false;

    RCLCPP_INFO(this->get_logger(), "Odometry reset to (0, 0, 0).");
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_init_sub_;
  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Time last_time_;

  double x_;
  double y_;
  double yaw_;

  double linear_vel_;
  double angular_vel_;

  double kv_left_;
  double kv_right_;
  double kw_;
  double angular_bias_;
  double rpm_diff_deadband_;

  bool invert_left_;
  bool invert_right_;
  bool swap_left_right_;

  bool initialized_time_;
  bool got_initial_pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometerNode>());
  rclcpp::shutdown();
  return 0;
}