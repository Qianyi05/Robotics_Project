#include <cmath>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

#include "first_project/msg/tf_error_msg.hpp"

class TfErrorNode : public rclcpp::Node
{
public:
  TfErrorNode()
  : Node("tf_error"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    initialized_(false),
    last_x_(0.0),
    last_y_(0.0),
    travelled_distance_(0.0)
  {
    error_pub_ = this->create_publisher<first_project::msg::TfErrorMsg>("/tf_error_msg", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TfErrorNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "tf_error node started");
  }

private:
  void timerCallback()
  {
    geometry_msgs::msg::TransformStamped gt_tf;
    geometry_msgs::msg::TransformStamped my_tf;

    try {
      // Ground-truth TF from the bag
      gt_tf = tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);

      // Computed odometry TF from our odometer node
      my_tf = tf_buffer_.lookupTransform("odom", "base_link2", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "TF lookup failed: %s",
        ex.what());
      return;
    }

    const rclcpp::Time current_time = this->now();

    const double gt_x = gt_tf.transform.translation.x;
    const double gt_y = gt_tf.transform.translation.y;

    const double my_x = my_tf.transform.translation.x;
    const double my_y = my_tf.transform.translation.y;

    // First valid TF pair: initialize reference time and last GT position.
    // Do not publish error for this first sample.
    if (!initialized_) {
      start_time_ = current_time;

      last_x_ = my_x;
      last_y_ = my_y;

      travelled_distance_ = 0.0;
      initialized_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "tf_error initialized at time %.3f, gt=(%.3f, %.3f), my=(%.3f, %.3f)",
        current_time.seconds(),
        gt_x,
        gt_y,
        my_x,
        my_y);

      return;
    }

    const double dx = gt_x - my_x;
    const double dy = gt_y - my_y;
    const double error = std::sqrt(dx * dx + dy * dy);

    updateTravelledDistance(my_x, my_y);

    const double time_from_start = (current_time - start_time_).seconds();

    first_project::msg::TfErrorMsg msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = "odom";

    msg.tf_error = static_cast<float>(error);
    msg.time_from_start = static_cast<int32_t>(time_from_start);
    msg.travelled_distance = static_cast<float>(travelled_distance_);

    error_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "tf_error=%.3f m, time=%d s, travelled=%.3f m",
      msg.tf_error,
      msg.time_from_start,
      msg.travelled_distance);
  }

  void updateTravelledDistance(double x, double y)
  {
    const double dx = x - last_x_;
    const double dy = y - last_y_;

    travelled_distance_ += std::sqrt(dx * dx + dy * dy);

    last_x_ = x;
    last_y_ = y;
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<first_project::msg::TfErrorMsg>::SharedPtr error_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;

  bool initialized_;

  double last_x_;
  double last_y_;
  double travelled_distance_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfErrorNode>());
  rclcpp::shutdown();
  return 0;
}