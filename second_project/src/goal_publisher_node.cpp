#include <memory>
#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <deque>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

struct Goal {
    double x;
    double y;
    double theta;
};

std::vector<Goal> readGoalsFromCSV(const std::string& file_path) {
    std::vector<Goal> goals;
    std::ifstream file(file_path);

    if (!file.is_open()) {
        std::cerr << "[ERROR] Failed to open CSV file: " << file_path << std::endl;
        return goals;
    }

    std::string line;
    bool first_line = true;

    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }

        if (first_line) {
            first_line = false;
            continue;
        }

        std::stringstream ss(line);
        std::string x_str;
        std::string y_str;
        std::string theta_str;

        if (
            std::getline(ss, x_str, ',') &&
            std::getline(ss, y_str, ',') &&
            std::getline(ss, theta_str, ',')
        ) {
            try {
                Goal goal;
                goal.x = std::stod(x_str);
                goal.y = std::stod(y_str);
                goal.theta = std::stod(theta_str);
                goals.push_back(goal);
            } catch (const std::exception& e) {
                std::cerr << "[WARN] Failed to parse CSV line: "
                          << line
                          << ". Reason: "
                          << e.what()
                          << std::endl;
            }
        }
    }

    file.close();
    return goals;
}

class GoalPublisherNode : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using ClearEntireCostmap = nav2_msgs::srv::ClearEntireCostmap;
    using Twist = geometry_msgs::msg::Twist;
    using Marker = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;

    GoalPublisherNode()
        : Node("goal_publisher_node"),
          current_goal_index_(0),
          escape_publish_count_(0),
          escaping_(false)
    {
        action_client_ =
            rclcpp_action::create_client<NavigateToPose>(
                this,
                "/navigate_to_pose"
            );

        clear_global_costmap_client_ =
            this->create_client<ClearEntireCostmap>(
                "/global_costmap/clear_entirely_global_costmap"
            );

        clear_local_costmap_client_ =
            this->create_client<ClearEntireCostmap>(
                "/local_costmap/clear_entirely_local_costmap"
            );

        goal_marker_publisher_ =
            this->create_publisher<MarkerArray>(
                "/waypoints",
                rclcpp::QoS(1).transient_local().reliable()
            );

        cmd_vel_publisher_ =
            this->create_publisher<Twist>(
                "/cmd_vel",
                rclcpp::QoS(10)
            );

        cmd_vel_subscription_ =
            this->create_subscription<Twist>(
                "/cmd_vel",
                rclcpp::QoS(10),
                std::bind(
                    &GoalPublisherNode::cmd_vel_callback,
                    this,
                    std::placeholders::_1
                )
            );

        std::string package_share_dir =
            ament_index_cpp::get_package_share_directory("second_project");

        std::string csv_path = package_share_dir + "/csv/goals.csv";

        RCLCPP_INFO(
            this->get_logger(),
            "Loading goals from CSV file: %s",
            csv_path.c_str()
        );

        goals_ = readGoalsFromCSV(csv_path);
        abort_counts_.assign(goals_.size(), 0);

        if (goals_.empty()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "The CSV file is empty or could not be loaded."
            );
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Loaded %zu goals from CSV.",
            goals_.size()
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2000),
            std::bind(&GoalPublisherNode::send_next_goal, this)
        );
    }

private:
    static constexpr double kEscapeSpeed = 0.10;
    static constexpr double kEscapeDurationSeconds = 1.5;
    static constexpr double kCmdVelHistorySeconds = 1.0;
    static constexpr double kMinimumRecordedSpeed = 0.03;
    static constexpr int kEscapeTimerPeriodMs = 100;
    static constexpr int kEscapePublishLimit =
        static_cast<int>(
            kEscapeDurationSeconds * 1000.0 / kEscapeTimerPeriodMs
        );
    static constexpr int kMaxAbortFailuresPerGoal = 2;

    enum class LogLevel {
        Info,
        Warn,
        Error
    };

    struct TimedTwist {
        rclcpp::Time stamp;
        Twist twist;
    };

    void log_if_changed(
        const std::string& key,
        LogLevel level,
        const std::string& message
    ) {
        auto last_message = last_log_messages_.find(key);

        if (
            last_message != last_log_messages_.end() &&
            last_message->second == message
        ) {
            return;
        }

        last_log_messages_[key] = message;

        switch (level) {
            case LogLevel::Info:
                RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
                break;

            case LogLevel::Warn:
                RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
                break;

            case LogLevel::Error:
                RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
                break;
        }
    }

    void clear_costmaps() {
        auto global_request =
            std::make_shared<ClearEntireCostmap::Request>();

        auto local_request =
            std::make_shared<ClearEntireCostmap::Request>();

        if (
            clear_global_costmap_client_->wait_for_service(
                std::chrono::milliseconds(500)
            )
        ) {
            clear_global_costmap_client_->async_send_request(global_request);

            log_if_changed(
                "global_costmap_clear",
                LogLevel::Info,
                "Requested global costmap clearing."
            );
        } else {
            log_if_changed(
                "global_costmap_clear",
                LogLevel::Warn,
                "Global costmap clearing service is not available."
            );
        }

        if (
            clear_local_costmap_client_->wait_for_service(
                std::chrono::milliseconds(500)
            )
        ) {
            clear_local_costmap_client_->async_send_request(local_request);

            log_if_changed(
                "local_costmap_clear",
                LogLevel::Info,
                "Requested local costmap clearing."
            );
        } else {
            log_if_changed(
                "local_costmap_clear",
                LogLevel::Warn,
                "Local costmap clearing service is not available."
            );
        }
    }

    double get_goal_yaw(size_t goal_index) const {
        return goals_[goal_index].theta;
    }

    void publish_next_goal_marker(
        const Goal& goal,
        size_t goal_index,
        double goal_yaw
    ) {
        MarkerArray marker_array;

        Marker delete_previous_markers;
        delete_previous_markers.header.frame_id = "map";
        delete_previous_markers.header.stamp = this->now();
        delete_previous_markers.action = Marker::DELETEALL;
        marker_array.markers.push_back(delete_previous_markers);

        Marker goal_marker;
        goal_marker.header.frame_id = "map";
        goal_marker.header.stamp = delete_previous_markers.header.stamp;
        goal_marker.ns = "next_goal";
        goal_marker.id = 0;
        goal_marker.type = Marker::SPHERE;
        goal_marker.action = Marker::ADD;
        goal_marker.pose.position.x = goal.x;
        goal_marker.pose.position.y = goal.y;
        goal_marker.pose.position.z = 0.15;
        goal_marker.pose.orientation.w = 1.0;
        goal_marker.scale.x = 0.45;
        goal_marker.scale.y = 0.45;
        goal_marker.scale.z = 0.30;
        goal_marker.color.r = 0.0;
        goal_marker.color.g = 1.0;
        goal_marker.color.b = 0.0;
        goal_marker.color.a = 0.95;
        marker_array.markers.push_back(goal_marker);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, goal_yaw);
        q.normalize();

        Marker heading_marker;
        heading_marker.header.frame_id = "map";
        heading_marker.header.stamp = delete_previous_markers.header.stamp;
        heading_marker.ns = "next_goal";
        heading_marker.id = 1;
        heading_marker.type = Marker::ARROW;
        heading_marker.action = Marker::ADD;
        heading_marker.pose.position.x = goal.x;
        heading_marker.pose.position.y = goal.y;
        heading_marker.pose.position.z = 0.35;
        heading_marker.pose.orientation.x = q.x();
        heading_marker.pose.orientation.y = q.y();
        heading_marker.pose.orientation.z = q.z();
        heading_marker.pose.orientation.w = q.w();
        heading_marker.scale.x = 0.70;
        heading_marker.scale.y = 0.12;
        heading_marker.scale.z = 0.12;
        heading_marker.color.r = 1.0;
        heading_marker.color.g = 0.85;
        heading_marker.color.b = 0.0;
        heading_marker.color.a = 1.0;
        marker_array.markers.push_back(heading_marker);

        Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = delete_previous_markers.header.stamp;
        text_marker.ns = "next_goal";
        text_marker.id = 2;
        text_marker.type = Marker::TEXT_VIEW_FACING;
        text_marker.action = Marker::ADD;
        text_marker.pose.position.x = goal.x;
        text_marker.pose.position.y = goal.y;
        text_marker.pose.position.z = 0.85;
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.35;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;

        std::ostringstream label;
        label << "Goal " << goal_index + 1 << "/" << goals_.size();
        text_marker.text = label.str();
        marker_array.markers.push_back(text_marker);

        goal_marker_publisher_->publish(marker_array);
    }

    void clear_goal_marker() {
        MarkerArray marker_array;

        Marker delete_marker;
        delete_marker.header.frame_id = "map";
        delete_marker.header.stamp = this->now();
        delete_marker.action = Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        goal_marker_publisher_->publish(marker_array);
    }

    void send_next_goal() {
        if (current_goal_index_ >= goals_.size()) {
            log_if_changed(
                "goal_sequence",
                LogLevel::Info,
                "All goals have been reached."
            );

            timer_->cancel();
            clear_goal_marker();
            return;
        }

        if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            log_if_changed(
                "nav2_action_server",
                LogLevel::Info,
                "Waiting for the Nav2 action server: /navigate_to_pose..."
            );
            return;
        }

        timer_->cancel();

        clear_costmaps();

        auto goal_msg = NavigateToPose::Goal();

        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();

        const auto& current_goal = goals_[current_goal_index_];
        const double goal_yaw = get_goal_yaw(current_goal_index_);
        publish_next_goal_marker(
            current_goal,
            current_goal_index_,
            goal_yaw
        );

        goal_msg.pose.pose.position.x = current_goal.x;
        goal_msg.pose.pose.position.y = current_goal.y;
        goal_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, goal_yaw);
        q.normalize();

        goal_msg.pose.pose.orientation.x = q.x();
        goal_msg.pose.pose.orientation.y = q.y();
        goal_msg.pose.pose.orientation.z = q.z();
        goal_msg.pose.pose.orientation.w = q.w();

        RCLCPP_INFO(
            this->get_logger(),
            "Sending goal %zu/%zu: x=%.2f, y=%.2f, yaw=%.2f",
            current_goal_index_ + 1,
            goals_.size(),
            current_goal.x,
            current_goal.y,
            goal_yaw
        );

        auto send_goal_options =
            rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            std::bind(
                &GoalPublisherNode::goal_response_callback,
                this,
                std::placeholders::_1
            );

        send_goal_options.result_callback =
            std::bind(
                &GoalPublisherNode::result_callback,
                this,
                std::placeholders::_1
            );

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(
        const GoalHandleNav::SharedPtr& goal_handle
    ) {
        if (!goal_handle) {
            std::ostringstream message;
            message << "Goal "
                    << current_goal_index_ + 1
                    << " was rejected by the Nav2 action server. Retrying.";

            log_if_changed(
                "goal_response",
                LogLevel::Error,
                message.str()
            );

            clear_costmaps();

            timer_->reset();
            return;
        }

        std::ostringstream message;
        message << "Goal "
                << current_goal_index_ + 1
                << " was accepted by the Nav2 action server.";

        log_if_changed(
            "goal_response",
            LogLevel::Info,
            message.str()
        );
    }

    void prune_cmd_vel_history(const rclcpp::Time& now) {
        while (
            !cmd_vel_history_.empty() &&
            (now - cmd_vel_history_.front().stamp).seconds() >
                kCmdVelHistorySeconds
        ) {
            cmd_vel_history_.pop_front();
        }
    }

    void cmd_vel_callback(const Twist::SharedPtr msg) {
        if (escaping_) {
            return;
        }

        const rclcpp::Time now = this->now();
        prune_cmd_vel_history(now);
        cmd_vel_history_.push_back(TimedTwist{now, *msg});
    }

    bool build_escape_twist_from_cmd_history(
        Twist& escape_twist,
        double& average_vx,
        double& average_vy,
        double& average_speed
    ) {
        const rclcpp::Time now = this->now();
        prune_cmd_vel_history(now);

        if (cmd_vel_history_.empty()) {
            return false;
        }

        average_vx = 0.0;
        average_vy = 0.0;

        for (const auto& sample : cmd_vel_history_) {
            average_vx += sample.twist.linear.x;
            average_vy += sample.twist.linear.y;
        }

        average_vx /= static_cast<double>(cmd_vel_history_.size());
        average_vy /= static_cast<double>(cmd_vel_history_.size());
        average_speed =
            std::sqrt(average_vx * average_vx + average_vy * average_vy);

        if (average_speed < kMinimumRecordedSpeed) {
            return false;
        }

        escape_twist.linear.x = -kEscapeSpeed * average_vx / average_speed;
        escape_twist.linear.y = -kEscapeSpeed * average_vy / average_speed;
        escape_twist.linear.z = 0.0;
        escape_twist.angular.x = 0.0;
        escape_twist.angular.y = 0.0;
        escape_twist.angular.z = 0.0;

        return true;
    }

    void execute_smart_escape_then_retry() {
        Twist escape_twist;
        double average_vx = 0.0;
        double average_vy = 0.0;
        double average_speed = 0.0;

        if (
            !build_escape_twist_from_cmd_history(
                escape_twist,
                average_vx,
                average_vy,
                average_speed
            )
        ) {
            RCLCPP_WARN(
                this->get_logger(),
                "Velocity-based escape skipped because recent /cmd_vel linear speed is below %.2f m/s. Clearing costmaps and retrying goal %zu.",
                kMinimumRecordedSpeed,
                current_goal_index_ + 1
            );

            clear_costmaps();
            timer_->reset();
            return;
        }

        escape_twist_ = escape_twist;
        escape_publish_count_ = 0;
        escaping_ = true;

        RCLCPP_WARN(
            this->get_logger(),
            "Velocity-based escape for goal %zu: recent average vx=%.2f, vy=%.2f, speed=%.2f. Escaping with vx=%.2f, vy=%.2f.",
            current_goal_index_ + 1,
            average_vx,
            average_vy,
            average_speed,
            escape_twist_.linear.x,
            escape_twist_.linear.y
        );

        if (escape_timer_) {
            escape_timer_->cancel();
        }

        escape_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(kEscapeTimerPeriodMs),
            std::bind(&GoalPublisherNode::publish_escape_velocity, this)
        );
    }

    void publish_escape_velocity() {
        if (!escaping_) {
            return;
        }

        if (escape_publish_count_ < kEscapePublishLimit) {
            cmd_vel_publisher_->publish(escape_twist_);
            escape_publish_count_++;
            return;
        }

        Twist stop_twist;
        cmd_vel_publisher_->publish(stop_twist);

        escaping_ = false;

        if (escape_timer_) {
            escape_timer_->cancel();
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Velocity-based escape complete. Clearing costmaps and retrying goal %zu.",
            current_goal_index_ + 1
        );

        clear_costmaps();
        timer_->reset();
    }

    void handle_aborted_goal() {
        if (current_goal_index_ >= goals_.size()) {
            return;
        }

        abort_counts_[current_goal_index_]++;

        if (abort_counts_[current_goal_index_] > kMaxAbortFailuresPerGoal) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Goal %zu was aborted more than %d times. Skipping this goal.",
                current_goal_index_ + 1,
                kMaxAbortFailuresPerGoal
            );

            clear_costmaps();
            current_goal_index_++;
            timer_->reset();
            return;
        }

        RCLCPP_ERROR(
            this->get_logger(),
            "Goal %zu was aborted. Running smart escape before retry %d/%d.",
            current_goal_index_ + 1,
            abort_counts_[current_goal_index_],
            kMaxAbortFailuresPerGoal
        );

        execute_smart_escape_then_retry();
    }

    void result_callback(
        const GoalHandleNav::WrappedResult& result
    ) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            {
                std::ostringstream message;
                message << "Goal "
                        << current_goal_index_ + 1
                        << " reached successfully.";

                log_if_changed(
                    "goal_result",
                    LogLevel::Info,
                    message.str()
                );

                clear_costmaps();

                abort_counts_[current_goal_index_] = 0;
                current_goal_index_++;

                timer_->reset();
                break;
            }

            case rclcpp_action::ResultCode::ABORTED:
            {
                handle_aborted_goal();
                break;
            }

            case rclcpp_action::ResultCode::CANCELED:
            {
                std::ostringstream message;
                message << "Goal "
                        << current_goal_index_ + 1
                        << " was canceled. Retrying the same goal.";

                log_if_changed(
                    "goal_result",
                    LogLevel::Warn,
                    message.str()
                );

                clear_costmaps();

                timer_->reset();
                break;
            }

            default:
            {
                std::ostringstream message;
                message << "Goal "
                        << current_goal_index_ + 1
                        << " finished with an unknown result code. Retrying the same goal.";

                log_if_changed(
                    "goal_result",
                    LogLevel::Error,
                    message.str()
                );

                clear_costmaps();

                timer_->reset();
                break;
            }
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Client<ClearEntireCostmap>::SharedPtr clear_global_costmap_client_;
    rclcpp::Client<ClearEntireCostmap>::SharedPtr clear_local_costmap_client_;
    rclcpp::Publisher<MarkerArray>::SharedPtr goal_marker_publisher_;
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr escape_timer_;
    std::vector<Goal> goals_;
    std::vector<int> abort_counts_;
    std::unordered_map<std::string, std::string> last_log_messages_;
    std::deque<TimedTwist> cmd_vel_history_;
    Twist escape_twist_;
    size_t current_goal_index_;
    int escape_publish_count_;
    bool escaping_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GoalPublisherNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
