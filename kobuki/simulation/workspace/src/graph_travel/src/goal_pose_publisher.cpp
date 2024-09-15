#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "graph.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


namespace {
    float distance_remaining = 0;
    float old_distance_remaining = 0;
    int failed_pubs = 0;
    geometry_msgs::msg::PoseStamped old_msg;
    float x_shift = 0.0;
    float y_shift = 0.0;
    bool cmd_shutdown_flag = false;
}


class GoalPosePublisher : public rclcpp::Node
{
public:
    GoalPosePublisher()
        : Node("goal_pose_publisher")
    {
        RCLCPP_INFO(this->get_logger(), "Started");
        this->declare_parameter("allow_travel", false);
        this->declare_parameter("is_sim", true);
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1);

        cmd_vel_warper_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/is_cmd_vel_allow", 1);

        subscription_ = this->create_subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>(
            "/navigate_to_pose/_action/feedback", 1, std::bind(&GoalPosePublisher::topic_callback, this, _1));

        aruco_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/is_slave_finded", 10, std::bind(&GoalPosePublisher::aruco_topic_callback, this, _1));

        timer_ = this->create_wall_timer(
            2000ms, std::bind(&GoalPosePublisher::timer_callback, this));

        graph = Graph();
    }

private:
    void timer_callback()
    {
        if (not this->get_parameter("allow_travel").as_bool()) {
            return;
        }

        geometry_msgs::msg::PoseStamped message;
        old_msg = graph.currentMsg();
        float delta_distance_remaining = distance_remaining - old_distance_remaining;
        if (delta_distance_remaining < 0.1) {
            failed_pubs++;
        }

        if (distance_remaining < 0.3) {
            message = graph.dropMsg();
            failed_pubs = 0;
        }
        else {
            message = graph.currentMsg();

        if (message == old_msg && failed_pubs == 10) {
            message = graph.dropMsg();
            failed_pubs = 0;
        }
        }

        message.header.frame_id = "map";
        message.header.stamp = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Publishing: '%lf %lf, %lf %lf'", message.pose.position.x, message.pose.position.y,
        distance_remaining, old_distance_remaining);
        if (this->get_parameter("is_sim").as_bool()) {
            x_shift = 0.0;
            y_shift = 0.0;
        }
        else {
            x_shift = 0.34;
            y_shift = 0.18;
        }
        message.pose.position.x += x_shift;
        message.pose.position.y += y_shift;
        publisher_->publish(message);
    }

    void topic_callback(const nav2_msgs::action::NavigateToPose_FeedbackMessage& log)
    {
        old_distance_remaining = distance_remaining;
        distance_remaining = log.feedback.distance_remaining;
    }

    void aruco_topic_callback(const std_msgs::msg::Bool& is_aruco_found)
    {
        if (cmd_shutdown_flag) {
            return;
        }

        if (is_aruco_found.data) {
            rclcpp::Parameter allow_travel("allow_travel", false);
            this->set_parameter(allow_travel);
            std_msgs::msg::Bool msg;
            msg.data = false;
            cmd_vel_warper_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "ARUCO_FOUND");
            cmd_shutdown_flag = true;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aruco_subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cmd_vel_warper_publisher_;
    Graph graph;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
