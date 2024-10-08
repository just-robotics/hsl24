#include <chrono>
#include <iostream>
#include <memory>
#include <cmath>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


using namespace std::chrono_literals;


class Controller : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr allow_driving_sub_;
    rclcpp::Subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>::SharedPtr feedback_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; 
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr result_pub_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr cancel_navigation_client_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool verbose_;
    // bool use_emergency_stop_;
    bool allow_driving_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    // float backward_linear_vel_;
    nav2_msgs::action::NavigateToPose_FeedbackMessage feedback_;
    double delta_;
    rclcpp::Time t_;
    rclcpp::Time rotate_t_;
    nav_msgs::msg::Odometry odom_;
    double start_angle_;
    bool rotate_ = false;
    bool start_rotate_ = false;

public:
    Controller();

private:
    void delay(size_t ms);
    void cmdVelCallback(geometry_msgs::msg::Twist msg);
    void goalPoseCallback(geometry_msgs::msg::PoseStamped msg);
    void allowDrivingCallback(std_msgs::msg::Bool msg);
    void feedbackCallback(nav2_msgs::action::NavigateToPose_FeedbackMessage msg);
    void odomCallback(nav_msgs::msg::Odometry msg);
    void rotateCallback();

    void cancelNavigation(bool status);
};


Controller::Controller() : Node("master_controller") {
    using std::placeholders::_1;

    this->declare_parameter("verbose", true);
    this->declare_parameter("goal_pose_pub_topic", "");
    this->declare_parameter("goal_pose_sub_topic", "");
    this->declare_parameter("nav2_cmd_vel_topic", "");
    this->declare_parameter("odom_topic", "");
    this->declare_parameter("cmd_vel_topic", "");
    this->declare_parameter("allow_driving_topic", "");
    this->declare_parameter("feedback_topic", "");
    this->declare_parameter("result_topic", "");
    // this->declare_parameter("use_emergency_stop", false);
    // this->declare_parameter("backward_linear_vel", 0.0);
    this->declare_parameter("delta", 0.0);

    verbose_ = this->get_parameter("verbose").as_bool();
    std::string goal_pose_pub_topic = this->get_parameter("goal_pose_pub_topic").as_string();
    std::string goal_pose_sub_topic = this->get_parameter("goal_pose_sub_topic").as_string();
    std::string nav2_cmd_vel_topic = this->get_parameter("nav2_cmd_vel_topic").as_string();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    std::string allow_driving_topic = this->get_parameter("allow_driving_topic").as_string();
    std::string feedback_topic = this->get_parameter("feedback_topic").as_string();
    std::string result_topic = this->get_parameter("result_topic").as_string();
    // use_emergency_stop_ = this->get_parameter("use_emergency_stop").as_bool();
    // backward_linear_vel_ = this->get_parameter("backward_linear_vel").as_double();
    delta_ = this->get_parameter("delta").as_double();

    RCLCPP_INFO(this->get_logger(), "verbose: '%s'", verbose_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "goal_pose_pub_topic: '%s'", goal_pose_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "goal_pose_sub_topic: '%s'", goal_pose_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "nav2_cmd_vel_topic: '%s'", nav2_cmd_vel_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "cmd_vel_topic: '%s'", cmd_vel_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "allow_driving_topic: '%s'", allow_driving_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "feedback_topic: '%s'", feedback_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "result_topic: '%s'", result_topic.c_str());
    // RCLCPP_INFO(this->get_logger(), "use_emergency_stop: '%s'", use_emergency_stop_ ? "true" : "false");
    // RCLCPP_INFO(this->get_logger(), "backward_linear_vel: '%f'", backward_linear_vel_);
    RCLCPP_INFO(this->get_logger(), "delta: '%f'", delta_);

    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_pub_topic, 10);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(nav2_cmd_vel_topic, 10, std::bind(&Controller::cmdVelCallback, this, _1));
    allow_driving_sub_ = this->create_subscription<std_msgs::msg::Bool>(allow_driving_topic, 10, std::bind(&Controller::allowDrivingCallback, this, _1));
    feedback_sub_ = this->create_subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>(feedback_topic, 10, std::bind(&Controller::feedbackCallback, this, _1));
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(goal_pose_sub_topic, 10, std::bind(&Controller::goalPoseCallback, this, _1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&Controller::odomCallback, this, _1));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    result_pub_ = this->create_publisher<std_msgs::msg::Bool>(result_topic, 10);

    cancel_navigation_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this,"/navigate_to_pose");

    allow_driving_ = true;

    timer_ = this->create_wall_timer(10ms, std::bind(&Controller::rotateCallback, this));
}


void Controller::delay(size_t ms) {
    auto start = std::chrono::system_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count() < static_cast<long int>(ms)) {}
}


double euler_from_quaternion(geometry_msgs::msg::Quaternion quaternion) {
    auto x = quaternion.x;
    auto y = quaternion.y;
    auto z = quaternion.z;
    auto w = quaternion.w;

    auto siny_cosp = 2 * (w * z + x * y);
    auto cosy_cosp = 1 - 2 * (y * y + z * z);
    auto yaw = std::atan2(siny_cosp, cosy_cosp) / 3.14 * 180;

    return yaw;
}

void Controller::rotateCallback() {
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = 0.8;

    if (start_rotate_) {
        start_rotate_ = false;
        rotate_ = true;
        auto q = odom_.pose.pose.orientation;
        start_angle_ = euler_from_quaternion(q);
        RCLCPP_INFO(this->get_logger(), "start_angle: %lf", start_angle_);
        cmd_vel_pub_->publish(msg);
        rotate_t_ = this->get_clock()->now();
        return;
    }

    if (this->get_clock()->now().seconds() - rotate_t_.seconds() < 1.0) {
        return;
    }

    if (rotate_) {
        auto q = odom_.pose.pose.orientation;
        double yaw = euler_from_quaternion(q);
        RCLCPP_INFO(this->get_logger(), "angle: %lf", yaw);
        if (std::abs(yaw - start_angle_) < 2) {
            rotate_ = false;
            msg.angular.z = 0.0;
            cmd_vel_pub_->publish(msg);
            auto res = std_msgs::msg::Bool();
            res.data = true;
            result_pub_->publish(res);
        }
        else {
            cmd_vel_pub_->publish(msg);
        }
    }
}


void Controller::cmdVelCallback(geometry_msgs::msg::Twist msg) {
    if (!allow_driving_ || rotate_) {
        RCLCPP_WARN(this->get_logger(), "driving is forbidden");
        return;
    }
    cmd_vel_pub_->publish(msg);
}


void Controller::goalPoseCallback(geometry_msgs::msg::PoseStamped msg) {
    // goal_pose_ = msg;
    t_ = this->get_clock()->now();
    // goal_pose_pub_->publish(msg);
}


void Controller::allowDrivingCallback(std_msgs::msg::Bool msg) {
    allow_driving_ = msg.data;
    RCLCPP_INFO(this->get_logger(), "allow_driving_ is %s", allow_driving_ ? "true" : "false");
}


void Controller::feedbackCallback(nav2_msgs::action::NavigateToPose_FeedbackMessage msg) {
    if (rotate_) {
        return;
    }
    static auto res = std_msgs::msg::Bool();
    res.data = true;
    feedback_ = msg;
    RCLCPP_INFO(this->get_logger(), "DELTA, %lf, time = %lf", feedback_.feedback.distance_remaining, this->get_clock()->now().seconds() - t_.seconds());
    if (feedback_.feedback.distance_remaining < delta_ && ((this->get_clock()->now().seconds() - t_.seconds()) > 1)) {
       cancelNavigation(true);
       start_rotate_ = true;
       return;
       result_pub_->publish(res);
    }
}


void Controller::odomCallback(nav_msgs::msg::Odometry msg) {
    odom_ = msg;
}


void Controller::cancelNavigation(bool status) {
    if (verbose_) {
        cancel_navigation_client_->async_cancel_all_goals();
        if (status) {
            RCLCPP_INFO(this->get_logger(), "goal_pose reached");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "goal_pose canceled");
        }
    }
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>()); 
    rclcpp::shutdown();
    return 0;
}
