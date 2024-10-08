#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>


using namespace std::chrono_literals;


class Dispatcher : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr master_allow_driving_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr slave_allow_driving_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr result_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool verbose_;
    bool started_ = false;
    size_t state_ = 0;

    std::vector<geometry_msgs::msg::PoseStamped> graph_;

public:
    Dispatcher();

private:
    void initCallback();

    void parsePoints(std::vector<double> points);
    geometry_msgs::msg::PoseStamped dropPoint();
    void feedbackCallback(std_msgs::msg::Bool msg);
};


Dispatcher::Dispatcher() : Node("dispatcher") {
    using std::placeholders::_1;

    this->declare_parameter("verbose", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("goal_pose_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("master_allow_driving_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("slave_allow_driving_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("result_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("points", rclcpp::PARAMETER_DOUBLE_ARRAY);

    this->declare_parameter("start", false);
    
    verbose_ = this->get_parameter("verbose").as_bool();
    std::string goal_pose_topic = this->get_parameter("goal_pose_topic").as_string();
    std::string master_allow_driving_topic = this->get_parameter("master_allow_driving_topic").as_string();
    std::string slave_allow_driving_topic = this->get_parameter("slave_allow_driving_topic").as_string();
    std::string result_topic = this->get_parameter("result_topic").as_string();
    std::vector<double> points = this->get_parameter("points").as_double_array();

    RCLCPP_INFO(this->get_logger(), "verbose: '%s'", verbose_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "goal_pose_topic: '%s'", goal_pose_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "master_allow_driving_topic: '%s'", master_allow_driving_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "slave_allow_driving_topic: '%s'", slave_allow_driving_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "result_topic: '%s'", result_topic.c_str());
    parsePoints(points);

    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic, 10);
    master_allow_driving_pub_ = this->create_publisher<std_msgs::msg::Bool>(master_allow_driving_topic, 10);
    slave_allow_driving_pub_ = this->create_publisher<std_msgs::msg::Bool>(slave_allow_driving_topic, 10);
    result_sub_ = this->create_subscription<std_msgs::msg::Bool>(result_topic, 10, std::bind(&Dispatcher::feedbackCallback, this, _1));

    timer_ = this->create_wall_timer(100ms, std::bind(&Dispatcher::initCallback, this));
}


void Dispatcher::initCallback() {
    if (!this->get_parameter("start").as_bool() || started_) {
            return;
    }

    started_ = true;
    auto msg = graph_[state_];
    goal_pose_pub_->publish(msg);
}


void Dispatcher::parsePoints(std::vector<double> points) {
    for (size_t i = 0; i < points.size(); i += 3) {
        RCLCPP_INFO(this->get_logger(), "point %ld: %lf %lf %lf", (i == 0 ? i : i - 1) / 3, points[i], points[i+1], points[i+2]);
        tf2::Quaternion q;
        q.setRPY(0, 0, points[i+2]);
        auto pt = geometry_msgs::msg::PoseStamped();
        pt.pose.position.x = points[i];
        pt.pose.position.y = points[i+1];
        pt.pose.orientation.x = q.getX();
        pt.pose.orientation.y = q.getY();
        pt.pose.orientation.z = q.getZ();
        pt.pose.orientation.w = q.getW();
        pt.header.frame_id = "map";
        graph_.push_back(pt);
    }
}


geometry_msgs::msg::PoseStamped Dispatcher::dropPoint() {
    state_ = state_ == graph_.size() - 1 ? 0 : state_ + 1;
    auto pt = graph_[state_];
    pt.header.stamp = this->get_clock()->now();
    return pt;
}


void Dispatcher::feedbackCallback(std_msgs::msg::Bool msg) {
    if (msg.data) {
        goal_pose_pub_->publish(dropPoint());
    }
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Dispatcher>()); 
    rclcpp::shutdown();
    return 0;
}
