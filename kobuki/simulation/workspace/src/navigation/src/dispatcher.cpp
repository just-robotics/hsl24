#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>


using namespace std::chrono_literals;


class Dispatcher : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr master_allow_driving_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr slave_allow_driving_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool verbose_;
    bool started_ = false;
    size_t state_ = 0;

    std::vector<geometry_msgs::msg::PoseStamped> graph_;

public:
    Dispatcher();

private:
    void initCallback();
};


Dispatcher::Dispatcher() : Node("dispatcher") {
    using std::placeholders::_1;

    this->declare_parameter("verbose", true);
    this->declare_parameter("goal_pose_topic", "");
    this->declare_parameter("master_allow_driving_topic", "");
    this->declare_parameter("slave_allow_driving_topic", "");
    
    verbose_ = this->get_parameter("verbose").as_bool();
    std::string goal_pose_topic = this->get_parameter("goal_pose_topic").as_string();
    std::string master_allow_driving_topic = this->get_parameter("master_allow_driving_topic").as_string();
    std::string slave_allow_driving_topic = this->get_parameter("slave_allow_driving_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "verbose: '%s'", verbose_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "goal_pose_topic: '%s'", goal_pose_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "master_allow_driving_topic: '%s'", master_allow_driving_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "slave_allow_driving_topic: '%s'", slave_allow_driving_topic.c_str());

    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic, 10);
    master_allow_driving_pub_ = this->create_publisher<std_msgs::msg::Bool>(master_allow_driving_topic, 10);
    slave_allow_driving_pub_ = this->create_publisher<std_msgs::msg::Bool>(slave_allow_driving_topic, 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&Dispatcher::initCallback, this));
}


void Dispatcher::initCallback() {
    if (!this->get_parameter("start").as_bool() || started_) {
            return;
    }

    started_ = true;
    auto msg = graph_[0];
    goal_pose_pub_->publish(msg);
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Dispatcher>()); 
    rclcpp::shutdown();
    return 0;
}
