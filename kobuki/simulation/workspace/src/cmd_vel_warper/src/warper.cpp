#include <memory>
#include <string>
#include <vector>
#include <math.h> 
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>


using namespace std::chrono_literals;


class Warper : public rclcpp::Node {
public:
    Warper();

private:

void sub_cb(const geometry_msgs::msg::Twist& msg);
void allow_cb(const std_msgs::msg::Bool& msg);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr allow_sub_;

    bool flag_ = true;
};


Warper::Warper() : Node("cmd_vel_warper") {
    using std::placeholders::_1;

    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "warper_cmd_vel", 10, std::bind(&Warper::sub_cb, this, _1));

    allow_sub_ = create_subscription<std_msgs::msg::Bool>(
        "allow_move", 10, std::bind(&Warper::allow_cb, this, _1));

    RCLCPP_INFO(this->get_logger(), "warper node created");
}

void Warper::allow_cb(const std_msgs::msg::Bool& msg) {
    flag_ = msg.data;
}


void Warper::sub_cb(const geometry_msgs::msg::Twist& msg) {
    if (!flag_) {
        return;
    }
    vel_pub_->publish(msg);
}


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Warper>());
  rclcpp::shutdown();
  return 0;
}
