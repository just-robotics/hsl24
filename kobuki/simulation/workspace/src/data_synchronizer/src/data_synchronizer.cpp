#include <chrono>
#include <iostream>
#include <memory>

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "dynamic_nav_interfaces/msg/yolo_data.hpp"


class DataSyncronizer : public rclcpp::Node {
private:
    using odom_policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, nav_msgs::msg::Odometry>;
    using realsense_policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    
    std::unique_ptr<message_filters::Synchronizer<odom_policy>> odom_sync_;
    std::unique_ptr<message_filters::Synchronizer<realsense_policy>> realsense_sync_;

    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;

    rclcpp::Publisher<dynamic_nav_interfaces::msg::YoloData>::SharedPtr output_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_vis_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool verbose_;
    int check_input_t_ = 5;
    rclcpp::Time t_;

public:
    DataSyncronizer();

private:
    void odomCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr rgb,
        const sensor_msgs::msg::Image::ConstSharedPtr depth,
        const nav_msgs::msg::Odometry::ConstSharedPtr odom
    );
    void cameraCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr rgb,
        const sensor_msgs::msg::Image::ConstSharedPtr depth
    );
    void timerCallback();
};


DataSyncronizer::DataSyncronizer() : Node("data_synchronizer") {
    this->declare_parameter("verbose", true);
    this->declare_parameter("rgb_topic", "");
    this->declare_parameter("depth_topic", "");
    this->declare_parameter("odom_topic", "");
    this->declare_parameter("output_topic", "");
    this->declare_parameter("rgb_vis_topic", "");
    this->declare_parameter("realsense", false);

    verbose_ = this->get_parameter("verbose").as_bool();
    std::string rgb_topic = this->get_parameter("rgb_topic").as_string();
    std::string depth_topic = this->get_parameter("depth_topic").as_string();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    std::string rgb_vis_topic = this->get_parameter("rgb_vis_topic").as_string();
    bool realsense = this->get_parameter("realsense").as_bool();

    RCLCPP_INFO(this->get_logger(), "verbose: '%s'", (verbose_ ? "true" : "false"));
    RCLCPP_INFO(this->get_logger(), "rgb_topic: '%s'", rgb_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "depth_topic: '%s'", depth_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "odom_topic: '%s'", odom_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "output_topic: '%s'", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "rgb_vis_topic: '%s'", rgb_vis_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "realsense: '%s'", (realsense ? "true" : "false"));

    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = 10;
    qos.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    qos.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE;

    rgb_sub_.subscribe(this, rgb_topic, qos);
    depth_sub_.subscribe(this, depth_topic, qos);

    if (!realsense) {
        odom_sub_.subscribe(this, odom_topic);
        odom_sync_.reset(new message_filters::Synchronizer<odom_policy>(odom_policy(10), rgb_sub_, depth_sub_, odom_sub_));
        odom_sync_->registerCallback(std::bind(&DataSyncronizer::odomCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        RCLCPP_INFO(this->get_logger(), "setting odom callback");
    }
    else {
        realsense_sync_.reset(new message_filters::Synchronizer<realsense_policy>(realsense_policy(10), rgb_sub_, depth_sub_));
        realsense_sync_->registerCallback(std::bind(&DataSyncronizer::cameraCallback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "setting realsense callback");
    }
    
    output_pub_ = this->create_publisher<dynamic_nav_interfaces::msg::YoloData>(output_topic, 10);
    rgb_vis_pub_ = this->create_publisher<sensor_msgs::msg::Image>(rgb_vis_topic, 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(check_input_t_), std::bind(&DataSyncronizer::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "node started");

    t_ = this->get_clock()->now();
}


void DataSyncronizer::odomCallback(const sensor_msgs::msg::Image::ConstSharedPtr rgb,
                               const sensor_msgs::msg::Image::ConstSharedPtr depth, const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
    auto start_timer = std::chrono::system_clock::now();

    auto msg = dynamic_nav_interfaces::msg::YoloData();
    msg.rgb = *rgb;
    msg.depth = *depth;
    msg.odom = *odom;

    output_pub_->publish(msg);
    rgb_vis_pub_->publish(msg.rgb);

    auto end_timer = std::chrono::system_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count();
    
    if (verbose_) {
        RCLCPP_INFO(this->get_logger(), "sync with odom created: %u:%u %u:%u %u:%u; dt = %ldms;", msg.rgb.header.stamp.sec, msg.rgb.header.stamp.nanosec,
        msg.depth.header.stamp.sec, msg.depth.header.stamp.nanosec, msg.odom.header.stamp.sec, msg.odom.header.stamp.nanosec, dt);
    }

    t_ = this->get_clock()->now();
}


void DataSyncronizer::cameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr rgb, const sensor_msgs::msg::Image::ConstSharedPtr depth) {  
    auto start_timer = std::chrono::system_clock::now();

    auto msg = dynamic_nav_interfaces::msg::YoloData();
    msg.rgb = *rgb;
    msg.depth = *depth;
    msg.odom = nav_msgs::msg::Odometry();

    output_pub_->publish(msg);
    rgb_vis_pub_->publish(msg.rgb);

    auto end_timer = std::chrono::system_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count();
    
    if (verbose_) {
        RCLCPP_INFO(this->get_logger(), "realsense sync created: %u:%u %u:%u; dt = %ldms;", msg.rgb.header.stamp.sec, msg.rgb.header.stamp.nanosec,
        msg.depth.header.stamp.sec, msg.depth.header.stamp.nanosec, dt);
    }

    t_ = this->get_clock()->now();
}


void DataSyncronizer::timerCallback() {
    if (this->get_clock()->now().seconds() - t_.seconds() > check_input_t_) {
        RCLCPP_WARN(this->get_logger(), "no data received");
    }
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataSyncronizer>()); 
    rclcpp::shutdown();
    return 0;
}
