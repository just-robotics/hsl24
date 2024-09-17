#include <chrono>
#include <iostream>
#include <memory>

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>


class FakeRealSense : public rclcpp::Node {
private:
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::TimerBase::SharedPtr image_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;

    bool verbose_;

    sensor_msgs::msg::CameraInfo camera_info_;
    sensor_msgs::msg::Image rgb_;
    sensor_msgs::msg::Image depth_;
    sensor_msgs::msg::Imu imu_;

public:
    FakeRealSense();

private:
    void imageCallback();
    void imuCallback();
};


FakeRealSense::FakeRealSense() : Node("fake_realsense") {
    this->declare_parameter("verbose", true);
    this->declare_parameter("camera_info_topic", "");
    this->declare_parameter("rgb_topic", "");
    this->declare_parameter("depth_topic", "");
    this->declare_parameter("imu_topic", "");
    this->declare_parameter("framerate", 0);
    this->declare_parameter("imu_rate", 0);

    verbose_ = this->get_parameter("verbose").as_bool();
    std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    std::string rgb_topic = this->get_parameter("rgb_topic").as_string();
    std::string depth_topic = this->get_parameter("depth_topic").as_string();
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    int framerate = this->get_parameter("framerate").as_int();
    int imu_rate = this->get_parameter("imu_rate").as_int();

    if (framerate <= 0 ) {
        RCLCPP_ERROR(this->get_logger(), "wrong framerate: '%d'", framerate);
        exit(1);
    }
    if (imu_rate <= 0 ) {
        RCLCPP_ERROR(this->get_logger(), "wrong imu_rate: '%d'", imu_rate);
        exit(1);
    }

    RCLCPP_INFO(this->get_logger(), "verbose: '%s'", (verbose_ ? "true" : "false"));
    RCLCPP_INFO(this->get_logger(), "camera_info_topic: '%s'", camera_info_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "rgb_topic: '%s'", rgb_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "depth_topic: '%s'", depth_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic: '%s'", imu_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "framerate: '%d'", framerate);
    RCLCPP_INFO(this->get_logger(), "imu_rate: '%d'", imu_rate);
    
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, 10);
    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(rgb_topic, 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_topic, 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);

    image_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / framerate)), std::bind(&FakeRealSense::imageCallback, this));
    imu_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / imu_rate)), std::bind(&FakeRealSense::imuCallback, this));

    camera_info_ = sensor_msgs::msg::CameraInfo();
    rgb_ = *cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, cv::Mat::zeros({640, 480}, CV_8UC3)).toImageMsg();
    depth_ = *cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, cv::Mat::zeros({640, 480}, CV_16UC1)).toImageMsg();
    imu_ = sensor_msgs::msg::Imu();

    rgb_.header.frame_id = "camera_link_optical";
    rgb_.header.frame_id = "camera_link_optical";
    depth_.header.frame_id = "camera_link_optical";
    imu_.header.frame_id = "camera_imu_optical_frame";

    RCLCPP_INFO(this->get_logger(), "node started");
}


void FakeRealSense::imageCallback() {
    camera_info_.header.stamp = this->get_clock()->now();
    rgb_.header.stamp = this->get_clock()->now();
    depth_.header.stamp = this->get_clock()->now();

    rgb_pub_->publish(rgb_);
    depth_pub_->publish(depth_);
    
    if (verbose_) {
        RCLCPP_INFO(this->get_logger(), "publishing rgb & depth at %u.%u", rgb_.header.stamp.sec, rgb_.header.stamp.nanosec);
    }
}


void FakeRealSense::imuCallback() {
    imu_.header.stamp = this->get_clock()->now();

    imu_pub_->publish(imu_);
    
    if (verbose_) {
        RCLCPP_INFO(this->get_logger(), "publishing imu at %u.%u", imu_.header.stamp.sec, imu_.header.stamp.nanosec);
    }
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeRealSense>()); 
    rclcpp::shutdown();
    return 0;
}
