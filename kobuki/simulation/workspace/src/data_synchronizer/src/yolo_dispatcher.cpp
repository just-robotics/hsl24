#pragma GCC target("avx")


#include <immintrin.h>

#include <iostream>
#include <memory>

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "dynamic_nav_interfaces/msg/detector_data.hpp"
#include "dynamic_nav_interfaces/msg/yolo_data.hpp"


using std::placeholders::_1;


class YoloDispatcher : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<dynamic_nav_interfaces::msg::YoloData>::SharedPtr yolo_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<dynamic_nav_interfaces::msg::DetectorData>::SharedPtr detector_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr real_mask_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    bool realsense_;
    bool verbose_;

    int check_input_t_ = 5; // sec
    rclcpp::Time yolo_t_;
    rclcpp::Time info_t_;

public:
    YoloDispatcher();

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {camera_info_ = msg; info_t_ = this->get_clock()->now();}
    void yoloCallback(dynamic_nav_interfaces::msg::YoloData::SharedPtr msg);
    void timerCallback();
};


YoloDispatcher::YoloDispatcher() : Node("yolo_dispatcher") {
    this->declare_parameter("camera_info_input_topic", "");
    this->declare_parameter("camera_info_topic", "");
    this->declare_parameter("yolo_topic", "");
    this->declare_parameter("rgb_topic", "");
    this->declare_parameter("depth_topic", "");
    this->declare_parameter("mask_topic", "");
    this->declare_parameter("odom_topic", "");
    this->declare_parameter("detector_topic", "");
    this->declare_parameter("real_mask_topic", "");
    this->declare_parameter("realsense", false);
    this->declare_parameter("verbose", true);

    std::string camera_info_input_topic = this->get_parameter("camera_info_input_topic").as_string();
    std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    std::string yolo_topic = this->get_parameter("yolo_topic").as_string();
    std::string rgb_topic = this->get_parameter("rgb_topic").as_string();
    std::string depth_topic = this->get_parameter("depth_topic").as_string();
    std::string mask_topic = this->get_parameter("mask_topic").as_string();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    std::string detector_topic = this->get_parameter("detector_topic").as_string();
    std::string real_mask_topic = this->get_parameter("real_mask_topic").as_string();
    realsense_ = this->get_parameter("realsense").as_bool();
    verbose_ = this->get_parameter("verbose").as_bool();

    RCLCPP_INFO(this->get_logger(), "camera_info_input_topic: '%s'", camera_info_input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_info_topic: '%s'", camera_info_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "yolo_topic: '%s'", yolo_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "rgb_topic: '%s'", rgb_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "depth_topic: '%s'", depth_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "mask_topic: '%s'", mask_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "odom_topic: '%s'", odom_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "detector_topic: '%s'", detector_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "real_mask_topic: '%s'", real_mask_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "realsense: '%s'", realsense_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "verbose: '%s'", verbose_ ? "true" : "false");

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_input_topic, 10, std::bind(&YoloDispatcher::cameraInfoCallback, this, _1));
    yolo_sub_ = this->create_subscription<dynamic_nav_interfaces::msg::YoloData>(yolo_topic, 10, std::bind(&YoloDispatcher::yoloCallback, this, _1));
    
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, 10);
    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(rgb_topic, 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_topic, 10);
    mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(mask_topic, 10);
    detector_pub_ = this->create_publisher<dynamic_nav_interfaces::msg::DetectorData>(detector_topic, 10);
    real_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(real_mask_topic, 10);
    if (!realsense_) {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    }

    timer_ = this->create_wall_timer(std::chrono::seconds(check_input_t_), std::bind(&YoloDispatcher::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "node started");

    yolo_t_ = this->get_clock()->now();
    info_t_ = this->get_clock()->now();
}


void YoloDispatcher::yoloCallback(dynamic_nav_interfaces::msg::YoloData::SharedPtr msg) {
    if (!camera_info_) {
        RCLCPP_WARN(this->get_logger(), "no camera_info");
        return;
    }

    auto start_timer = std::chrono::system_clock::now();
    auto dilate_s = std::chrono::system_clock::now();

    // dilate mask for rtabmap
    static const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    auto mask_dilated = cv_bridge::toCvCopy(msg->mask);
    for (size_t i = 0; i < msg->boxes.size(); i += 4) {
        int x0 = int(msg->boxes[i+0] - 10); x0 = x0 >= 0 ? x0 : 0;
        int y0 = int(msg->boxes[i+1] - 10); y0 = y0 >= 0 ? y0 : 0;
        int x1 = int(msg->boxes[i+2] + 10); x1 = x1 < mask_dilated->image.cols ? x1 : mask_dilated->image.cols;
        int y1 = int(msg->boxes[i+3] + 10); y1 = y1 < mask_dilated->image.rows ? y1 : mask_dilated->image.rows;
        cv::rectangle(mask_dilated->image, {x0, y0}, {x1, y1}, {255}, cv::FILLED);
    }
    cv::dilate(mask_dilated->image, mask_dilated->image, kernel, cv::Point(-1, -1), 2);

    auto dilate_e = std::chrono::system_clock::now();
    auto depth_s = std::chrono::system_clock::now();

    // filter depth for rtabmap
    // depth only if mask pixel is 0
    sensor_msgs::msg::Image static_depth = msg->depth;
    if (!realsense_) {
        static_depth = msg->depth; 
        uint8_t* mask_dilated_data = static_cast<uint8_t*>(mask_dilated->image.data); // get pointer to mask_dilated data
        float* static_depth_data = reinterpret_cast<float*>(static_depth.data.data()); // get pointer to static_depth data
        __m256i uint32_zeros = _mm256_set1_epi32(0);
        __m256 float_zeros = _mm256_set1_ps(0.0);
        for (size_t i = 0; i < msg->depth.height * msg->depth.width; i += 8) {
            __m128i reg_mask = _mm_loadu_si128((__m128i*)(mask_dilated_data + i)); // load mask to 8-bit integer avx
            __m256i reg_mask_epi32 = _mm256_cvtepu8_epi32(reg_mask); // convert 8-bit integer mask avx to 32-bit integer
            __m256i cmp_res = _mm256_cmpgt_epi32(reg_mask_epi32, uint32_zeros); // compare 32-bit integer mask avx with zeros
            _mm256_maskstore_ps(static_depth_data + i, cmp_res, float_zeros); // set static_depth to 0 if cmp_res is 1 (mask not zero)
            // explanation:
            // if (mask_dilated_data[i] != 0) {
            //     static_depth_data[i] = 0.0;
            // }
        }
    }
    else {
        cv::Mat float_depth = cv::Mat::zeros({int(msg->depth.width), int(msg->depth.height)}, CV_32FC1);
        uint8_t* mask_dilated_data = static_cast<uint8_t*>(mask_dilated->image.data); // get pointer to mask_dilated data
        uint16_t* static_depth_data = reinterpret_cast<uint16_t*>(msg->depth.data.data()); // get pointer to static_depth data
        float* float_depth_data = reinterpret_cast<float*>(float_depth.data); // get pointer to float_depth data
        __m256i uint32_zeros = _mm256_set1_epi16(0);
        __m256 scale = _mm256_set1_ps(0.001);
        for (size_t i = 0; i < msg->depth.height * msg->depth.width; i += 8) {
            __m128i reg_mask = _mm_loadu_si128((__m128i*)(mask_dilated_data + i)); // load mask to 8-bit uint avx
            __m128i reg_depth = _mm_loadu_si128((__m128i*)(static_depth_data + i)); // load depth to 16-bit uint avx
            __m256i reg_mask_epi32 = _mm256_cvtepu8_epi32(reg_mask); // convert 8-bit uint mask avx to 32-bit integer
            __m256i reg_depth_epi32 = _mm256_cvtepu16_epi32(reg_depth); // convert 16-bit uint depth avx to 32-bit integer
            __m256i reg_cmp_res = _mm256_cmpeq_epi32(reg_mask_epi32, uint32_zeros); // compare 32-bit integer mask avx with zeros
            reg_cmp_res = _mm256_srli_epi32(reg_cmp_res, 31); // -1 -> 1, 0 -> 0
            __m256 reg_cmp_res_float = _mm256_cvtepi32_ps(reg_cmp_res); // convert compare results from 32-bit integer to float
            __m256 reg_depth_float = _mm256_cvtepi32_ps(reg_depth_epi32); // convert 32-bit integer depth avx to float
            __m256 reg_static_depth = _mm256_mul_ps(reg_depth_float, reg_cmp_res_float); // if cmp is zero than depth is zero
            reg_static_depth = _mm256_mul_ps(reg_static_depth, scale); // set scale from uint16 to float (rep 118)
            _mm256_storeu_ps(float_depth_data + i, reg_static_depth); // store static depth
            // explanation:
            // if (mask_dilated_data[i] != 0) {
            //     static_depth_data[i] = 0.0;
            // }
        }
        static_depth = *(cv_bridge::CvImage(msg->depth.header, sensor_msgs::image_encodings::TYPE_32FC1, float_depth).toImageMsg());
    }

    auto depth_e = std::chrono::system_clock::now();
    auto pub_s = std::chrono::system_clock::now();

    // send rtabmap data
    camera_info_->header.stamp = msg->rgb.header.stamp;
    static_depth.header.stamp = msg->rgb.header.stamp;
    mask_dilated->header.stamp = msg->rgb.header.stamp;
    
    camera_info_pub_->publish(*camera_info_);
    rgb_pub_->publish(msg->rgb);
    depth_pub_->publish(static_depth);
    mask_pub_->publish(*mask_dilated->toImageMsg());
    real_mask_pub_->publish(msg->mask);
    if (!realsense_) {
        msg->odom.header.stamp = msg->rgb.header.stamp;
        odom_pub_->publish(msg->odom);
    }
    
    // send obstacles_tracker data
    auto detector_data = dynamic_nav_interfaces::msg::DetectorData();
    detector_data.camera_info = *camera_info_;
    detector_data.rgb = msg->rgb;
    detector_data.depth = msg->depth;
    detector_data.mask = msg->mask;
    detector_data.boxes = msg->boxes;

    detector_pub_->publish(detector_data);

    auto pub_e = std::chrono::system_clock::now();
    auto end_timer = std::chrono::system_clock::now();
    
    auto dt_dilate = std::chrono::duration_cast<std::chrono::milliseconds>(dilate_e - dilate_s).count();
    auto dt_depth = std::chrono::duration_cast<std::chrono::milliseconds>(depth_e - depth_s).count();
    auto dt_pub = std::chrono::duration_cast<std::chrono::milliseconds>(pub_e - pub_s).count();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count();
    
    if (verbose_) {
        RCLCPP_INFO(this->get_logger(), "dt = %ldms; dt_dilate = %ldms; dt_depth = %ldms; dt_pub = %ldms;", dt, dt_dilate, dt_depth, dt_pub);
    }
    yolo_t_ = this->get_clock()->now();
}


void YoloDispatcher::timerCallback() {
    auto t = this->get_clock()->now().seconds();
    if (t - yolo_t_.seconds() > check_input_t_) {
        RCLCPP_WARN(this->get_logger(), "no data from yolo received");
    }
    if (t - info_t_.seconds() > check_input_t_) {
        RCLCPP_WARN(this->get_logger(), "no camera_info received");
    }
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YoloDispatcher>()); 
    rclcpp::shutdown();
    return 0;
}
