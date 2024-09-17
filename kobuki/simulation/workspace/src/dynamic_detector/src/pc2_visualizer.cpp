#pragma GCC target("avx")


#include <immintrin.h>

#include <chrono>
#include <iostream>
#include <memory>

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "dynamic_nav_interfaces/msg/detector_data.hpp"


class Visualizer : public rclcpp::Node {
private:
    rclcpp::Subscription<dynamic_nav_interfaces::msg::DetectorData>::SharedPtr input_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_pub_;

    image_geometry::PinholeCameraModel camera_model_;
    float range_max_;
    std::string camera_link_optical_frame_;

public:
    Visualizer();

private:
    cv::Mat extractDynamicDepth(const cv::Mat& depth, cv::Mat& mask);
    void depth2pc2(const sensor_msgs::msg::Image& depth_msg, sensor_msgs::msg::PointCloud2::SharedPtr point_cloud);

    void callback(dynamic_nav_interfaces::msg::DetectorData::SharedPtr msg);
};


Visualizer::Visualizer() : Node("pc2_visualizer") {
    using std::placeholders::_1;

    this->declare_parameter("input_topic", "");
    this->declare_parameter("pc2_topic", "");
    this->declare_parameter("range_max", 0.0);
    this->declare_parameter("camera_link_optical_frame", "");

    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string pc2_topic = this->get_parameter("pc2_topic").as_string();
    range_max_ = this->get_parameter("range_max").as_double();
    camera_link_optical_frame_ = this->get_parameter("camera_link_optical_frame").as_string();

    RCLCPP_INFO(this->get_logger(), "input_topic: '%s'", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "pc2_topic: '%s'", pc2_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "range_max: '%f'", range_max_);
    RCLCPP_INFO(this->get_logger(), "camera_link_optical_frame: '%s'", camera_link_optical_frame_.c_str());


    input_sub_ = this->create_subscription<dynamic_nav_interfaces::msg::DetectorData>(input_topic, 1, std::bind(&Visualizer::callback, this, _1));
    pc2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pc2_topic, 1);
}


cv::Mat Visualizer::extractDynamicDepth(const cv::Mat& depth, cv::Mat& mask) {
    // erode mask
    static const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat mask_eroded;
    cv::erode(mask, mask_eroded, kernel, cv::Point(-1, -1), 1);

    // extract dynamic_depth only
    cv::Mat dynamic_depth = depth.clone(); // depth only if mask pixel is 0
    uint8_t* mask_data = static_cast<uint8_t*>(mask.data); // get pointer to mask_dilated data
    float* dynamic_depth_data = reinterpret_cast<float*>(dynamic_depth.data); // get pointer to dynamic_depth data
    __m256i uint32_zeros = _mm256_set1_epi32(0);
    __m256 float_zeros = _mm256_set1_ps(0.0);
    for (int i = 0; i < dynamic_depth.rows * dynamic_depth.cols; i += 8) {
        __m128i reg_mask = _mm_loadu_si128((__m128i*)(mask_data + i)); // load mask to 8-bit integer avx
        __m256i reg_mask_epi32 = _mm256_cvtepu8_epi32(reg_mask); // convert 8-bit integer mask avx to 32-bit integer
        __m256i cmp_res = _mm256_cmpeq_epi32(reg_mask_epi32, uint32_zeros); // compare 32-bit integer mask avx with zeros
        _mm256_maskstore_ps(dynamic_depth_data + i, cmp_res, float_zeros); // set static_depth to 0 if cmp_res is 0 (mask is zero)
        // explanation:
        // if (mask_data[i] == 0) {
        //     dynamic_depth_data[i] = 0.0;
        // }
    }

    return dynamic_depth;
}


void Visualizer::depth2pc2(const sensor_msgs::msg::Image& depth_msg, sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) {
    point_cloud->header.frame_id = camera_link_optical_frame_;
    point_cloud->header.stamp = this->get_clock()->now();
    point_cloud->height = depth_msg.height;
    point_cloud->width = depth_msg.width;
    point_cloud->is_dense = false;
    point_cloud->is_bigendian = false;
    point_cloud->fields.clear();
    point_cloud->fields.reserve(1);

    sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    // Use correct principal point from calibration
    float center_x = camera_model_.cx();
    float center_y = camera_model_.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    float constant_x = 1 / camera_model_.fx();
    float constant_y = 1 / camera_model_.fy();

    sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");
    const float* depth_row = reinterpret_cast<const float*>(&depth_msg.data[0]);
    int row_step = depth_msg.step / sizeof(float);

    for (int v = 0; v < static_cast<int>(point_cloud->height); ++v, depth_row += row_step) {
        for (int u = 0; u < static_cast<int>(point_cloud->width); ++u, ++iter_x, ++iter_y, ++iter_z) {
            float depth = depth_row[u];

            if (!std::isfinite(depth) || depth > range_max_) {
                depth = range_max_;
            }

            *iter_x = (u - center_x) * depth * constant_x;
            *iter_y = (v - center_y) * depth * constant_y;
            *iter_z = depth;
        }
    }
}


void Visualizer::callback(dynamic_nav_interfaces::msg::DetectorData::SharedPtr msg) {
    auto start_timer = std::chrono::system_clock::now();

    cv::Mat depth = cv_bridge::toCvCopy(msg->depth)->image;
    cv::Mat mask = cv_bridge::toCvCopy(msg->mask)->image;
    camera_model_.fromCameraInfo(msg->camera_info);

    cv::Mat dynamic_depth = extractDynamicDepth(depth, mask);
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    std::memcpy(msg->depth.data.data(), dynamic_depth.data, sizeof(float) * dynamic_depth.cols * dynamic_depth.rows);
    depth2pc2(msg->depth, point_cloud);
    pc2_pub_->publish(*point_cloud);

    auto end_timer = std::chrono::system_clock::now();

    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count();
    
    RCLCPP_INFO(this->get_logger(), "dt = %ld;", dt);
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Visualizer>()); 
    rclcpp::shutdown();
    return 0;
}
