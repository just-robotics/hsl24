#pragma GCC target("avx")


#include <immintrin.h>

#include <chrono>
#include <iostream>
#include <memory>

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>

#include<cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "dynamic_nav_interfaces/msg/obstacles_footprints.hpp"
#include "dynamic_nav_interfaces/msg/detector_data.hpp"


class DynamicDetector : public rclcpp::Node {
private:
    rclcpp::Subscription<dynamic_nav_interfaces::msg::DetectorData>::SharedPtr input_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr danger_dist_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr box_pub_;
    rclcpp::Publisher<dynamic_nav_interfaces::msg::ObstaclesFootprints>::SharedPtr obstacles_fp_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bb_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dyn_depth_pub_;

    image_geometry::PinholeCameraModel camera_model_;
    float range_max_;
    std::string camera_link_optical_frame_;
    bool realsense_;
    bool verbose_;
    const cv::Scalar red_ = {0, 0, 255};
    const cv::Scalar magenta_ = {255, 0, 255};
    cv::Scalar bb_color_;
    bool use_lifetime_;

    std::vector<geometry_msgs::msg::Point> box_points_;

public:
    DynamicDetector();

private:
    std::vector<std::vector<cv::Point>> setImagePoints(std::vector<uint64_t>& boxes);
    cv::Mat extractDynamicDepth(const cv::Mat& depth, cv::Mat& mask, std::vector<std::vector<cv::Point>>& image_points);
    void depth2pc2(const sensor_msgs::msg::Image& depth_msg, sensor_msgs::msg::PointCloud2::SharedPtr point_cloud);
    geometry_msgs::msg::Point quickConvert(const cv::Mat& depth, size_t x, size_t y, float median_d = -1);
    float calcMedianDepth(cv::Mat& dynamic_depth, cv::Point ul, cv::Point lr);
    std::vector<std::vector<geometry_msgs::msg::Point>> getRealPoints(std::vector<std::vector<cv::Point>>& image_points, cv::Mat& dynamic_depth);
    std::vector<std::vector<geometry_msgs::msg::Point>> setRealBox(std::vector<std::vector<geometry_msgs::msg::Point>>& real_points);
    double calcUpperY(geometry_msgs::msg::Point lower_point, cv::Point image_lower_point);
    void createLine(visualization_msgs::msg::Marker& line_list, geometry_msgs::msg::Point pt0, geometry_msgs::msg::Point pt1);

    visualization_msgs::msg::Marker createBoxMsg(std::vector<std::vector<geometry_msgs::msg::Point>>& real_points, std::vector<std::vector<cv::Point>>& image_points);

    void drawBoxes(cv::Mat& image, std::vector<uint64_t>& boxes);

    void callback(dynamic_nav_interfaces::msg::DetectorData::SharedPtr msg);
    void dangerDistanceCallback(const std_msgs::msg::Bool msg);
};


DynamicDetector::DynamicDetector() : Node("dynamic_detector") {
    using std::placeholders::_1;

    this->declare_parameter("verbose", true);
    this->declare_parameter("input_topic", "");
    this->declare_parameter("pc2_topic", "");
    this->declare_parameter("box_topic", "");
    this->declare_parameter("obstacles_fp_topic", "");
    this->declare_parameter("bb_image_topic", "");
    this->declare_parameter("dyn_depth_topic", "");
    this->declare_parameter("danger_distance_topic", "");
    this->declare_parameter("range_max", 0.0);
    this->declare_parameter("camera_link_optical_frame", "");
    this->declare_parameter("realsense", false);
    this->declare_parameter("use_lifetime", false);

    verbose_ = this->get_parameter("verbose").as_bool();
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string pc2_topic = this->get_parameter("pc2_topic").as_string();
    std::string box_topic = this->get_parameter("box_topic").as_string();
    std::string obstacles_fp_topic = this->get_parameter("obstacles_fp_topic").as_string();
    std::string bb_image_topic = this->get_parameter("bb_image_topic").as_string();
    std::string dyn_depth_topic = this->get_parameter("dyn_depth_topic").as_string();
    std::string danger_distance_topic = this->get_parameter("danger_distance_topic").as_string();
    range_max_ = this->get_parameter("range_max").as_double();
    camera_link_optical_frame_ = this->get_parameter("camera_link_optical_frame").as_string();
    realsense_ = this->get_parameter("realsense").as_bool();
    use_lifetime_ = this->get_parameter("use_lifetime").as_bool();

    RCLCPP_INFO(this->get_logger(), "verbose: '%s'", verbose_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "input_topic: '%s'", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "pc2_topic: '%s'", pc2_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "box_topic: '%s'", box_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "obstacles_fp_topic: '%s'", obstacles_fp_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "bb_image_topic: '%s'", bb_image_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "dyn_depth_topic: '%s'", dyn_depth_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "danger_distance_topic: '%s'", danger_distance_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "range_max: '%f'", range_max_);
    RCLCPP_INFO(this->get_logger(), "camera_link_optical_frame: '%s'", camera_link_optical_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "realsense: '%s'", realsense_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "use_lifetime: '%s'", use_lifetime_ ? "true" : "false");

    input_sub_ = this->create_subscription<dynamic_nav_interfaces::msg::DetectorData>(input_topic, 10, std::bind(&DynamicDetector::callback, this, _1));
    danger_dist_sub_ = this->create_subscription<std_msgs::msg::Bool>(danger_distance_topic, 10, std::bind(&DynamicDetector::dangerDistanceCallback, this, _1));
    pc2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pc2_topic, 1);
    box_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(box_topic, 1);
    obstacles_fp_pub_ = this->create_publisher<dynamic_nav_interfaces::msg::ObstaclesFootprints>(obstacles_fp_topic, 1);
    bb_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(bb_image_topic, 1);
    dyn_depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(dyn_depth_topic, 1);

    bb_color_ = magenta_;
}


std::vector<std::vector<cv::Point>> DynamicDetector::setImagePoints(std::vector<uint64_t>& boxes) {
    std::vector<std::vector<cv::Point>> points;
    for (size_t i = 0; i < boxes.size(); i += 4) {
        std::vector<cv::Point> pts;
        cv::Point ulf {int(boxes[i+0]), int(boxes[i+1])};
        cv::Point urf {int(boxes[i+2]), int(boxes[i+1])};
        cv::Point llf {int(boxes[i+0]), int(boxes[i+3])};
        cv::Point lrf {int(boxes[i+2]), int(boxes[i+3])};
        pts.push_back(ulf);
        pts.push_back(urf);
        pts.push_back(llf);
        pts.push_back(lrf);
        points.push_back(pts);
    }
    return points;
}


cv::Mat DynamicDetector::extractDynamicDepth(const cv::Mat& depth, cv::Mat& mask, std::vector<std::vector<cv::Point>>& image_points) {
    // erode mask
    static const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat mask_eroded;
    cv::erode(mask, mask_eroded, kernel, cv::Point(-1, -1), 1);

    // extract dynamic_depth only
    cv::Mat dynamic_depth;
    if (!realsense_) {
        dynamic_depth = depth.clone(); // depth only if mask pixel is 0
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
    }
    else {
        dynamic_depth = cv::Mat::zeros({int(depth.cols), int(depth.rows)}, CV_32FC1);
        uint8_t* mask_dilated_data = static_cast<uint8_t*>(mask.data); // get pointer to mask_dilated data
        uint16_t* realsense_depth_data = reinterpret_cast<uint16_t*>(depth.data); // get pointer to static_depth data
        float* float_depth_data = reinterpret_cast<float*>(dynamic_depth.data); // get pointer to float_depth data
        __m256i uint32_zeros = _mm256_set1_epi16(0);
        __m256 scale = _mm256_set1_ps(0.001);
        for (int i = 0; i < depth.rows * depth.cols; i += 8) {
            __m128i reg_mask = _mm_loadu_si128((__m128i*)(mask_dilated_data + i)); // load mask to 8-bit uint avx
            __m128i reg_depth = _mm_loadu_si128((__m128i*)(realsense_depth_data + i)); // load depth to 16-bit uint avx
            __m256i reg_mask_epi32 = _mm256_cvtepu8_epi32(reg_mask); // convert 8-bit uint mask avx to 32-bit integer
            __m256i reg_depth_epi32 = _mm256_cvtepu16_epi32(reg_depth); // convert 16-bit uint depth avx to 32-bit integer
            __m256i reg_cmp_res = _mm256_cmpgt_epi32(reg_mask_epi32, uint32_zeros); // compare 32-bit integer mask avx with zeros
            reg_cmp_res = _mm256_srli_epi32(reg_cmp_res, 31); // -1 -> 1, 0 -> 0
            __m256 reg_cmp_res_float = _mm256_cvtepi32_ps(reg_cmp_res); // convert compare results from 32-bit integer to float
            __m256 reg_depth_float = _mm256_cvtepi32_ps(reg_depth_epi32); // convert 32-bit integer depth avx to float
            __m256 reg_static_depth = _mm256_mul_ps(reg_depth_float, reg_cmp_res_float); // if cmp is zero than depth is zero
            reg_static_depth = _mm256_mul_ps(reg_static_depth, scale); // set scale from uint16 to float (rep 118)
            _mm256_storeu_ps(float_depth_data + i, reg_static_depth); // store static depth
            // explanation:
            // if (mask_data[i] != 0) {
            //     dynamic_depth_data[i] = 0.0;
            // }
        }
    }

    for (size_t i = 0; i < image_points.size(); i++) {
        std::vector<cv::Point> image_pts = image_points[i];
        for (size_t j = 0; j < image_pts.size(); j++) {
            dynamic_depth.at<float>(image_pts[j].y, image_pts[j].x) = depth.at<float>(image_pts[j].y, image_pts[j].x);
        }
    }

    return dynamic_depth;
}


void DynamicDetector::depth2pc2(const sensor_msgs::msg::Image& depth_msg, sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) {
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
    std::vector<geometry_msgs::msg::Point> xyz;

    for (int v = 0; v < static_cast<int>(point_cloud->height); ++v, depth_row += row_step) {
        for (int u = 0; u < static_cast<int>(point_cloud->width); ++u, ++iter_x, ++iter_y, ++iter_z) {
            float depth = depth_row[u];

            if (!std::isfinite(depth) || depth > range_max_) {
                depth = range_max_;
            }

            *iter_x = (u - center_x) * depth * constant_x;
            *iter_y = (v - center_y) * depth * constant_y;
            *iter_z = depth;

            geometry_msgs::msg::Point pt;
            pt.x = *iter_x;
            pt.y = *iter_y;
            pt.z = *iter_z;
            xyz.push_back(pt);
        }
    }
}


geometry_msgs::msg::Point DynamicDetector::quickConvert(const cv::Mat& depth, size_t x, size_t y, float median_d) {
    float d = median_d < 0 ? depth.at<float>(y, x) : median_d;

    if (!std::isfinite(d) || d > range_max_) {
        d = range_max_;
    }

    geometry_msgs::msg::Point pt;

    pt.x = (x - camera_model_.cx()) * d / camera_model_.fx();
    pt.y = (y - camera_model_.cy()) * d / camera_model_.fy();
    pt.z = d;

    return pt;
}


float DynamicDetector::calcMedianDepth(cv::Mat& dynamic_depth, cv::Point ul, cv::Point lr) {
    std::vector<float> array;

    for (int y = ul.y; y < lr.y; y += 2) {
        for (int x = ul.x; x < lr.x; x += 2) {
            float d = dynamic_depth.at<float>(y, x);
            if (d != 0) {
                array.push_back(d);
            }
        }
    }

    std::sort(array.begin(), array.end());
    size_t n = array.size();
    if (n % 2 == 0) {
        return (array[n / 2 - 1] + array[n / 2]) / 2.0;
    }
    return array[n / 2];
}


std::vector<std::vector<geometry_msgs::msg::Point>> DynamicDetector::getRealPoints(std::vector<std::vector<cv::Point>>& image_points, cv::Mat& dynamic_depth) {
    std::vector<std::vector<geometry_msgs::msg::Point>> real_points;
    for (size_t i = 0; i < image_points.size(); i++) {
        std::vector<cv::Point> image_pts = image_points[i];
        std::vector<geometry_msgs::msg::Point> real_pts;
        float median_d = calcMedianDepth(dynamic_depth, image_pts[0], image_pts[3]);
        for (size_t j = 0; j < image_pts.size(); j++) {
            geometry_msgs::msg::Point real_pt = quickConvert(dynamic_depth, image_pts[j].x, image_pts[j].y, median_d);
            real_pts.push_back(real_pt);
        }
        real_points.push_back(real_pts);
    }
    return real_points;
}


std::vector<std::vector<geometry_msgs::msg::Point>> DynamicDetector::setRealBox(std::vector<std::vector<geometry_msgs::msg::Point>>& real_points) {
    std::vector<std::vector<geometry_msgs::msg::Point>> all_points;
    for (size_t i = 0; i < real_points.size(); i++) {
        std::vector<geometry_msgs::msg::Point> real_pts = real_points[i];
        float l = real_pts[3].x - real_pts[2].x;
        std::vector<geometry_msgs::msg::Point> all_pts = real_pts;
        for (size_t j = 0; j < real_pts.size(); j++) {
            all_pts[j].z -= 0.5 * l;
            geometry_msgs::msg::Point pt = real_pts[j];
            pt.z += l;
            all_pts.push_back(pt);
        }
        all_points.push_back(all_pts);
    }
    return all_points;
}


inline double DynamicDetector::calcUpperY(geometry_msgs::msg::Point lower_point, cv::Point image_upper_point) {
   return (image_upper_point.y - camera_model_.cy()) * lower_point.z / camera_model_.fy();
}


inline void DynamicDetector::createLine(visualization_msgs::msg::Marker& line_list, geometry_msgs::msg::Point pt0, geometry_msgs::msg::Point pt1) {
    line_list.points.push_back(pt0);
    line_list.points.push_back(pt1);
}


visualization_msgs::msg::Marker DynamicDetector::createBoxMsg(std::vector<std::vector<geometry_msgs::msg::Point>>& real_points,
                                        std::vector<std::vector<cv::Point>>& image_points) {
    visualization_msgs::msg::Marker line_list;
    line_list.header.frame_id = camera_link_optical_frame_;
    line_list.header.stamp = this->get_clock()->now();
    line_list.id = 2;
    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_list.scale.x = 0.01;
    line_list.color.b = 1.0; // blue
    line_list.color.r = 1.0; // red
    line_list.color.a = 1.0; // alpha
    if (!use_lifetime_) {
        line_list.lifetime.nanosec = 40 /* ms */ * 10e6;
    }

    for (size_t i = 0; i < real_points.size(); i++) {
        std::vector<geometry_msgs::msg::Point> real_pts = real_points[i];
        std::vector<cv::Point> image_pts = image_points[i];

        geometry_msgs::msg::Point ulf = real_pts[0];
        geometry_msgs::msg::Point urf = real_pts[1];
        geometry_msgs::msg::Point llf = real_pts[2];
        geometry_msgs::msg::Point lrf = real_pts[3];
        geometry_msgs::msg::Point ulb = real_pts[4];
        geometry_msgs::msg::Point urb = real_pts[5];
        geometry_msgs::msg::Point llb = real_pts[6];
        geometry_msgs::msg::Point lrb = real_pts[7];

        ulb.x = llb.x; ulb.z = llb.z; ulb.y = calcUpperY(llb, image_pts[0]);
        urb.x = lrb.x; urb.z = lrb.z; urb.y = calcUpperY(lrb, image_pts[1]);
        ulf.x = llf.x; ulf.z = llf.z; ulf.y = ulb.y;
        urf.x = lrf.x; urf.z = lrf.z; urf.y = urb.y;

        box_points_.push_back(llf);
        box_points_.push_back(lrf);
        box_points_.push_back(llb);
        box_points_.push_back(lrb);

        createLine(line_list, ulf, urf);
        createLine(line_list, ulf, llf);
        createLine(line_list, llf, lrf);
        createLine(line_list, urf, lrf);
        
        createLine(line_list, ulb, urb);
        createLine(line_list, ulb, llb);
        createLine(line_list, llb, lrb);
        createLine(line_list, urb, lrb);

        createLine(line_list, ulf, ulb);
        createLine(line_list, urf, urb);
        createLine(line_list, llf, llb);
        createLine(line_list, lrf, lrb);
    }
    
    return line_list;
}


void DynamicDetector::drawBoxes(cv::Mat& image, std::vector<uint64_t>& boxes) {
    for (size_t i = 0; i < boxes.size(); i += 4) {
        cv::rectangle(image, cv::Rect(cv::Point{int(boxes[i]), int(boxes[i+1])}, cv::Point{int(boxes[i+2]), int(boxes[i+3])}), bb_color_, 2);
    }
}


void DynamicDetector::callback(dynamic_nav_interfaces::msg::DetectorData::SharedPtr msg) {

    auto start_timer = std::chrono::system_clock::now();
    auto unpack_s = std::chrono::system_clock::now();

    size_t obstacles_num = msg->boxes.size() / 4;

    cv_bridge::CvImagePtr rgb = cv_bridge::toCvCopy(msg->rgb);
    cv::Mat depth = cv_bridge::toCvCopy(msg->depth)->image;
    cv::Mat mask = cv_bridge::toCvCopy(msg->mask)->image;

    std::vector<uint64_t> boxes = msg->boxes;

    camera_model_.fromCameraInfo(msg->camera_info);

    auto unpack_e = std::chrono::system_clock::now();

    std::vector<std::vector<cv::Point>> image_points = setImagePoints(boxes);
    auto dds = std::chrono::system_clock::now();
    cv::Mat dynamic_depth = extractDynamicDepth(depth, mask, image_points);
    auto dde = std::chrono::system_clock::now();

    auto st = std::chrono::system_clock::now();
    std::vector<std::vector<geometry_msgs::msg::Point>> front_real_points = getRealPoints(image_points, dynamic_depth);
    auto mid = std::chrono::system_clock::now();
    std::vector<std::vector<geometry_msgs::msg::Point>> real_points = setRealBox(front_real_points);

    visualization_msgs::msg::Marker line_list = createBoxMsg(real_points, image_points);

    auto pub_s = std::chrono::system_clock::now();
    auto obstacles_footprints = dynamic_nav_interfaces::msg::ObstaclesFootprints();
    obstacles_footprints.count = obstacles_num;
    obstacles_footprints.points = box_points_;

    drawBoxes(rgb->image, boxes);
    auto rgb_msg = rgb->toImageMsg();
    auto depth_msg = cv_bridge::CvImage(msg->depth.header, sensor_msgs::image_encodings::TYPE_32FC1, dynamic_depth);

    if (!use_lifetime_ || (line_list.points.size() > 0)) {
        box_pub_->publish(line_list);
    }
    obstacles_fp_pub_->publish(obstacles_footprints);
    bb_image_pub_->publish(*rgb_msg);
    dyn_depth_pub_->publish(*depth_msg.toImageMsg());

    box_points_ = std::vector<geometry_msgs::msg::Point>();
    auto pub_e = std::chrono::system_clock::now();

    auto end_timer = std::chrono::system_clock::now();

    auto dt_unpack = std::chrono::duration_cast<std::chrono::milliseconds>(unpack_e - unpack_s).count();
    auto dt_dd = std::chrono::duration_cast<std::chrono::milliseconds>(dde - dds).count();
    auto f = std::chrono::duration_cast<std::chrono::milliseconds>(mid - st).count();
    auto dt_pub = std::chrono::duration_cast<std::chrono::milliseconds>(pub_e - pub_s).count();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count();
    
    if (verbose_) {
        RCLCPP_INFO(this->get_logger(), "%ld obstacle(s) detected; dt = %ld; dt_unpack = %ld; dt_dd = %ld; f = %ld; dt_pub = %ld;",
        obstacles_num, dt, dt_unpack, dt_dd, f, dt_pub);
    }
}


void DynamicDetector::dangerDistanceCallback(const std_msgs::msg::Bool msg) {
    bb_color_ = msg.data ? red_ : magenta_;
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicDetector>()); 
    rclcpp::shutdown();
    return 0;
}
