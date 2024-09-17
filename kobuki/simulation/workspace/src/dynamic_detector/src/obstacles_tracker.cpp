#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "dynamic_nav_interfaces/msg/dynamic_layer_msg.hpp"
#include "dynamic_nav_interfaces/msg/costmap_circle.hpp"
#include "dynamic_nav_interfaces/msg/obstacles_footprints.hpp"
#include "dynamic_nav_interfaces/msg/min_distance.hpp"


class ObstaclesTracker : public rclcpp::Node {
private:
    const int IMG_UNOCCUPIED_ = 255;
    const int IMG_OCCUPIED_ = 0;
    const int IMG_UNKNOWN_ = 128;
    int MAP_UNOCCUPIED_;
    int MAP_OCCUPIED_;
    int MAP_UNKNOWN_;

    rclcpp::Subscription<dynamic_nav_interfaces::msg::ObstaclesFootprints>::SharedPtr fp_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
    rclcpp::Publisher<dynamic_nav_interfaces::msg::DynamicLayerMsg>::SharedPtr costmap_circles_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr remap_lc_pub_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr box_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr box_pub_;
    rclcpp::Publisher<dynamic_nav_interfaces::msg::MinDistance>::SharedPtr min_distance_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

    std::string map_frame_, camera_link_optical_frame_, odom_frame_, lidar_frame_, base_footprint_frame_;
    float obstacle_offset_;
    cv::Mat map_img_;
    nav_msgs::msg::OccupancyGrid map_;
    tf2::Transform map_tf_, odom_tf_, odom2map_tf_, base_footprint2map_tf_, lidar2base_footprint_tf_;
    bool empty_base_footprint2map_tf_, empty_lidar2base_footprint_tf_;
    bool verbose_;
    size_t lost_memory_time_;
    bool use_lifetime_;
    size_t bb_lifetime_;

    size_t prev_count_;
    std::vector<tf2::Vector3> prev_map_points_;
    std::vector<tf2::Vector3> prev_local_costmap_points_;
    std::vector<tf2::Vector3> lost_map_points_;
    std::vector<tf2::Vector3> lost_local_costmap_points_;
    std::vector<rclcpp::Time> lost_map_time_;
    std::vector<rclcpp::Time> lost_local_costmap_time_;

public:
    ObstaclesTracker();

private:
    visualization_msgs::msg::Marker create2dBoxMsg(std::vector<geometry_msgs::msg::Point> points);
    std::vector<tf2::Vector3> getLostPoints(std::vector<tf2::Vector3>& lost_points, std::vector<rclcpp::Time>& lost_time);

    void footprintsCallback(const dynamic_nav_interfaces::msg::ObstaclesFootprints msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid msg);
    void localCostmapCallback(const nav_msgs::msg::OccupancyGrid msg);
    void transformCallback();
    void boxCallback(visualization_msgs::msg::Marker::SharedPtr msg);
};


ObstaclesTracker::ObstaclesTracker() : Node("obstacles_tracker") {
    using namespace std::chrono_literals;
    using std::placeholders::_1;

    this->declare_parameter("verbose", true);
    this->declare_parameter("footprints_topic", "");
    this->declare_parameter("map_sub_topic", "");
    this->declare_parameter("map_pub_topic", "");
    this->declare_parameter("local_costmap_sub_topic", "");
    this->declare_parameter("costmap_circles_topic", "");
    this->declare_parameter("remap_local_costmap_topic", "");
    this->declare_parameter("box_sub_topic", "");
    this->declare_parameter("box_pub_topic", "");
    this->declare_parameter("min_distance_topic", "");
    this->declare_parameter("map_frame", "");
    this->declare_parameter("odom_frame", "");
    this->declare_parameter("camera_link_optical_frame", "");
    this->declare_parameter("obstacle_offset", 0.0);
    this->declare_parameter("unoccupied_value", 0);
    this->declare_parameter("occupied_value", 0);
    this->declare_parameter("unknown_value", 0);
    this->declare_parameter("use_emergency_stop", false);
    this->declare_parameter("danger_distance", 0.0);
    this->declare_parameter("lidar_frame", "");
    this->declare_parameter("base_footprint_frame", "");
    this->declare_parameter("lost_memory_time", 0);
    this->declare_parameter("use_lifetime", false);
    this->declare_parameter("bb_lifetime", 0);

    verbose_ = this->get_parameter("verbose").as_bool();
    std::string fp_topic = this->get_parameter("footprints_topic").as_string();
    std::string map_sub_topic = this->get_parameter("map_sub_topic").as_string();
    std::string map_pub_topic = this->get_parameter("map_pub_topic").as_string();
    std::string local_costmap_sub_topic = this->get_parameter("local_costmap_sub_topic").as_string();
    std::string costmap_circles_topic = this->get_parameter("costmap_circles_topic").as_string();
    std::string remap_local_costmap_topic = this->get_parameter("remap_local_costmap_topic").as_string();
    std::string box_sub_topic = this->get_parameter("box_sub_topic").as_string();
    std::string box_pub_topic = this->get_parameter("box_pub_topic").as_string();
    std::string min_distance_topic = this->get_parameter("min_distance_topic").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    camera_link_optical_frame_ = this->get_parameter("camera_link_optical_frame").as_string();
    obstacle_offset_ = this->get_parameter("obstacle_offset").as_double();
    MAP_UNOCCUPIED_ = this->get_parameter("unoccupied_value").as_int();
    MAP_OCCUPIED_ = this->get_parameter("occupied_value").as_int();
    MAP_UNKNOWN_ = this->get_parameter("unknown_value").as_int();
    lidar_frame_ = this->get_parameter("lidar_frame").as_string();
    base_footprint_frame_ = this->get_parameter("base_footprint_frame").as_string();
    lost_memory_time_ = this->get_parameter("lost_memory_time").as_int();
    use_lifetime_ = this->get_parameter("use_lifetime").as_bool();
    bb_lifetime_ = this->get_parameter("bb_lifetime").as_int();

    RCLCPP_INFO(this->get_logger(), "verbose: '%s'", verbose_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "footprints_topic: '%s'", fp_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "map_sub_topic: '%s'", map_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "map_pub_topic: '%s'", map_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "local_costmap_sub_topic: '%s'", local_costmap_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "costmap_circles_topic: '%s'", costmap_circles_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "remap_local_costmap_topic: '%s'", remap_local_costmap_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "box_sub_topic: '%s'", box_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "box_pub_topic: '%s'", box_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "min_distance_topic: '%s'", min_distance_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "map_frame: '%s'", map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "odom_frame: '%s'", odom_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_link_optical_frame: '%s'", camera_link_optical_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "obstacle_offset: '%f'", obstacle_offset_);
    RCLCPP_INFO(this->get_logger(), "unoccupied_value: '%d'", MAP_UNOCCUPIED_);
    RCLCPP_INFO(this->get_logger(), "occupied_value: '%d'", MAP_OCCUPIED_);
    RCLCPP_INFO(this->get_logger(), "unknown_value: '%d'", MAP_UNKNOWN_);
    RCLCPP_INFO(this->get_logger(), "lidar_frame: '%s'", lidar_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "base_footprint_frame: '%s'", base_footprint_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "lost_memory_time: '%ld'", lost_memory_time_);
    RCLCPP_INFO(this->get_logger(), "use_lifetime: '%s'", use_lifetime_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "bb_lifetime: '%ld'", bb_lifetime_);

    fp_sub_ = this->create_subscription<dynamic_nav_interfaces::msg::ObstaclesFootprints>(fp_topic, 10, std::bind(&ObstaclesTracker::footprintsCallback, this, _1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_sub_topic, 10, std::bind(&ObstaclesTracker::mapCallback, this, _1));
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_pub_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(local_costmap_sub_topic, 10, std::bind(&ObstaclesTracker::localCostmapCallback, this, _1));
    costmap_circles_pub_ = this->create_publisher<dynamic_nav_interfaces::msg::DynamicLayerMsg>(costmap_circles_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    remap_lc_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(remap_local_costmap_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().best_effort());
    box_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(box_sub_topic, 10, std::bind(&ObstaclesTracker::boxCallback, this, _1));
    box_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(box_pub_topic, 10);
    min_distance_pub_ = this->create_publisher<dynamic_nav_interfaces::msg::MinDistance>(min_distance_topic, 10);

    timer_ = this->create_wall_timer(10ms, std::bind(&ObstaclesTracker::transformCallback, this));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    empty_base_footprint2map_tf_ = true;
    empty_lidar2base_footprint_tf_ = true;

    prev_count_ = 0;
}


std::vector<tf2::Vector3> ObstaclesTracker::getLostPoints(std::vector<tf2::Vector3>& lost_points, std::vector<rclcpp::Time>& lost_time) {
    std::vector<tf2::Vector3> new_lost_points;
    std::vector<rclcpp::Time> new_lost_time;
    std::vector<tf2::Vector3> actual_points;
    std::vector<size_t> actual_ids;
    auto time = this->get_clock()->now();

    for (size_t i = 0; i < lost_time.size(); i++) {
        if (time.seconds() - lost_time[i].seconds() < lost_memory_time_) {
            actual_points.push_back(lost_points[4*i+0]);
            actual_points.push_back(lost_points[4*i+1]);
            actual_points.push_back(lost_points[4*i+2]);
            actual_points.push_back(lost_points[4*i+3]);
            actual_ids.push_back(i);
        }
    }

    for (size_t i = 0; i < actual_ids.size(); i++) {
        new_lost_points.push_back(lost_points[4*actual_ids[i]+0]);
        new_lost_points.push_back(lost_points[4*actual_ids[i]+1]);
        new_lost_points.push_back(lost_points[4*actual_ids[i]+2]);
        new_lost_points.push_back(lost_points[4*actual_ids[i]+3]);
        new_lost_time.push_back(lost_time[actual_ids[i]]);
    }

    lost_points = new_lost_points;
    lost_time = new_lost_time;

    return actual_points;
}


void ObstaclesTracker::footprintsCallback(const dynamic_nav_interfaces::msg::ObstaclesFootprints msg) {
    auto start_timer = std::chrono::system_clock::now();

    if (map_img_.empty()) {
        RCLCPP_WARN(this->get_logger(), "static map is empty");
        if (map_.data.size() > 0) {
            map_.header.stamp = this->get_clock()->now();
            map_pub_->publish(map_);
        }
        return;
    }

    // static map's origin coordinates
    tf2::Vector3 map_origin;
    map_origin.setX(map_.info.origin.position.x);
    map_origin.setY(map_.info.origin.position.y);
    map_origin.setZ(map_.info.origin.position.z);

    std::vector<tf2::Vector3> map_points;
    std::vector<tf2::Vector3> local_costmap_points;

    dynamic_nav_interfaces::msg::MinDistance min_distance;
    min_distance.min_distance = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < msg.points.size(); i++) {
        tf2::Vector3 v(msg.points[i].x, msg.points[i].y, msg.points[i].z); // set points in camera_link_optical frame to tf2::Vector3
        tf2::Vector3 static_map_pt = (map_tf_ * v); // transform points from camera_link_optical to map frame
        tf2::Vector3 local_costmap_pt = (odom_tf_ * v); // transform points from camera_link_optical to odom frame
        map_points.push_back(static_map_pt);
        local_costmap_points.push_back(local_costmap_pt);

        float x = msg.points[i].x;
        float z = msg.points[i].z;
        float d = std::sqrt(x * x + z * z);
        if (d < min_distance.min_distance) {
            min_distance.min_distance = d;
            min_distance.angle = std::atan(x / z);
        }
    }

    min_distance_pub_->publish(min_distance);

    if (msg.count < prev_count_) {
        lost_map_points_.insert(lost_map_points_.end(), prev_map_points_.begin(), prev_map_points_.end());
        lost_local_costmap_points_.insert(lost_local_costmap_points_.end(), prev_local_costmap_points_.begin(), prev_local_costmap_points_.end());
        auto time = this->get_clock()->now();
        for (size_t i = 0; i < prev_count_; i++) {
            lost_map_time_.push_back(time);
            lost_local_costmap_time_.push_back(time);
        }
    }

    std::vector<tf2::Vector3> lost_map_points = getLostPoints(lost_map_points_, lost_map_time_);
    std::vector<tf2::Vector3> lost_local_costmap_points = getLostPoints(lost_local_costmap_points_, lost_local_costmap_time_);

    prev_map_points_ = map_points;
    prev_local_costmap_points_ = local_costmap_points;
    prev_count_ = msg.count;

    map_points.insert(map_points.end(), lost_map_points.begin(), lost_map_points.end());
    local_costmap_points.insert(local_costmap_points.end(), lost_local_costmap_points.begin(), lost_local_costmap_points.end());

    for (size_t i = 0; i < map_points.size(); i++) {
        map_points[i] -= map_origin; // transform points from camera_link_optical to static map's origin
    }

    float resolution = map_.info.resolution;
    cv::Mat dynamic_map_img = map_img_.clone();
    
    static const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat mask0 = (dynamic_map_img == IMG_OCCUPIED_);
    cv::erode(mask0, mask0, kernel, cv::Point(-1, -1), 4);
    dynamic_map_img.setTo(IMG_OCCUPIED_, mask0);

    auto dynamic_layer = dynamic_nav_interfaces::msg::DynamicLayerMsg();
    for (size_t i = 0; i < map_points.size(); i += 4) {
        // static map
        tf2::Vector3 static_pt0 = map_points[i+0];
        tf2::Vector3 static_pt3 = map_points[i+3];
        float x = (static_pt0.x() + static_pt3.x()) / 2; // find x_center of obstacle's rectangle in static map's origin
        float y = (static_pt0.y() + static_pt3.y()) / 2; // find y_center of obstacle's rectangle in static map's origin
        float rx = std::abs(x - static_pt0.x());
        float ry = std::abs(y - static_pt0.y());
        uint64_t r = static_cast<int>((std::sqrt(rx * rx + ry * ry) + obstacle_offset_) / resolution); // find obstacle footprint circle radius
        cv::Point center(static_cast<int>(x / resolution), static_cast<int>(y / resolution)); // find circle center
        try {
            cv::circle(dynamic_map_img, center, r, {double(IMG_OCCUPIED_)}, cv::FILLED);
        }
        catch (...) {
            RCLCPP_WARN(this->get_logger(), "cannot put obstacle into static_map");
        }

        // local costmap
        tf2::Vector3 local_costmap_pt0 = local_costmap_points[i+0];
        tf2::Vector3 local_costmap_pt3 = local_costmap_points[i+3];
        auto dynamic_obstacle = dynamic_nav_interfaces::msg::CostmapCircle();
        dynamic_obstacle.x = (local_costmap_pt0.x() + local_costmap_pt3.x()) / 2; // find x_center of obstacle's rectangle in local costmap's origin
        dynamic_obstacle.y = (local_costmap_pt0.y() + local_costmap_pt3.y()) / 2; // find y_center of obstacle's rectangle in local costmap's origin
        rx = std::abs(dynamic_obstacle.x - local_costmap_pt0.x());
        ry = std::abs(dynamic_obstacle.y - local_costmap_pt0.y());
        dynamic_obstacle.r = std::sqrt(rx * rx + ry * ry) + obstacle_offset_; // find obstacle footprint circle radius
        dynamic_layer.obstacles.push_back(dynamic_obstacle);
    }

    auto dynamic_map = map_;

    dynamic_map.header.stamp = this->get_clock()->now();

    for (size_t i = 0; i < size_t(map_img_.rows * map_img_.cols); i++) {
        if (dynamic_map_img.data[i] == IMG_UNOCCUPIED_) {
            dynamic_map.data[i] = MAP_UNOCCUPIED_;
        }
        if (dynamic_map_img.data[i] == IMG_OCCUPIED_) {
            dynamic_map.data[i] = MAP_OCCUPIED_;
        }
        if (dynamic_map_img.data[i] == IMG_UNKNOWN_) {
            dynamic_map.data[i] = MAP_UNKNOWN_;
        }
    }

    map_pub_->publish(dynamic_map);
    costmap_circles_pub_->publish(dynamic_layer);

    auto end_timer = std::chrono::system_clock::now();

    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(end_timer - start_timer).count();
    
    if (verbose_) {
        RCLCPP_INFO(this->get_logger(), "got %ld obstacle(s); dt = %ld;", msg.count, dt);
    }
#if false
    cv::flip(dynamic_map_img, dynamic_map_img, 1);
    cv::imshow("dynamic_map", dynamic_map_img);
    cv::waitKey(1);
#endif
}


void ObstaclesTracker::mapCallback(const nav_msgs::msg::OccupancyGrid msg) {
    if (verbose_) {
        RCLCPP_INFO(this->get_logger(), "static map updated");
    }
    map_ = msg;
    map_img_ = cv::Mat::zeros({int(msg.info.width), int(msg.info.height)}, CV_8UC1);
    for (size_t i = 0; i < size_t(map_img_.rows * map_img_.cols); i++) {
        if (msg.data[i] == MAP_UNOCCUPIED_) {
            map_img_.data[i] = IMG_UNOCCUPIED_;
        }
        if (msg.data[i] == MAP_OCCUPIED_) {
            map_img_.data[i] = IMG_OCCUPIED_;
        }
        if (msg.data[i] == MAP_UNKNOWN_) {
            map_img_.data[i] = IMG_UNKNOWN_;
        }
    }
}


void ObstaclesTracker::localCostmapCallback(nav_msgs::msg::OccupancyGrid msg) {
    if (verbose_) {
        RCLCPP_INFO(this->get_logger(), "local_costmap updated");
    }

    tf2::Vector3 odom_origin;
    odom_origin.setX(msg.info.origin.position.x);
    odom_origin.setY(msg.info.origin.position.y);
    odom_origin.setZ(msg.info.origin.position.z);

    tf2::Quaternion odom_q;
    odom_q.setX(msg.info.origin.orientation.x);
    odom_q.setY(msg.info.origin.orientation.y);
    odom_q.setZ(msg.info.origin.orientation.z);
    odom_q.setW(msg.info.origin.orientation.w);

    tf2::Vector3 map_origin = odom2map_tf_ * odom_origin;
    tf2::Quaternion map_q = odom2map_tf_ * odom_q;

    msg.info.origin.position.x = map_origin.x();
    msg.info.origin.position.y = map_origin.y();
    msg.info.origin.position.z = map_origin.z();
    msg.info.origin.orientation.x = map_q.getX();
    msg.info.origin.orientation.y = map_q.getY();
    msg.info.origin.orientation.z = map_q.getZ();
    msg.info.origin.orientation.w = map_q.getW();

    msg.header.frame_id = map_frame_;
    msg.header.stamp = this->get_clock()->now();

    remap_lc_pub_->publish(msg);
}


void ObstaclesTracker::transformCallback() {
    // camera_link_optical -> map tf
    try {
        auto transform = tf_buffer_->lookupTransform(map_frame_, camera_link_optical_frame_, tf2::TimePointZero);
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        tf2::Vector3 v(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );
        map_tf_ = tf2::Transform(q, v);
    }
    catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform '%s' to '%s': %s", map_frame_.c_str(), camera_link_optical_frame_.c_str(), ex.what());
    }

    // camera_link_optical -> odom tf
    try {
        auto transform = tf_buffer_->lookupTransform(odom_frame_, camera_link_optical_frame_, tf2::TimePointZero);
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        tf2::Vector3 v(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );
        odom_tf_ = tf2::Transform(q, v);
    }
    catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform '%s' to '%s': %s", odom_frame_.c_str(), camera_link_optical_frame_.c_str(), ex.what());
    }

    // odom -> map tf
    try {
        auto transform = tf_buffer_->lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero);
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        tf2::Vector3 v(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );
        odom2map_tf_ = tf2::Transform(q, v);
    }
    catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform '%s' to '%s': %s", map_frame_.c_str(), odom_frame_.c_str(), ex.what());
    }
#if false
    // map -> base_footprint tf
    try {
        auto transform = tf_buffer_->lookupTransform(map_frame_, base_footprint_frame_, tf2::TimePointZero);
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        tf2::Vector3 v(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );
        base_footprint2map_tf_ = tf2::Transform(q, v);
        empty_base_footprint2map_tf_ = false;
    }
    catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform '%s' to '%s': %s", map_frame_.c_str(), base_footprint_frame_.c_str(), ex.what());
    }

    // lidar -> base_footprint tf
    try {
        auto transform = tf_buffer_->lookupTransform(base_footprint_frame_, lidar_frame_, tf2::TimePointZero);
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        tf2::Vector3 v(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );
        lidar2base_footprint_tf_ = tf2::Transform(q, v);
        empty_lidar2base_footprint_tf_ = false;
    }
    catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform '%s' to '%s': %s", base_footprint_frame_.c_str(), lidar_frame_.c_str(), ex.what());
    }

    // lidar -> map tf
    try {
        auto transform = tf_buffer_->lookupTransform(map_frame_, lidar_frame_, tf2::TimePointZero);
        tf2_broadcaster_->sendTransform(transform);
    }
    catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform '%s' to '%s': %s", map_frame_.c_str(), lidar_frame_.c_str(), ex.what());
    }

    // calc lidar -> map transform
    if (empty_base_footprint2map_tf_ || empty_lidar2base_footprint_tf_) {
        RCLCPP_WARN(this->get_logger(), "Empty transforms: empty_base_footprint2map_tf_: %s; empty_lidar2base_footprint_tf_: %s",
        empty_base_footprint2map_tf_ ? "true" : "false",
        empty_lidar2base_footprint_tf_ ? "true" : "false");
    }
    else {
        tf2::Transform lidar2map_tf = base_footprint2map_tf_ * lidar2base_footprint_tf_;
        geometry_msgs::msg::TransformStamped pub_tf;
        pub_tf.header.stamp = this->get_clock()->now();
        // pub_tf.header.frame_id = map_frame_;
        pub_tf.child_frame_id = lidar_frame_;
        // pub_tf.transform.translation.x = lidar2map_tf.getOrigin().getX();
        // pub_tf.transform.translation.y = lidar2map_tf.getOrigin().getY();
        // pub_tf.transform.translation.z = lidar2map_tf.getOrigin().getZ();
        // pub_tf.transform.rotation.x = lidar2map_tf.getRotation().getX();
        // pub_tf.transform.rotation.y = lidar2map_tf.getRotation().getY();
        // pub_tf.transform.rotation.z = lidar2map_tf.getRotation().getZ();
        // pub_tf.transform.rotation.w = lidar2map_tf.getRotation().getW();
        pub_tf.header.frame_id = base_footprint_frame_;
        pub_tf.transform.translation.x = lidar2base_footprint_tf_.getOrigin().getX();
        pub_tf.transform.translation.y = lidar2base_footprint_tf_.getOrigin().getY();
        pub_tf.transform.translation.z = lidar2base_footprint_tf_.getOrigin().getZ();
        pub_tf.transform.rotation.x = lidar2base_footprint_tf_.getRotation().getX();
        pub_tf.transform.rotation.y = lidar2base_footprint_tf_.getRotation().getY();
        pub_tf.transform.rotation.z = lidar2base_footprint_tf_.getRotation().getZ();
        pub_tf.transform.rotation.w = lidar2base_footprint_tf_.getRotation().getW();
        tf2_broadcaster_->sendTransform(pub_tf);
    }
#endif
}


void ObstaclesTracker::boxCallback(visualization_msgs::msg::Marker::SharedPtr msg) {
    if (map_img_.empty() || msg->points.size() == 0) {
        return;
    }

    // static map's origin coordinates
    tf2::Vector3 static_map_origin;
    static_map_origin.setX(map_.info.origin.position.x);
    static_map_origin.setY(map_.info.origin.position.y);
    static_map_origin.setZ(map_.info.origin.position.z);

    for (size_t i = 0; i < msg->points.size(); i++) {
        tf2::Vector3 v(msg->points[i].x, msg->points[i].y, msg->points[i].z); // set points in camera_link_optical frame to tf2::Vector3
        tf2::Vector3 pt = (map_tf_ * v); // transform points from camera_link_optical to static map's origin
        msg->points[i].x = pt.getX();
        msg->points[i].y = pt.getY();
        msg->points[i].z = pt.getZ();
    }

    msg->header.frame_id = map_frame_;
    msg->header.stamp = this->get_clock()->now();

    if (use_lifetime_) {
        msg->lifetime.sec = static_cast<int32_t>(bb_lifetime_ / 10e3);
        msg->lifetime.nanosec = (bb_lifetime_ - msg->lifetime.sec * 10e3) * 10e6;
    }
    else {
        msg->lifetime.nanosec = 40 * 10e6;
    }

    box_pub_->publish(*msg);
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstaclesTracker>()); 
    rclcpp::shutdown();
    return 0;
}
