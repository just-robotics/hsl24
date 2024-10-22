#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


using namespace std::chrono_literals;


class Point {
private:
    geometry_msgs::msg::PoseStamped pt_;
    size_t idx_;
    size_t parent_idx_;
    tf2::Quaternion q_;

public:
    Point() {}
    Point(float x, float y, float t, float idx, float parent_idx) {
        pt_ = geometry_msgs::msg::PoseStamped();
        q_.setRPY(0, 0, t);
        pt_.pose.position.x = x;
        pt_.pose.position.y = y;
        pt_.pose.orientation.x = q_.getX();
        pt_.pose.orientation.y = q_.getY();
        pt_.pose.orientation.z = q_.getZ();
        pt_.pose.orientation.w = q_.getW();
        pt_.header.frame_id = "map";
        idx_ = idx;
        parent_idx_ = parent_idx;
    }

    float x() {return pt_.pose.position.x;}
    float y() {return pt_.pose.position.y;}
    float t() {tf2::Matrix3x3 m(q_); double roll, pitch, yaw; m.getRPY(roll, pitch, yaw); return yaw;}
    size_t idx() {return idx_;}
    size_t parent_idx() {return parent_idx_;}
    geometry_msgs::msg::PoseStamped pt() {return pt_;}
};


class Dispatcher : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr slave_goal_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr master_allow_driving_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr slave_allow_driving_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr result_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr slave_result_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aruco_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rotated_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr translated_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr result_loop_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_rot_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_tran_pub_;

    rclcpp::TimerBase::SharedPtr timer1_, timer2_;

    bool verbose_;
    bool started_ = false;
    size_t state_ = 1;
    bool aruco_ = false;

    bool slave_ready_ = true;
    bool master_ready_ = false;
    bool last_ = false;
    Point slave_pt_;

    bool rotated_ = false;
    bool sent_rotate_ = false;

    bool translated_ = false;
    bool sent_translate_ = false;

    std::vector<Point> graph_;

public:
    Dispatcher();

private:
    void initCallback();
    void arucoCallback(std_msgs::msg::Bool msg);
    void slaveCallback(std_msgs::msg::Bool msg);
    void callback();
    void rotatedCallback(std_msgs::msg::Bool msg);
    void translatedCallback(std_msgs::msg::Bool msg);

    void parsePoints(std::vector<double> points);
    geometry_msgs::msg::PoseStamped dropPoint();
    void feedbackCallback(std_msgs::msg::Bool msg);
};


Dispatcher::Dispatcher() : Node("dispatcher") {
    using std::placeholders::_1;

    this->declare_parameter("verbose", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("goal_pose_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("slave_goal_pose_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("master_allow_driving_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("slave_allow_driving_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("result_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("slave_result_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("aruco_topic", rclcpp::PARAMETER_STRING);
    this->declare_parameter("points", rclcpp::PARAMETER_DOUBLE_ARRAY);

    this->declare_parameter("start", false);
    
    verbose_ = this->get_parameter("verbose").as_bool();
    std::string goal_pose_topic = this->get_parameter("goal_pose_topic").as_string();
    std::string slave_goal_pose_topic = this->get_parameter("slave_goal_pose_topic").as_string();
    std::string master_allow_driving_topic = this->get_parameter("master_allow_driving_topic").as_string();
    std::string slave_allow_driving_topic = this->get_parameter("slave_allow_driving_topic").as_string();
    std::string result_topic = this->get_parameter("result_topic").as_string();
    std::string slave_result_topic = this->get_parameter("slave_result_topic").as_string();
    std::string aruco_topic = this->get_parameter("aruco_topic").as_string();
    std::vector<double> points = this->get_parameter("points").as_double_array();

    RCLCPP_INFO(this->get_logger(), "verbose: '%s'", verbose_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "goal_pose_topic: '%s'", goal_pose_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "slave_goal_pose_topic: '%s'", slave_goal_pose_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "master_allow_driving_topic: '%s'", master_allow_driving_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "slave_allow_driving_topic: '%s'", slave_allow_driving_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "result_topic: '%s'", result_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "slave_result_topic: '%s'", slave_result_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "aruco_topic: '%s'", aruco_topic.c_str());
    parsePoints(points);

    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic, 10);
    slave_goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(slave_goal_pose_topic, 10);
    master_allow_driving_pub_ = this->create_publisher<std_msgs::msg::Bool>(master_allow_driving_topic, 10);
    slave_allow_driving_pub_ = this->create_publisher<std_msgs::msg::Bool>(slave_allow_driving_topic, 10);
    result_sub_ = this->create_subscription<std_msgs::msg::Bool>(result_topic, 10, std::bind(&Dispatcher::feedbackCallback, this, _1));
    rotated_sub_ = this->create_subscription<std_msgs::msg::Bool>("/slave/rotated", 10, std::bind(&Dispatcher::rotatedCallback, this, _1));
    translated_sub_ = this->create_subscription<std_msgs::msg::Bool>("/slave/translated", 10, std::bind(&Dispatcher::translatedCallback, this, _1));
    slave_result_sub_ = this->create_subscription<std_msgs::msg::Bool>(slave_result_topic, 10, std::bind(&Dispatcher::slaveCallback, this, _1));
    aruco_sub_ = this->create_subscription<std_msgs::msg::Bool>(aruco_topic, 10, std::bind(&Dispatcher::arucoCallback, this, _1));
    result_loop_pub_ = this->create_publisher<std_msgs::msg::Bool>(result_topic, 10);
    start_rot_pub_ = this->create_publisher<std_msgs::msg::Bool>("/slave/start_rotation", 10);
    start_tran_pub_ = this->create_publisher<std_msgs::msg::Bool>("/slave/start_translation", 10);

    timer1_ = this->create_wall_timer(100ms, std::bind(&Dispatcher::initCallback, this));
    timer2_ = this->create_wall_timer(10ms, std::bind(&Dispatcher::callback, this));
}


void Dispatcher::initCallback() {
    if (!this->get_parameter("start").as_bool() || started_) {
            return;
    }

    started_ = true;
    auto msg = graph_[state_].pt();
    goal_pose_pub_->publish(msg);
}


void Dispatcher::parsePoints(std::vector<double> points) {
    for (size_t i = 0; i < points.size(); i += 5) {
        auto pt = Point(points[i], points[i+1], points[i+2], static_cast<size_t>(points[i+3]), static_cast<size_t>(points[i+4]));
        graph_.push_back(pt);
        RCLCPP_INFO(this->get_logger(), "point %ld: %lf %lf %lf %ld %ld", i / 5, pt.x(), pt.y(), pt.t(), pt.idx(), pt.parent_idx());
    }
}


geometry_msgs::msg::PoseStamped Dispatcher::dropPoint() {
    state_ = state_ == graph_.size() - 1 ? 0 : state_ + 1;
    auto pt = graph_[state_].pt();
    pt.header.stamp = this->get_clock()->now();
    return pt;
}


void Dispatcher::feedbackCallback(std_msgs::msg::Bool msg) {
    if (!aruco_) {
        if (msg.data) {
            goal_pose_pub_->publish(dropPoint());
            auto pt = graph_[state_];
            RCLCPP_INFO(this->get_logger(), "publishing point %ld: %lf %lf %lf %ld %ld", state_, pt.x(), pt.y(), pt.t(), pt.idx(), pt.parent_idx());
        }
        return;
    }

    master_ready_ = true;
}


void delay(size_t ms) {
    auto start = std::chrono::system_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count() < static_cast<long int>(ms)) {}
}


void Dispatcher::callback() {

    if (master_ready_) {
        if (!rotated_ && !sent_rotate_) {
            RCLCPP_INFO(this->get_logger(), "SEND ROTATION");
            delay(1000);
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            start_rot_pub_->publish(msg);
            sent_rotate_ = true;
            return;
        }
        else if (rotated_ && !translated_ && !sent_translate_) {
            RCLCPP_INFO(this->get_logger(), "SEND TRANSLATION");
            sent_translate_ = true;
            delay(1000);
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            start_tran_pub_->publish(msg);
            auto curr_master_pt = graph_[state_];
            slave_pt_ = curr_master_pt;
            size_t parent_idx = curr_master_pt.parent_idx();
            if (state_ != parent_idx) {
                auto pt = graph_[parent_idx];
                goal_pose_pub_->publish(pt.pt());
                state_ = parent_idx;
                RCLCPP_INFO(this->get_logger(), "publishing master point %ld: %lf %lf %lf %ld %ld", state_, pt.x(), pt.y(), pt.t(), pt.idx(), pt.parent_idx());
                master_ready_ = false;
            }
            else {
                RCLCPP_INFO(this->get_logger(), "MASTER STOP");
            }
            return;
        }
        else if (translated_) {
            RCLCPP_INFO(this->get_logger(), "LOOP CLOSING");
            sent_rotate_ = false;
            rotated_ = false;
            sent_translate_ = false;
            translated_ = false;
            master_ready_ = true;
        }

        // delay(1000);
        // Point& pt = slave_pt_;
        // slave_goal_pose_pub_->publish(pt.pt());
        // RCLCPP_INFO(this->get_logger(), "publishing slave point: %lf %lf %lf %ld %ld", pt.x(), pt.y(), pt.t(), pt.idx(), pt.parent_idx());
        // slave_ready_ = false;
    }
}


void Dispatcher::arucoCallback(std_msgs::msg::Bool msg) {
    if (aruco_) {
        return;
    }
    
    if (msg.data) {
        msg.data = false;
        master_allow_driving_pub_->publish(msg);
        aruco_ = true;
        msg.data = true;
        master_allow_driving_pub_->publish(msg);
        result_loop_pub_->publish(msg);
        master_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "aruco found");
    }
}


void Dispatcher::slaveCallback(std_msgs::msg::Bool msg) {
    if (msg.data) {
        slave_ready_ = true;
    }
}


void Dispatcher::rotatedCallback(std_msgs::msg::Bool msg) {
    if (msg.data) {
        rotated_ = true;
        RCLCPP_INFO(this->get_logger(), "rotated");
    }
}


void Dispatcher::translatedCallback(std_msgs::msg::Bool msg) {
    if (msg.data) {
        translated_ = true;
        RCLCPP_INFO(this->get_logger(), "translated");
    }
}



int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Dispatcher>()); 
    rclcpp::shutdown();
    return 0;
}
