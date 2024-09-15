#ifndef graph_hpp
#define graph_hpp


#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "points.hpp"


namespace Node {
    
}


class Graph {
private:
    geometry_msgs::msg::PoseStamped* node_array;
    size_t current_idx;

public:
    Graph();
    
    geometry_msgs::msg::PoseStamped dropMsg();
    geometry_msgs::msg::PoseStamped currentMsg();
    bool repeat;
};


Graph::Graph() {

    node_array = new geometry_msgs::msg::PoseStamped[Points::points_number];
    current_idx = 0;
    repeat = false;

    for (size_t i = 0; i < Points::points_number; i++) {
        auto msg = geometry_msgs::msg::PoseStamped();
        tf2::Quaternion q;
        q.setRPY(0, 0, Points::pt_array[i].yaw);

        msg.pose.position.x = Points::pt_array[i].x;
        msg.pose.position.y = Points::pt_array[i].y;
        msg.pose.position.z = 0;
        msg.pose.orientation.x = q.getX();
        msg.pose.orientation.y = q.getY();
        msg.pose.orientation.z = q.getZ();
        msg.pose.orientation.w = q.getW();

        node_array[i] = msg;
    }
}


geometry_msgs::msg::PoseStamped Graph::dropMsg() {
    if (!repeat) {
        repeat = true;
        return currentMsg();
    }
    geometry_msgs::msg::PoseStamped* msg = &node_array[current_idx];
    if (current_idx == Points::points_number - 1) {
        current_idx = 0;
    }
    else {
        current_idx++;
    }
    return *msg;
}


geometry_msgs::msg::PoseStamped Graph::currentMsg() {
    return node_array[current_idx];
}


#endif // graph_hpp
