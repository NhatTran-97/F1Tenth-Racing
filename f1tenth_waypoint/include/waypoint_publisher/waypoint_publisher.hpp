#ifndef F1TENTH_WAYPOINT__WAYPOINT_PUBLISHER_HPP_
#define F1TENTH_WAYPOINT__WAYPOINT_PUBLISHER_HPP_

// #include <cstdlib> /*IO operation*/
#include <cstdio>
// #include <fstream>
#include <memory>
#include <string>
#include <chrono>
#include <iostream>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/path.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class WaypointPublisher : public rclcpp::Node
{
public:
    WaypointPublisher(const std::string &name);

private:
    void timer_callback();

    std::string get_file_loc(std::string file_name);

    std::vector<double> get_orientation(geometry_msgs::msg::PoseStamped pose1, geometry_msgs::msg::PoseStamped pose2);

    nav_msgs::msg::Path get_waypoints(std::string filepath, std::string filename);

    nav_msgs::msg::Path blend_paths(nav_msgs::msg::Path path1, nav_msgs::msg::Path path2, double blend_factor = 0.5);



private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raceline_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr centerline_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr blend_pub_;

    nav_msgs::msg::Path raceline_path;
    nav_msgs::msg::Path centerline_path;
    nav_msgs::msg::Path blend_path;
    
    std::string centerline_file_name;
    std::string centerline_file_loc;
};

#endif // F1TENTH_WAYPOINT_FOLLOWER__WAYPOINT_PANEL_HPP_
