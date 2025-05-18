#ifndef F1TENTH_WAYPOINT__WAYPOINT_GENERATOR_NODE_HPP_
#define F1TENTH_WAYPOINT__WAYPOINT_GENERATOR_NODE_HPP_

#include <string>
#include <string_view>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "waypoint_generator_node.hpp"
#include "f1tenth_waypoint/record_odom_waypoints.hpp"
#include "f1tenth_waypoint/record_clicked_waypoints.hpp"
#include <memory>

class Waypoint : public rclcpp::Node
{
public:
    Waypoint(const std::string &name);
    void initialize();

private:
    std::string message_type, waypoint_topic;
    static constexpr char CLICKED_WAYPOINT[] = "clicked_waypoint";
    static constexpr char ODOM_WAYPOINT[] = "odom_waypoint";

    // Subscriptions (use SharedPtr)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;

    

    // Publisher for waypoint markers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoint_marker_pub_;

    std::shared_ptr<OdomWaypoint> odom_waypoint_;
    std::shared_ptr<ClickedWaypoint> clicked_waypoint_;

private:
    std::pair<std::string, std::string> setup_waypoint_option();

    // Use specific message types in callbacks
    void waypoint_callback(const geometry_msgs::msg::PointStamped::SharedPtr waypoint_msgs);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msgs);
    

   

    // Map enum to string using string_view
};

#endif // F1TENTH_WAYPOINT__WAYPOINT_GENERATOR_NODE_HPP_
