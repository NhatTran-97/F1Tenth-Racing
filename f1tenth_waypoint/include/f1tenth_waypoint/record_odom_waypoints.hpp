#ifndef F1TENTH_WAYPOINT__RECORD_ODOM_WAYPOINTS_HPP_
#define F1TENTH_WAYPOINT__RECORD_ODOM_WAYPOINTS_HPP_

#include <string>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "f1tenth_waypoint/waypoint_utils.hpp"

class OdomWaypoint : public UtilsWaypoint
{
public:
    OdomWaypoint(std::shared_ptr<rclcpp::Node> rclcpp_node);
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msgs, 
                       const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& waypoint_marker_pub_);
    
    ~OdomWaypoint();

private:
    std::shared_ptr<CSVFile> csv_file_;
private:
    double min_distance;
    double x_old = 0.0f;
    double y_old = 0.0f;

    std::shared_ptr<rclcpp::Node> node_;
    void initialize(std::shared_ptr<rclcpp::Node> rclcpp_node_);
    void public_waypoint_marker(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& waypoint_marker_pub_);

};

#endif  