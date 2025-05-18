#ifndef F1TENTH_WAYPOINT__RECORD_CLICKED_WAYPOINTS_HPP_
#define F1TENTH_WAYPOINT__RECORD_CLICKED_WAYPOINTS_HPP_

#include <string>

#include "geometry_msgs/msg/point_stamped.hpp" 
#include "geometry_msgs/msg/pose.hpp"

#include "f1tenth_waypoint/waypoint_utils.hpp" 

#include "rclcpp/rclcpp.hpp"
#include <memory>

class ClickedWaypoint : public UtilsWaypoint
{
public:
    ClickedWaypoint(std::shared_ptr<rclcpp::Node> rclcpp_node);

    void clicked_point_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr clickedPoint_msgs,
                                const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& waypoint_marker_pub_);
    
   ~ClickedWaypoint();


private:

    std::vector<geometry_msgs::msg::Point> interpolate_points( const geometry_msgs::msg::Point &start_point,
                                                                const geometry_msgs::msg::Point &end_point, 
                                                                uint8_t numpoints);
    void public_waypoint_marker(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& waypoint_marker_pub_);



private:
    std::shared_ptr<rclcpp::Node> node_; 
    std::shared_ptr<geometry_msgs::msg::Point> previous_point_;
    uint8_t precision_;
};



#endif  