#include "f1tenth_waypoint/record_clicked_waypoints.hpp"
#include <cmath> 
#include <vector>

ClickedWaypoint::ClickedWaypoint(std::shared_ptr<rclcpp::Node> rclcpp_node):    UtilsWaypoint(rclcpp_node),
                                                                                node_(rclcpp_node), 
                                                                                previous_point_(nullptr),
                                                                                precision_(10)
{

}


void ClickedWaypoint::clicked_point_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr clickedPoint_msgs,
                                            const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& waypoint_marker_pub_)
{
    geometry_msgs::msg::Point new_point = clickedPoint_msgs->point;

    if (this->previous_point_ != nullptr)
    {
        std::vector<geometry_msgs::msg::Point> interpolated_points = this->interpolate_points(*previous_point_, new_point, precision_);
       
        for(const auto &point : interpolated_points)
        {
            RCLCPP_INFO(this->node_->get_logger(), "Interpolated Point: x = %.2f, y = %.2f, z = %.2f", point.x, point.y, point.z);
            auto point_shared = std::make_shared<geometry_msgs::msg::Point>(point); 
            this->write_waypoint_to_csv(point_shared);
            this->update_waypoint_marker_callback(point_shared); 
              
        }
    }

    auto new_point_shared = std::make_shared<geometry_msgs::msg::Point>(new_point);

    this->write_waypoint_to_csv(new_point_shared);
    this->update_waypoint_marker_callback(new_point_shared); 
    this->public_waypoint_marker(waypoint_marker_pub_);

    this->previous_point_ = new_point_shared; //std::make_shared<geometry_msgs::msg::Point>(new_point);
}

void ClickedWaypoint::public_waypoint_marker(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& waypoint_marker_pub_)
{
    const auto& waypoint_marker = this->get_waypoint_marker();
    waypoint_marker_pub_->publish(waypoint_marker);
}


std::vector<geometry_msgs::msg::Point> ClickedWaypoint::interpolate_points(const geometry_msgs::msg::Point &start_point,
                                                                const geometry_msgs::msg::Point &end_point, 
                                                                uint8_t numpoints)
{
    std::vector<geometry_msgs::msg::Point> points; // to store the interpolated points
    geometry_msgs::msg::Point interpolated_point;

    for(int i = 1; i <= numpoints; i++)
    {
        double fraction = static_cast<double>(i) / (numpoints + 1);
        // Interpolate x, y and z
        interpolated_point.x = start_point.x + fraction * (end_point.x - start_point.x);
        interpolated_point.y = start_point.y + fraction * (end_point.y - start_point.y);
        interpolated_point.z = start_point.z + fraction * (end_point.z - start_point.z);
        points.push_back(interpolated_point); // Add the interpolated point to the list
    }
    return points;
}



ClickedWaypoint::~ClickedWaypoint()
{

}