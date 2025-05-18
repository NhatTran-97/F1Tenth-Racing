#include "f1tenth_waypoint/record_odom_waypoints.hpp"
#include <cmath> 

OdomWaypoint::OdomWaypoint(std::shared_ptr<rclcpp::Node> rclcpp_node) : UtilsWaypoint(rclcpp_node) , 
                                                                        node_(rclcpp_node)
{


    try
    {
        if (!node_->has_parameter("odom_min_distance"))
        {
            node_->declare_parameter<double>("odom_min_distance", 0.05);
        }
        min_distance = this->node_->get_parameter("odom_min_distance").as_double();
        RCLCPP_INFO(this->node_->get_logger(), "min distance parameter set to: %.2f", min_distance);

    }
    catch(const rclcpp::exceptions::ParameterNotDeclaredException &e) 
    {
        RCLCPP_WARN(this->node_ -> get_logger(), "Could not get parameters ... setting variables to default");
        RCLCPP_WARN(this->node_ -> get_logger(), "Error: %s", e.what());
    }
    catch(const std::exception &e)
    {
        RCLCPP_WARN(this->node_ ->get_logger(), "An error occured: %s", e.what());
        
    }
    
    this->initialize(rclcpp_node);
    
    
}

void OdomWaypoint::initialize(std::shared_ptr<rclcpp::Node> rclcpp_node_)
{
    csv_file_ = std::make_shared<CSVFile>(rclcpp_node_);

   // csv_file_ = std::make_shared<CSVFile>(this->shared_from_this());
}


void OdomWaypoint::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msgs,
                                const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& waypoint_marker_pub_)
{
    double diff = sqrt(pow((odom_msgs->pose.pose.position.x - x_old), 2) + 
                        pow((odom_msgs->pose.pose.position.y - y_old), 2));

    if(diff > min_distance)
    {
        RCLCPP_INFO(this->node_->get_logger(), "Received odometry message: x = %.2f, y = %.2f",
                     odom_msgs->pose.pose.position.x, odom_msgs->pose.pose.position.y);

         //auto new_point_shared = std::make_shared<geometry_msgs::msg::Point>(new_point);

        this->write_waypoint_to_csv(odom_msgs);
        this->update_waypoint_marker_callback(odom_msgs); 
        this->public_waypoint_marker(waypoint_marker_pub_);
        

        // update old positions
        this->x_old = odom_msgs->pose.pose.position.x;
        this->y_old = odom_msgs->pose.pose.position.y;
        

    }
}

void OdomWaypoint::public_waypoint_marker(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& waypoint_marker_pub_)
{
    const auto& waypoint_marker = this->get_waypoint_marker();
    waypoint_marker_pub_->publish(waypoint_marker);
}



OdomWaypoint::~OdomWaypoint()
{

}
