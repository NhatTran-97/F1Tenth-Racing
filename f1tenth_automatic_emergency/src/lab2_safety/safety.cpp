#include "f1tenth_automatic_emergency/safety.hpp"

using std::placeholders::_1;



Safety::Safety(const std::string &name): Node(name), ttc_threshold(0.0), speed(0.0)
{
    declare_parameter<double>("ttc_threshold", 0.6);
    ttc_threshold = get_parameter("ttc_threshold").as_double();

    this->odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&Safety::odom_callback, this, _1));
    this->laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Safety::scan_callback, this, _1));
    this->safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>("/emergency_breaking", 10);
    this->drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
}


void Safety::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    this->speed =  odom_msg->twist.twist.linear.x;
}

void Safety::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    bool emergency_breaking = false;
    
    for (std::size_t i = 0; i < scan_msg->ranges.size(); i++)
    {
       double r = scan_msg->ranges[i];
       if (std::isnan(r) || r > scan_msg->range_max || r < scan_msg->range_min)
       {
        continue;
       }

       if (r / std::max(this->speed * std::cos(scan_msg->angle_min + (double)i * scan_msg->angle_increment), 0.001) < ttc_threshold)
       {
            emergency_breaking = true;
            break;
       }
        
    }
    auto emergency_msg = std_msgs::msg::Bool();
    emergency_msg.data = emergency_breaking;
    this->safety_stop_pub_->publish(emergency_msg);

    
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Safety>("safety_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}