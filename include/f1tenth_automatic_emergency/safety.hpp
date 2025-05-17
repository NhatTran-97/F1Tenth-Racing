#ifndef SAFETY_HPP
#define SAFETY_HPP

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Safety : public rclcpp::Node
{
    public:
        Safety(const std::string &name);

    private:
        void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
        void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

        double ttc_threshold;
        double speed;

};


#endif