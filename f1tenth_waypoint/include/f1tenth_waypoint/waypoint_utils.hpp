#ifndef F1TENTH_WAYPOINT_FOLLOWER__WAYPOINT_UTILS_HPP_
#define F1TENTH_WAYPOINT_FOLLOWER__WAYPOINT_UTILS_HPP_

#include <iostream>
#include <filesystem> 
#include <fstream>

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <memory>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 

// using namespace std::filesystem;
class CSVFile
{
    public:
        CSVFile(std::shared_ptr<rclcpp::Node> rclcpp_node) ;
        void write_points_to_csv(const geometry_msgs::msg::Point::ConstSharedPtr waypoint_msg_);
        void write_points_to_csv(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_);

    protected: 

    private:
        void initialize_parameters();
        void initialize_csv(std::string path_to_csv);
        

    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::string csv_path;
        std::string csvFile_name;
        std::string path_to_csvFile;

};


class Waypoint_Visualization
{
    public:
        Waypoint_Visualization(std::shared_ptr<rclcpp::Node> rclcpp_node);
        ~Waypoint_Visualization();
        void update_waypoint_marker_callback(const geometry_msgs::msg::Point::ConstSharedPtr waypoint_msg_);
        void update_waypoint_marker_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_);
        visualization_msgs::msg::Marker &get_waypoint_marker();
    
    protected: 
    private:
        void initialize_marker();
    
    private:
        std::shared_ptr<rclcpp::Node> node_;
        visualization_msgs::msg::Marker waypoint_marker_;
};


class UtilsWaypoint
{
    public:  
        UtilsWaypoint(std::shared_ptr<rclcpp::Node> rclcpp_node);
        ~UtilsWaypoint();

        void write_waypoint_to_csv(const geometry_msgs::msg::Point::ConstSharedPtr waypoint_msg_);
        void write_waypoint_to_csv(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_);
        void update_waypoint_marker_callback(const geometry_msgs::msg::Point::ConstSharedPtr waypoint_msg_);
        void update_waypoint_marker_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_);

        visualization_msgs::msg::Marker &get_waypoint_marker() ;
    
    protected:
        // void write_waypoint_to_csv(const geometry_msgs::msg::Point::ConstSharedPtr waypoint_msg_);
        // void write_waypoint_to_csv(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_);
        // void update_waypoint_marker_callback(const geometry_msgs::msg::Point::ConstSharedPtr waypoint_msg_);
        // void update_waypoint_marker_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_);
       // visualization_msgs::msg::Marker UtilsWaypoint::get_waypoint_marker() const;
// {
//     return this->waypoint_visualization;
// }

    private: 
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<CSVFile> csv_file;
        std::shared_ptr<Waypoint_Visualization> waypoint_visualization;

};

#endif  