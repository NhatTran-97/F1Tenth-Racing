#ifndef LOCAL_DYNAMIC_OCC_GRID_HPP
#define LOCAL_DYNAMIC_OCC_GRID_HPP

#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_eigen/tf2_eigen.h"

struct Pose{
    int x;
    int y;
    Pose(int px = 0, int py = 0) : x(px), y(py) {}
};

class DynamicOccupancyGrid : public rclcpp::Node
{
    public:
        explicit DynamicOccupancyGrid(const std::string &name);
    
    private:
        // Hàm chuyển đổi tọa độ thành Pose
        Pose coordinatesToPose(double px, double py, const nav_msgs::msg::MapMetaData &map_info);

         // Hàm kiểm tra Pose có nằm trong map không
        bool poseOnMap(const Pose &pose, const nav_msgs::msg::MapMetaData &map_info);

         // Hàm chuyển Pose thành chỉ số ô trong map
        int poseToCell(const Pose &pose, const nav_msgs::msg::MapMetaData &map_info);


         // Hàm lấy chỉ số từ góc
        std::unordered_map<std::string, int> get_index_from_angle(int start_angle, int end_angle, 
            std::unordered_map<std::string, double> lidar_param = {{"angle_min", -135.0}, {"angle_increment", 0.25}});


        // Callback cho LaserScan
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

        // Callback cho Timer
        void timer_callback();

        
    private:
        nav_msgs::msg::OccupancyGrid map_;
        std::vector<std::vector<double>> log_odds_grid_;
        double log_odds_free_;
        double log_odds_occupied_;
        double log_odds_min_;
        double log_odds_max_;
        double decay_rate_;
        bool enable_logging_;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::vector<double> laser_to_base_translation_; // (x, y, z)
        std::vector<double> laser_to_base_rotation_; // (x, y, z, w)
        std::unordered_map<std::string, int> angle_indices_;

};





#endif



