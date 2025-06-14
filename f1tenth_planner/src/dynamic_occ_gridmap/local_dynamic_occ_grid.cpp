#include "f1tenth_planner/local_dynamic_occ_grid.hpp"


DynamicOccupancyGrid::DynamicOccupancyGrid(const std::string &name): Node(name)
{
        // Khai báo tham số
    this->declare_parameter<double>("width", 8.0); // meters
    this->declare_parameter<double>("height", 4.0);
    this->declare_parameter<double>("resolution", 0.1);
    this->declare_parameter<double>("log_odds_free", -0.4); // Log-odds decrement for free space
    this->declare_parameter<double>("log_odds_occupied", 0.85); // Log-odds increment for occupied space
    this->declare_parameter<double>("log_odds_min", -2.0); // Minimum log-odds
    this->declare_parameter<double>("log_odds_max", 3.5); // Maximum log-odds
    this->declare_parameter<double>("decay_rate", 0.1); // Decay rate per timer callback
  
    this->declare_parameter<bool>("enable_logging", false);
    enable_logging_ = this->get_parameter("enable_logging").as_bool();

        // Lấy giá trị tham số
    double width = this->get_parameter("width").as_double();
    double height = this->get_parameter("height").as_double();
    double resolution = this->get_parameter("resolution").as_double();


        // Khởi tạo OccupancyGrid
    map_.info.resolution = resolution;
    map_.info.width = static_cast<uint32_t>(std::round(width / resolution));
    map_.info.height = static_cast<uint32_t>(std::round(height / resolution));
    map_.info.origin.position.x = -width / 2.0;
    map_.info.origin.position.y = -height / 2.0;
    map_.info.origin.position.z = 0.0;

        // Thiết lập quaternion (no rotation)
    map_.info.origin.orientation.x = 0.0;
    map_.info.origin.orientation.y = 0.0;
    map_.info.origin.orientation.z = 0.0;
    map_.info.origin.orientation.w = 1.0;

    map_.header.frame_id = "base_link"; // Local map frame
    map_.data.assign(map_.info.width * map_.info.height, -1); // Initialize as unknown

    log_odds_grid_.resize(map_.info.height, std::vector<double>(map_.info.width, 0.0));

    // Lấy các tham số log-odds
    log_odds_free_ = this->get_parameter("log_odds_free").as_double();
    log_odds_occupied_ = this->get_parameter("log_odds_occupied").as_double();
    log_odds_min_ = this->get_parameter("log_odds_min").as_double();
    log_odds_max_ = this->get_parameter("log_odds_max").as_double();
    decay_rate_ = this->get_parameter("decay_rate").as_double();


        // Khởi tạo publisher và subscriber
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap", 1);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10, std::bind(&DynamicOccupancyGrid::scan_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DynamicOccupancyGrid::timer_callback, this));


        // Định nghĩa transform tĩnh giữa base_link và laser_link
    laser_to_base_translation_ = {0.285, 0.0, 0.099}; // (x, y, z) in meters
    laser_to_base_rotation_ = {0.0, 0.0, 0.0, 1.0}; // (x, y, z, w) quaternion

    // Khởi tạo chỉ số góc
    angle_indices_ = get_index_from_angle(-80, 80);

}



// Hàm chuyển đổi tọa độ thành Pose
Pose DynamicOccupancyGrid::coordinatesToPose(double px, double py, const nav_msgs::msg::MapMetaData &map_info) 
{
    int x = static_cast<int>(std::round((px - map_info.origin.position.x) / map_info.resolution));
    int y = static_cast<int>(std::round((py - map_info.origin.position.y) / map_info.resolution));
    return Pose(x, y);
}

// Hàm kiểm tra Pose có nằm trong map không
bool DynamicOccupancyGrid::poseOnMap(const Pose &pose, const nav_msgs::msg::MapMetaData &map_info) 
{
    return (pose.x >= 0 && pose.x < static_cast<int>(map_info.width) &&
            pose.y >= 0 && pose.y < static_cast<int>(map_info.height));
}

// Hàm chuyển Pose thành chỉ số ô trong map
int DynamicOccupancyGrid::poseToCell(const Pose &pose, const nav_msgs::msg::MapMetaData &map_info) 
{
    return static_cast<int>(map_info.width * pose.y + pose.x);
}

// Hàm lấy chỉ số từ góc
std::unordered_map<std::string, int> DynamicOccupancyGrid::get_index_from_angle(int start_angle, int end_angle, 
    std::unordered_map<std::string, double> lidar_param)
{
    int start_index = static_cast<int>((start_angle - lidar_param["angle_min"]) / lidar_param["angle_increment"]);
    int end_index = static_cast<int>((end_angle - lidar_param["angle_min"]) / lidar_param["angle_increment"]);

    return {{"start_angle_index", start_index}, {"end_angle_index", end_index}};
}



// Callback cho LaserScan
void DynamicOccupancyGrid::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) 
{
    // Reset map data
    // std::fill(map_.data.begin(), map_.data.end(), -1);

    // Chuyển đổi quaternion sang Euler angles để lấy yaw
    double yaw;
    try {
        tf2::Quaternion q(
            laser_to_base_rotation_[0],
            laser_to_base_rotation_[1],
            laser_to_base_rotation_[2],
            laser_to_base_rotation_[3]
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting quaternion to euler: %s", e.what());
        return;
    }

    // Vị trí robot tại (0,0) trong frame base_link
    Pose robot_p(0, 0);

    if (!poseOnMap(robot_p, map_.info)) {
        RCLCPP_ERROR(this->get_logger(), "The robot is out of the map");
        return;
    }

    // Xử lý từng tia laser trong khoảng góc đã định
    for (int i = angle_indices_["start_angle_index"]; i <= angle_indices_["end_angle_index"]; ++i) 
    {
        if (i < 0 || i >= static_cast<int>(scan->ranges.size())) {
            continue;
        }

        double range = scan->ranges[i];
        if (std::isinf(range) || std::isnan(range) || range > 5.0) {
            continue;
        }

        // Tính góc với yaw của robot
        double angle = scan->angle_min + (i * scan->angle_increment) + yaw;

        // Tính vị trí chướng ngại vật trong frame base_link
        double px = range * std::cos(angle);
        double py = range * std::sin(angle);

        // Áp dụng transform tĩnh từ laser_link đến base_link
        double px_base = px + laser_to_base_translation_[0];
        double py_base = py + laser_to_base_translation_[1];

        // Chuyển đổi sang chỉ số lưới map
        Pose beam_p = coordinatesToPose(px_base, py_base, map_.info);
        if (!poseOnMap(beam_p, map_.info)) {
            continue;
        }

        // Cập nhật log-odds
        double &log_odds = log_odds_grid_[beam_p.y][beam_p.x];
        log_odds += log_odds_occupied_;

        // Giới hạn log-odds
        log_odds = std::max(log_odds_min_, std::min(log_odds, log_odds_max_));
    }
}

void DynamicOccupancyGrid::timer_callback() {
    // Áp dụng suy giảm log-odds
    for (auto &row : log_odds_grid_) {
        for (auto &cell : row) {
            cell += log_odds_free_;
            cell = std::max(log_odds_min_, std::min(cell, log_odds_max_));
        }
    }

    // Chuyển đổi log-odds thành xác suất chiếm dụng
    std::vector<int8_t> occupancy_prob;
    occupancy_prob.reserve(map_.info.width * map_.info.height);

    for (size_t y = 0; y < log_odds_grid_.size(); ++y) {
        for (size_t x = 0; x < log_odds_grid_[y].size(); ++x) {
            double cell_log_odds = log_odds_grid_[y][x];
            if (cell_log_odds == 0.0) {
                occupancy_prob.push_back(-1); // Unknown
            }
            else {
                double prob = 1.0 - 1.0 / (1.0 + std::exp(cell_log_odds));
                int8_t prob_scaled = static_cast<int8_t>(std::round(prob * 100.0));
                prob_scaled = std::clamp(prob_scaled, static_cast<int8_t>(0), static_cast<int8_t>(100));    // std::clamp C++17
                occupancy_prob.push_back(prob_scaled);
            }
        }
    }

    // Cập nhật dữ liệu OccupancyGrid
    map_.data = occupancy_prob;

    if (enable_logging_) {
        // Đếm số lượng ô chiếm dụng, tự do và không xác định
        int occupied = 0;
        int free = 0;
        int unknown = 0;

        for (const auto &cell : map_.data) {
            if (cell > 50) { // Giả định >50 là chiếm dụng
                occupied++;
            } else if (cell >= 0 && cell <= 50) { // Giả định 0-50 là tự do
                free++;
            } else { // -1 là không xác định
                unknown++;
            }
        }

        RCLCPP_INFO(this->get_logger(), " - Occupied cells: %d", occupied);
        RCLCPP_INFO(this->get_logger(), " - Free cells: %d", free);
        RCLCPP_INFO(this->get_logger(), " - Unknown cells: %d", unknown);
    }
    // Cập nhật tiêu đề với thời gian hiện tại
    map_.header.stamp = this->now();

    // Xuất bản bản đồ
    map_pub_->publish(map_);
}