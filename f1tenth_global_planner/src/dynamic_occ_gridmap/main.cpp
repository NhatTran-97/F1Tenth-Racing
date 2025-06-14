#include "f1tenth_planner/local_dynamic_occ_grid.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicOccupancyGrid>("local_dynamic_occ_grid");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}