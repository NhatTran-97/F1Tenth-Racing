#include "f1tenth_firmware/LED.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try 
    {
        auto node = std::make_shared<f1tenth_gpio::Led>();
        
        rclcpp::spin(node);

    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    rclcpp::shutdown();
    return 0;
}
