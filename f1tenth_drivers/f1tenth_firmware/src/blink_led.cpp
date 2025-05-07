#include "rclcpp/rclcpp.hpp"
#include <JetsonGPIO.h>
#include "sensor_msgs/msg/joy.hpp"

class BlinkLed : public rclcpp::Node 
{
public: 
    BlinkLed():Node("blink_led"), led_pin(13), led_state(false)
    {
        try 
        {
            ledInit();
            sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10, std::bind(&BlinkLed::joy_callback, this, std::placeholders::_1));
        } 
        catch (const std::runtime_error& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Runtime error during initialization: %s", e.what());
        }
    }

    void ledInit()
    {
        try {
            GPIO::setmode(GPIO::BOARD);
            GPIO::setwarnings(false);
            GPIO::setup(led_pin, GPIO::OUT, GPIO::HIGH);
        } 
        catch (const std::runtime_error& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Runtime error during GPIO setup: %s", e.what());
            throw;
        }
    }

    ~BlinkLed()
    {
        GPIO::cleanup();
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        try 
        {
            // if(msg->buttons[2] = 1)
            // {

            // }

            if(msg->buttons[2] == 1)
            {
                led_state = !led_state;
                GPIO::output(led_pin, led_state ? GPIO::HIGH : GPIO::LOW);
                RCLCPP_INFO(this->get_logger(), "LED is %s", led_state ? "ON" : "OFF"); 
            }
            else
            {
                GPIO::output(led_pin,GPIO::HIGH );
            }
        } 
        catch (const std::runtime_error& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Runtime error during GPIO output: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
    int led_pin;
    bool led_state;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try 
    {
        auto node = std::make_shared<BlinkLed>();
        rclcpp::spin(node);
    } catch (const std::runtime_error& e) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Runtime error: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
