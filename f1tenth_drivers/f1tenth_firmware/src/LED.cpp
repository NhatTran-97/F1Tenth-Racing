#include "f1tenth_firmware/LED.hpp"


namespace f1tenth_gpio
{
    Led::Led() : Node("blink_led"),led_pin(13), led_state(false)
    {
        try
        {
        this->ledInit();
        this->joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
                                "joy", 10, std::bind(&Led::joy_callback, this, std::placeholders::_1));
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void Led::ledInit(void)
    {
        try 
        {
            GPIO::setmode(GPIO::BOARD);
            GPIO::setwarnings(false);
            GPIO::setup(this->led_pin, GPIO::OUT, GPIO::HIGH);
        } 
        catch (const std::runtime_error& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Runtime error during GPIO setup: %s", e.what());
            throw;
        }
    }

    void Led::joy_callback(sensor_msgs::msg::Joy::SharedPtr msg)
    {
       bool is_pressed = msg->buttons[2];

       try
       {
            if(is_pressed == 1)
            {
                led_state= !led_state;
                GPIO::output(this->led_pin, led_state? GPIO::HIGH : GPIO::LOW);
              //  RCLCPP_INFO(this->get_logger(), "LED is %s", led_state? "ON" : "OFF"); 
            }
            else
            {
                GPIO::output(this->led_pin,GPIO::HIGH );
            }

       }
        catch (const std::runtime_error& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Runtime error during GPIO output: %s", e.what());
        }
    }

    Led::~Led()
    {
        GPIO::cleanup();
    }
}