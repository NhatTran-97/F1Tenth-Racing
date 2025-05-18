
#ifndef F1TENTH_FIRMWARE__LED_HPP_
#define F1TENTH_FIRMWARE__LED_HPP_

#include "rclcpp/rclcpp.hpp"
#include <JetsonGPIO.h>
#include "sensor_msgs/msg/joy.hpp"

namespace f1tenth_gpio
{
    class Led : public rclcpp::Node
    {
        public:
            explicit Led();
            ~Led();

        private:
            void ledInit(void);
            void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg);
        
        private:
            int led_pin;
            bool led_state;
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    };
}



#endif
