#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

class PublisherNode : public rclcpp::Node {
public:
    PublisherNode() : Node("initial_pose_pub_node") {
        // Create a publisher for PoseWithCovarianceStamped
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
        
        // Create a subscription to PointStamped
        subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point",
            1,
            std::bind(&PublisherNode::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received Data:\n X : %f \n Y : %f \n Z : %f", 
                    msg->point.x, msg->point.y, msg->point.z);
        publish(msg->point.x, msg->point.y);
    }

    void publish(double x, double y) {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();

        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;

        RCLCPP_INFO(this->get_logger(), "Publishing Initial Position\n X= %f \n Y= %f", 
                    msg.pose.pose.position.x, msg.pose.pose.position.y);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
