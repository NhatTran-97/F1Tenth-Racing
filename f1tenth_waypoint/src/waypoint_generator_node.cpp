#include "f1tenth_waypoint/waypoint_generator_node.hpp"

Waypoint::Waypoint(const std::string &name) : Node(name)
{

    // Set up the waypoint type and topic
    std::tie(message_type, waypoint_topic) = this->setup_waypoint_option();

    // Create subscriptions based on the message type
    if (!message_type.empty() && !waypoint_topic.empty())
    {
        if (message_type == "PointStamped")
        {
            // Initialize the subscription for PointStamped
            clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                waypoint_topic, 10, std::bind(&Waypoint::waypoint_callback, this, std::placeholders::_1));
        }
        else if (message_type == "Odometry")
        {
            // Initialize the subscription for Odometry
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                waypoint_topic, 10, std::bind(&Waypoint::odom_callback, this, std::placeholders::_1));
        }
    }

    // Optionally, set up a publisher for markers
    waypoint_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/f1tenth_waypoint_marker", 10);
}

void Waypoint::initialize()
{
        // Initialize ClickedWaypoint and odom_waypoint_ with the current node instance

    // odom_waypoint_ = std::make_shared<OdomWaypoint>(this->shared_from_this());
    // clicked_waypoint_ = std::make_shared<ClickedWaypoint>(this->shared_from_this());


    if (message_type == "Odometry")
    {
        // Only initialize OdomWaypoint
        odom_waypoint_ = std::make_shared<OdomWaypoint>(this->shared_from_this());
        RCLCPP_INFO(this->get_logger(), "Initialized OdomWaypoint");
    }
    else if (message_type == "PointStamped")
    {
        // Only initialize ClickedWaypoint
        clicked_waypoint_ = std::make_shared<ClickedWaypoint>(this->shared_from_this());
        RCLCPP_INFO(this->get_logger(), "Initialized ClickedWaypoint");
    }
}

std::pair<std::string, std::string> Waypoint::setup_waypoint_option()
{
    std::string odom_topic;
    std::string clicked_point_topic;
    std::string waypoint_option;

    try
    {
            // Declare the parameters

        if (!this->has_parameter("odom_topic"))
        {
            this->declare_parameter<std::string>("odom_topic", "/odom");
        }

        if (!this->has_parameter("clicked_point_topic"))
        {
            this->declare_parameter<std::string>("clicked_point_topic", "/clicked_point");
        }

        if (!this->has_parameter("waypoint_options"))
        {
            this->declare_parameter<std::string>("waypoint_options", "clicked_waypoint"); // clicked_waypoint   odom_waypoint
        }
        
        // Get the parameters
        odom_topic = this->get_parameter("odom_topic").as_string();
        clicked_point_topic = this->get_parameter("clicked_point_topic").as_string();
        waypoint_option = this->get_parameter("waypoint_options").as_string();

    }
    catch(const rclcpp::exceptions::ParameterNotDeclaredException &e) 
    {
        RCLCPP_WARN(this-> get_logger(), "Could not get parameters ... setting variables to default");
        RCLCPP_WARN(this-> get_logger(), "Error: %s", e.what());
    }
    catch(const std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "An error occured: %s", e.what());
        
    }
    



    // Set the message type and topic based on the waypoint type
    if (waypoint_option == this->CLICKED_WAYPOINT)
    {
        RCLCPP_INFO(this->get_logger(), "Using CLICKEDPOINT_WAYPOINT option");

        return {"PointStamped", clicked_point_topic};
    }
    else if (waypoint_option == this->ODOM_WAYPOINT)
    {
        RCLCPP_INFO(this->get_logger(), "Using ODOM_WAYPOINT option");
        return {"Odometry", odom_topic};
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Unknown waypoint option, using default");
        return {"", ""};
    }
}

 //rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rvizPoint_pub_;

void Waypoint::waypoint_callback(const geometry_msgs::msg::PointStamped::SharedPtr waypoint_msgs)
{
    if (waypoint_msgs != nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Received PointStamped message: x = %.2f, y = %.2f, z = %.2f",
                    waypoint_msgs->point.x, waypoint_msgs->point.y, waypoint_msgs->point.z);

        clicked_waypoint_->clicked_point_callback(waypoint_msgs, waypoint_marker_pub_);
    }
}

void Waypoint::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msgs)
{
    if (odom_msgs != nullptr)
    {
        RCLCPP_INFO(get_logger(), "Received Odometry message: x = %.2f, y = %.2f",
                    odom_msgs->pose.pose.position.x, odom_msgs->pose.pose.position.y);
        odom_waypoint_->odom_callback(odom_msgs, waypoint_marker_pub_);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto waypoint_node = std::make_shared<Waypoint>("waypoint_generator_node");
    waypoint_node->initialize();
    rclcpp::spin(waypoint_node);
    rclcpp::shutdown();
    return 0;
}
