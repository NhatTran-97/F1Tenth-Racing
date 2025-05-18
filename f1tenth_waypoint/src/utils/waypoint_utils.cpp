#include "f1tenth_waypoint/waypoint_utils.hpp"


CSVFile::CSVFile(std::shared_ptr<rclcpp::Node> rclcpp_node) : node_(rclcpp_node)
{
    this->initialize_parameters();

    /*std::filesystem::current_path()  return /home/fablab_01*/
    path_to_csvFile = std::filesystem::current_path() / this->csv_path / csvFile_name;

    RCLCPP_INFO(this->node_->get_logger(), "full_path %s", path_to_csvFile.c_str());

    this->initialize_csv(path_to_csvFile);


}


void CSVFile::initialize_parameters()
{
    try
    {

        // Check and declare parameters if they don't exist
        if (!node_->has_parameter("path_to_csvFile")) {
            this->node_->declare_parameter<std::string>("path_to_csvFile", "/home/fablab_01/f1_ws/src/f1tenth_projects/f1tenth_waypoint/racelines");
        }

        if (!node_->has_parameter("csvFile_name")) {
            this->node_->declare_parameter<std::string>("csvFile_name", "f1tenth_waypoint_path.csv");
        }
        // Retrieve parameters

        csv_path = this->node_->get_parameter("path_to_csvFile").as_string();
        csvFile_name = this->node_->get_parameter("csvFile_name").as_string();

        RCLCPP_INFO(this->node_ ->get_logger(), "CSV Path: %s", csv_path.c_str());
        RCLCPP_INFO(this->node_ ->get_logger(), "csvfile name %s", csvFile_name.c_str());

        }
        catch(const rclcpp::exceptions::ParameterNotDeclaredException &e) 
        {
            RCLCPP_WARN(this->node_ -> get_logger(), "Could not get parameters ... setting variables to default");
            RCLCPP_WARN(this->node_ -> get_logger(), "Error: %s", e.what());
        }
        catch(const std::exception &e)
        {
            RCLCPP_WARN(this->node_ ->get_logger(), "An error occured: %s", e.what());
            
        }

        

}

void CSVFile::initialize_csv(std::string path_to_csv_)
{
    // ofstream FileOut;
    // std::string header_name;

    std::ofstream csv_file(path_to_csv_, std::ios::out | std::ios::trunc);
    if(!csv_file.is_open())
    {
        std::cerr << "Failed to create or open CSV file " << path_to_csv_ << std::endl;
        return;
    }
    RCLCPP_INFO(this->node_ ->get_logger(), "Creating or overwriting CSV file at: %s", path_to_csv_);
    csv_file << "x, y, yaw" << std::endl;
    csv_file.close();
}


void CSVFile::write_points_to_csv(const geometry_msgs::msg::Point::ConstSharedPtr waypoint_msg_)
{
    std::ofstream csv_file(path_to_csvFile, std::ios::app);
    if(!csv_file.is_open())
    {
        std::cerr << "Failed to open CSV file: " << path_to_csvFile << std::endl;
        return;
    }
    double x = waypoint_msg_->x;
    double y = waypoint_msg_->y;
    double yaw = waypoint_msg_->z;
    std::cout << "Writing Clicked Point: x=" << x << ", y=" << y << ", yaw=" << yaw << std::endl;
    csv_file << x << "," << y << "," << yaw << std::endl;
    csv_file.close();


}
void CSVFile::write_points_to_csv(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_)
{
    std::ofstream csv_file(path_to_csvFile, std::ios::app);
    if(!csv_file.is_open())
    {
        std::cerr << "Failed to open CSV file: " << path_to_csvFile << std::endl;
        return;
    }

    double x = odom_msg_->pose.pose.position.x;
    double y = odom_msg_->pose.pose.position.y;

    // Extract quaternion and convert to yaw

    const geometry_msgs::msg::Quaternion &quat_msg = odom_msg_->pose.pose.orientation;

    tf2::Quaternion quat;
    tf2::fromMsg(quat_msg, quat); tf2::Matrix3x3 mat(quat);
    
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);



    std::cout << "Writing Odometry: x=" << x << ", y=" << y << ", yaw=" << yaw << std::endl;

    // Write row to CSV
    csv_file << x << "," << y << "," << yaw << std::endl;
    csv_file.close();


}


Waypoint_Visualization::Waypoint_Visualization(std::shared_ptr<rclcpp::Node> rclcpp_node) : node_(rclcpp_node)
{
    this->initialize_marker();
}

Waypoint_Visualization::~Waypoint_Visualization()
{

}


void Waypoint_Visualization::initialize_marker()
{
    this->waypoint_marker_.header.frame_id = "map";
    this->waypoint_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    this->waypoint_marker_.action = visualization_msgs::msg::Marker::ADD;

    this->waypoint_marker_.scale.x = 0.05;
    this->waypoint_marker_.scale.y = 0.1;
    this->waypoint_marker_.scale.z = 0.1;

    this->waypoint_marker_.color.a = 1.0;
    this->waypoint_marker_.color.r = 0.0;
    this->waypoint_marker_.color.g = 1.0;
    this->waypoint_marker_.color.b = 0.0;
}

void Waypoint_Visualization::update_waypoint_marker_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_)
{
    geometry_msgs::msg::Point point;
    point.x = odom_msg_->pose.pose.position.x;
    point.y = odom_msg_->pose.pose.position.y;
    point.z = 0.0;

    this->waypoint_marker_.points.push_back(point);

}

 void Waypoint_Visualization::update_waypoint_marker_callback(const geometry_msgs::msg::Point::ConstSharedPtr waypoint_msg_)
 {
    this->waypoint_marker_.points.push_back(*waypoint_msg_);
}


visualization_msgs::msg::Marker &Waypoint_Visualization::get_waypoint_marker()
{
    return waypoint_marker_;
}



/*===================================================================*/

UtilsWaypoint::UtilsWaypoint(std::shared_ptr<rclcpp::Node> rclcpp_node) : node_(rclcpp_node)
{
   csv_file = std::make_shared<CSVFile>(rclcpp_node);
   waypoint_visualization = std::make_shared<Waypoint_Visualization>(rclcpp_node);
}


void UtilsWaypoint::write_waypoint_to_csv(const geometry_msgs::msg::Point::ConstSharedPtr waypoint_msg_)
{
   this->csv_file -> write_points_to_csv(waypoint_msg_);
}
void UtilsWaypoint::write_waypoint_to_csv(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_)
{
    this->csv_file -> write_points_to_csv(odom_msg_);
}


void UtilsWaypoint::update_waypoint_marker_callback(const geometry_msgs::msg::Point::ConstSharedPtr waypoint_msg_)
{
    this->waypoint_visualization -> update_waypoint_marker_callback(waypoint_msg_);
}
void UtilsWaypoint::update_waypoint_marker_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_)
{
    this->waypoint_visualization -> update_waypoint_marker_callback(odom_msg_);
}


// visualization_msgs::msg::Marker UtilsWaypoint::get_waypoint_marker() const
// {
//     return this->waypoint_visualization;
// }


visualization_msgs::msg::Marker &UtilsWaypoint::get_waypoint_marker()
{
    // Delegate the call to the instance of Waypoint_Visualization
    return waypoint_visualization->get_waypoint_marker();
}


UtilsWaypoint::~UtilsWaypoint()
{

}

