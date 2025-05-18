#include "waypoint_publisher/waypoint_publisher.hpp"
using namespace std::chrono_literals;
using namespace std;

WaypointPublisher::WaypointPublisher(const std::string &name) : Node(name)
{
    try
    {
        if (!this->has_parameter("csvFile_name"))
        {
            this->declare_parameter<string>("csvFile_name", "/f1tenth_waypoint_path.csv");
        }
        centerline_file_name = this->get_parameter("csvFile_name").as_string();

        RCLCPP_INFO(this->get_logger(), "CSV name file: %s", centerline_file_name);

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

    raceline_pub_ = this->create_publisher<nav_msgs::msg::Path>("raceline_waypoints", 10);
    centerline_pub_ = this->create_publisher<nav_msgs::msg::Path>("fablab_waypoints", 10);
    blend_pub_ = this->create_publisher<nav_msgs::msg::Path>("blend_waypoints", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&WaypointPublisher::timer_callback, this));


    centerline_file_loc = this->get_file_loc(centerline_file_name);
    RCLCPP_INFO(this->get_logger(), "Center file location: %s", centerline_file_loc.c_str());

    centerline_path = this->get_waypoints(this->centerline_file_loc, this->centerline_file_name);
    RCLCPP_INFO(this->get_logger(), "Loaded %d centerline waypoints", (int)centerline_path.poses.size());
    this->timer_callback();
}

void WaypointPublisher::timer_callback()
{
    static bool raceline_logged = false;
    static bool centerline_logged = false;
    static bool blend_logged = false;

    if (raceline_path.poses.size() > 0)
    {
        raceline_pub_->publish(raceline_path);
        if(!raceline_logged)
        {
            RCLCPP_INFO(this->get_logger(), "Publishing %d raceline waypoints", (int)raceline_path.poses.size());
            raceline_logged = true;
        }
    }

    if (centerline_path.poses.size() > 0)
    {
        centerline_pub_->publish(centerline_path);
        if(!centerline_logged)
        {
            RCLCPP_INFO(this->get_logger(), "Publishing %d centerline waypoints", (int)centerline_path.poses.size());
            centerline_logged = true;
        }
        
    }
    if (blend_path.poses.size() > 0)
    {
        blend_pub_->publish(blend_path);
        if(!blend_logged)
        {
            RCLCPP_INFO(this->get_logger(), "Publishing %d blend waypoints", (int)blend_path.poses.size());
            blend_logged = true;
        }
        
    }
   
}

std::string WaypointPublisher::get_file_loc(std::string file_name) /*Gets the absolute path of our waypoint file*/
{
    return std::string((std::string)ament_index_cpp::get_package_share_directory("f1tenth_waypoint") + "/racelines" + file_name);
}

std::vector<double> WaypointPublisher::get_orientation(geometry_msgs::msg::PoseStamped pose1, geometry_msgs::msg::PoseStamped pose2)
{
    /*
    atan2: Tính toán arctangent (arc-tangent) của hai giá trị => trả về góc quay yaw giữa 2 điểm (pose1, pose2)
    z = sin(yaw / 2.0)
    w = cos(yaw / 2.0)
    */
    double yaw = atan2(pose2.pose.position.y - pose1.pose.position.y, pose2.pose.position.x - pose1.pose.position.x);
    return {0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0)};
}

nav_msgs::msg::Path WaypointPublisher::get_waypoints(std::string filepath, std::string filename)
{
    /*
    // read .csv waypoint file and convert it to a Path object
    */
    nav_msgs::msg::Path new_path;
    new_path.header.frame_id = "map";
    new_path.header.stamp = this->now();

    FILE *fp;
    fp = fopen(filepath.c_str(), "r");
    if (fp == NULL)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filepath.c_str());
        return new_path;
    }
    enum Parse_type
    {
        CENTERLINE,
        MINCURV,
        RACELINE,
        NOW
    };

    Parse_type line_type = NOW;

    RCLCPP_INFO(this->get_logger(), "File name: %s", filename.c_str());

    if (filename.find("f1tenth_waypoint") != std::string::npos || filename.find("path") != std::string::npos)
    {
        line_type = CENTERLINE;
        RCLCPP_INFO(this->get_logger(), "Detected centerline or path from filename: %s", filename.c_str());
    }

    else if (filename.find("mincurv") != std::string::npos)
    {
        line_type = MINCURV;
        RCLCPP_INFO(this->get_logger(), "Detected mincurv from filename %s", filename.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Failed to detect line type from filename: %s", filename.c_str());
    }

    char buff[256];

    bool first_row = true;
    while (fgets(buff, sizeof(buff), fp) != NULL) // char *fgets(char *str, int n, FILE *stream);
    {
        //RCLCPP_INFO(this->get_logger(), "File content: %s", buff);

        if(first_row)
        {
            first_row = false;
            RCLCPP_INFO(this->get_logger(), "Skipping header row: %s", buff);
            continue;
        }

        double x, y, yaw, v;

        if (line_type == CENTERLINE)
        {
            /*Dinh dang fl: double, ",": Cac gia tritrong chuoi tach nhau boi dau phay*/

            sscanf(buff, "%lf, %lf, %lf", &x, &y, &yaw); // int sscanf(const char *str, const char *format, ...);
            
            RCLCPP_INFO(this->get_logger(), "CENTERLINE %f %f %f ", x, y, yaw);
        //    RCLCPP_INFO(this->get_logger(), "CENTERLINE");
            v = 0;
        }
        else if (line_type == MINCURV)
        {
            // sscanf(buff, "%lf, %lf, %lf", &x, &y, &v);
            sscanf(buff, "%*f;%lf;%*f;%lf;%lf", &x, &y, &v);
            RCLCPP_INFO(this->get_logger(), "MINCURV");
        }
        else if (line_type == RACELINE)
        {
            // sscanf(buff, "%*f;%lf;%*f;%lf", &x, &y, &v);
            sscanf(buff, "%*f;%lf;%*f;%lf;%lf", &x, &y, &v);
            RCLCPP_INFO(this->get_logger(), "RACELINE");
        }
        else if (line_type == NOW)
        {
            sscanf(buff, "%*f %lf %lf", &x, &y);
            RCLCPP_INFO(this->get_logger(), "NOW");
            v = 0;
        }
        geometry_msgs::msg::PoseStamped pose;
        pose.header = new_path.header;
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = v;

        // orientation
        if (new_path.poses.size() > 0)
        {
            std::vector<double> orientation = this->get_orientation(new_path.poses[new_path.poses.size() - 1], pose);
            pose.pose.orientation.x = orientation[0];
            pose.pose.orientation.y = orientation[1];
            pose.pose.orientation.z = orientation[2];
            pose.pose.orientation.w = orientation[3];
        }
        else
        {
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
        }
        new_path.poses.push_back(pose);
        /*
        const auto &added_pose = new_path.poses.back(); // Lấy phần tử vừa được thêm
        RCLCPP_INFO(this->get_logger(), "New Pose added to new_path:");
        RCLCPP_INFO(this->get_logger(), "  Position: x = %f, y = %f, z = %f",
                    added_pose.pose.position.x, added_pose.pose.position.y, added_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "  Orientation: x = %f, y = %f, z = %f, w = %f",
                    added_pose.pose.orientation.x, added_pose.pose.orientation.y, added_pose.pose.orientation.z, added_pose.pose.orientation.w);
        */
    }

    return new_path;
}

nav_msgs::msg::Path WaypointPublisher::blend_paths(nav_msgs::msg::Path path1, nav_msgs::msg::Path path2, double blend_factor)
{
    // path1 should be centerline
    // path2 should be raceline
    nav_msgs::msg::Path new_path;
    new_path.header.frame_id = "map";
    new_path.header.stamp = this->now();

    // find shortest path
    nav_msgs::msg::Path &short_path = (path1.poses.size() < path2.poses.size()) ? path1 : path2;
    nav_msgs::msg::Path &long_path = (path1.poses.size() < path2.poses.size()) ? path2 : path1;
    for (size_t i = 0; i < short_path.poses.size(); i++)
    {
        geometry_msgs::msg::PoseStamped &current_pose = short_path.poses[i];
        // find closest point on other path
        double min_dist = 1000000;
        geometry_msgs::msg::PoseStamped *closest_pose;
        for (size_t j = 0; j < long_path.poses.size(); j++)
        {
            double dist = sqrt(pow(current_pose.pose.position.x - long_path.poses[j].pose.position.x, 2) + pow(current_pose.pose.position.y - long_path.poses[j].pose.position.y, 2));
            if (dist < min_dist)
            {
                closest_pose = &long_path.poses[j];
            }
        }
        // blend points
        geometry_msgs::msg::PoseStamped pose;
        pose.header = new_path.header;
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";

        pose.pose.position.x = short_path.poses[i].pose.position.x + (closest_pose->pose.position.x - short_path.poses[i].pose.position.x) * blend_factor;
        pose.pose.position.y = short_path.poses[i].pose.position.y + (closest_pose->pose.position.y - short_path.poses[i].pose.position.y) * blend_factor;

        // take largest z
        pose.pose.position.z = std::max(short_path.poses[i].pose.position.z, closest_pose->pose.position.z);
        new_path.poses.push_back(pose);
    }
    return new_path;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointPublisher>("waypoint_publisher_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}