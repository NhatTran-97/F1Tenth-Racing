        
        
geometry_msgs::msg::PoseStamped pose;
pose.header = new_path.header;
pose.header.stamp = this->now();
pose.header.frame_id = "map";
pose.pose.position.x = x;
pose.pose.position.y = y;
pose.pose.position.z = v;


RCLCPP_INFO(this->get_logger(), "Pose information:");
RCLCPP_INFO(this->get_logger(), "  Position: x = %f, y = %f, z = %f",
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
RCLCPP_INFO(this->get_logger(), "  Orientation: x = %f, y = %f, z = %f, w = %f",
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

RCLCPP_INFO(this->get_logger(), "New Path Poses:");

RCLCPP_INFO(this->get_logger(), "Number of poses in new_path: %zu", new_path.poses.size());

for (size_t i = 0; i < new_path.poses.size(); ++i)
{
        const auto &p = new_path.poses[i];
        RCLCPP_INFO(this->get_logger(), "Pose %d:", static_cast<int>(i + 1));
        RCLCPP_INFO(this->get_logger(), "  Position: x = %f, y = %f, z = %f",
                        p.pose.position.x, p.pose.position.y, p.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "  Orientation: x = %f, y = %f, z = %f, w = %f",
                        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
 }



/*Truy cập lấy phần tử trong mảng*/

new_path.poses.push_back(pose);

const auto &added_pose = new_path.poses.back(); // Lấy phần tử vừa được thêm
RCLCPP_INFO(this->get_logger(), "New Pose added to new_path:");
RCLCPP_INFO(this->get_logger(), "  Position: x = %f, y = %f, z = %f",
                added_pose.pose.position.x, added_pose.pose.position.y, added_pose.pose.position.z);
RCLCPP_INFO(this->get_logger(), "  Orientation: x = %f, y = %f, z = %f, w = %f",
                added_pose.pose.orientation.x, added_pose.pose.orientation.y, added_pose.pose.orientation.z, added_pose.pose.orientation.w);


    localize_robot_map_rosPKG = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("f1tenth_stack"), "launch", "provide_map.launch.py")
        )
    )



export AMENT_PREFIX_PATH=$(echo $AMENT_PREFIX_PATH | sed 's#/home/fablab_01/f1_ws/install/f1tenth_waypoint_generator##')
export CMAKE_PREFIX_PATH=$(echo $CMAKE_PREFIX_PATH | sed 's#/home/fablab_01/f1_ws/install/f1tenth_waypoint_generator##')