from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.conditions import IfCondition

def generate_launch_description():
    root_pkg_path = get_package_share_directory('f1tenth_stack')

    rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true", description="Flag to activate rviz feature")
    waypoint_launch = DeclareLaunchArgument('waypoint_config', default_value=os.path.join(get_package_share_directory('f1tenth_waypoint'), 'config', 'waypoint_params.yaml'), description='Default waypoint config file')


    safety_node = Node(package='f1tenth_automatic_emergency',executable='safety_node',name='safety_node',)
    
    rviz_node = Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
                    arguments=["-d", os.path.join(root_pkg_path, "visualize", "rviz_display.rviz")],
                    condition=IfCondition(LaunchConfiguration("use_rviz")))

    waypoint_publisher_cpp_node = Node(package='f1tenth_waypoint',
                                      executable='waypoint_publisher_node',
                                      name='waypoint_publisher_node',
                                      parameters=[LaunchConfiguration('waypoint_config')])

    load_base_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("f1tenth_stack"), "launch", "base_setup.launch.py")))

    localize_robot_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("f1tenth_stack"), "launch", "provide_map.launch.py")))

    return LaunchDescription([
        rviz_arg, waypoint_launch,
        load_base_setup,
        safety_node
        # rviz_node,
        # waypoint_publisher_cpp_node,
        
        # localize_robot_map
    ])
