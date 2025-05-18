from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare a launch argument to select which node to run (default is 'cpp')
    
    declare_arg = DeclareLaunchArgument("use_cpp",
                                        default_value="True",
                                        description="Select whether to use the C++ node (True) or the Python node (False).")
    use_cpp = LaunchConfiguration("use_cpp")

    config = os.path.join(
        get_package_share_directory('f1tenth_waypoint'), 'config', 'waypoint_params.yaml')

    # Define Python node
    waypoint_generator_python_node = Node(
        package='f1tenth_waypoint',
        executable='waypoint_generator_node.py',
        name='waypoint_generator_node',
        parameters=[config],
        condition = UnlessCondition(use_cpp)
        
    )

    # Define C++ node
    waypoint_generator_cpp_node = Node(
        package='f1tenth_waypoint',
        executable='waypoint_generator_node',
        name='waypoint_generator_node',
        parameters=[config],
        
        condition=IfCondition(use_cpp)
    )

    # Define RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory(
            'f1tenth_waypoint'), 'rviz', 'waypoint_visualization.rviz')])
    
    localize_robot_map_rosPKG = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("f1tenth_stack"), "launch", "provide_map.launch.py")
        )
    )
    
    return LaunchDescription([
        declare_arg,  # Declare the launch argument
        rviz_node,  # Add RViz node
        waypoint_generator_cpp_node,  # Add C++ node if condition is met
        waypoint_generator_python_node,  # Add Python node if condition is met
        localize_robot_map_rosPKG
    ])


