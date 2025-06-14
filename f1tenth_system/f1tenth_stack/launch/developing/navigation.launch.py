from launch import LaunchDescription
from launch_ros.actions import Node 
import os 
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    map_file = os.path.join(get_package_share_directory('f1tenth_mapping'), 'maps', 'map.yaml')
    config_dir = os.path.join(get_package_share_directory('f1tenth_stack'), 'config_params')
    params_file = os.path.join(config_dir, 'tb3_nav_params.yaml')

    rviz_config = os.path.join(config_dir, 'tb3_nav.rviz')
    
    x_pose = LaunchConfiguration('x_pose', default = '-5.2')
    y_pose = LaunchConfiguration('y_pose', default = '-6.7')

    f1tenth_nav = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch', '/bringup_launch.py']),
                                 launch_arguments={
                                     'map':map_file,
                                     'param_file': params_file}.items(),)
        
    rviz = Node(
        package = 'rviz2',
        output='screen',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', rviz_config]
    )
    
    ld = LaunchDescription()
    ld.add_action(rviz)
    ld.add_action(f1tenth_nav)
    return ld
