from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
import os
from launch.actions import DeclareLaunchArgument
import yaml
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    map_path = os.path.join(get_package_share_directory('f1tenth_mapping'), 'maps', 'map.yaml')
    amcl_config_path = os.path.join(get_package_share_directory('f1tenth_stack'), 'config_params', 'amcl_config.yaml')
    
    root_pkg_path = get_package_share_directory('f1tenth_stack')
    
    localize_config = os.path.join(get_package_share_directory('particle_filter'), 'config', 'localize.yaml')
    
    # localize_config_dict = yaml.safe_load(open(localize_config, 'r'))

    localize_la = DeclareLaunchArgument(
        'localize_config',
        default_value=localize_config,
        description='Localization configs')
    
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': False}, 
                    {'yaml_filename': map_path}])

    nav2_acml_node = Node(
        package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[{'use_sim_time': False},
                            {'amcl_config': amcl_config_path}])
    
    pf_node = Node(
        package='particle_filter',
        executable='particle_filter',
        name='particle_filter',
        parameters=[LaunchConfiguration('localize_config')])

    nav2_lifecycle_manager_node = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': False},
                            {'autostart': True},
                            {'node_names': ['map_server']}])

    rviz_node = Node( package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(root_pkg_path, "visualize", "display_map.rviz")],)
    
    local_costmap_node = Node(package='f1tenth_navigation',
                            executable='local_costmap',
                            name='local_costmap',
                            output='screen',)
    
    return LaunchDescription([
        localize_la,
        map_server_node,
        # nav2_acml_node,
        pf_node,
        
        nav2_lifecycle_manager_node,
        # local_costmap_node
        # rviz_node
        
    ])