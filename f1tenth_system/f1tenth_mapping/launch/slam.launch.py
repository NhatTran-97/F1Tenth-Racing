import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    slam_config_arg = DeclareLaunchArgument("slam_config", default_value=os.path.join(get_package_share_directory("f1tenth_mapping"), 
                                                                          "config", 
                                                                          "f1tenth_online_async.yaml"))

    slam_config = LaunchConfiguration("slam_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    lifecycle_nodes = ["map_saver_server"]
    
    """
    slam toolbox we have multiple options online async or online sync and offline scanning => That means that if the robot wants continue the scan from LIDAR
    it can althought it contains a map, then you want to stop a mapping process for some time
    """
    
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",     # async_slam_toolbox_node
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config, 
                    {"use_sim_time":use_sim_time}],)


    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        parameters=[
                    {"save_map_timeout":5.0},
                    {"use_sim_time":use_sim_time},
                    {"free_thresh_default":0.196},
                    {"occupied_thresh_default":0.65}])

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names":lifecycle_nodes},
            {"use_sim_time":use_sim_time},
            {"autostart":True}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        nav2_map_saver,
        slam_toolbox,
        nav2_lifecycle_manager,

    ])
