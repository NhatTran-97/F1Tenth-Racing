from launch import LaunchDescription
from launch_ros.actions import Node



rgbd_camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.32', '0', '0.05', '0', '0', '0', 'base_link', 'camera_link'])

lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0.285", "0.0", "0.0986",  # Translation (X, Y, Z)
                   "0.0", "0.0", "0.0", # Rotation (Roll, Pitch, Yaw)
                        "base_link", "laser_link","--disable-stdin"],
        output='screen',
                                 
        parameters=[{"use_sim_time": False}], # Tắt use_sim_time để đảm bảo thời gian thực
      )  
            

imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0.205", '0', '-0.018', '0', '0', '0', 'base_link', 'imu_link'])

def generate_launch_description():
    return LaunchDescription([
        rgbd_camera_static_tf,
        lidar_static_tf,
        imu_static_tf
    ])


