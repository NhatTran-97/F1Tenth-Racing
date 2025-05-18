from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.conditions import IfCondition

root_pkg_path = get_package_share_directory('f1tenth_stack')

rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true", description="Flag to activate rviz feature")
lidar_arg = DeclareLaunchArgument("use_lidar", default_value="true", description="Flag to activate lidar feature")
imu_arg = DeclareLaunchArgument("use_imu", default_value="true", description="Flag to activate lidar feature")
camera_arg = DeclareLaunchArgument("use_camera", default_value="true", description="Flag to activate camera feature")
static_tf_arg = DeclareLaunchArgument("use_static", default_value="true", description="Flag to activate static-TF feature")

load_param = [rviz_arg, lidar_arg, camera_arg, static_tf_arg, imu_arg]

# Get path to yaml files
config_dir = os.path.join(root_pkg_path, "config_base")
if not os.path.exists(config_dir):
    raise RuntimeError(f'Configuration directory {config_dir} not found')

joy_teleop_config = os.path.join( config_dir,'joy_teleop.yaml')
vesc_config = os.path.join(config_dir, 'vesc.yaml')
sensors_config = os.path.join( config_dir, 'sensors.yaml')
mux_config = os.path.join(config_dir, 'mux.yaml')
default_config_locks = os.path.join(config_dir, 'ackermann_mux_locks.yaml')
static_tf_config = os.path.join(config_dir, 'static_transform.yaml')


# Declare args
joy_launch = DeclareLaunchArgument('joy_config', default_value=joy_teleop_config, description='Descriptions for joy and joy_teleop configs')
vesc_launch = DeclareLaunchArgument('vesc_config',default_value=vesc_config, description='Descriptions for vesc configs')
sensors_launch = DeclareLaunchArgument('sensors_config',default_value=sensors_config, description='Descriptions for sensor configs')
mux_launch = DeclareLaunchArgument('mux_config', default_value=mux_config, description='Descriptions for ackermann mux configs')
lock_launch = DeclareLaunchArgument('config_locks',default_value=default_config_locks, description='Default locks config file')
static_launch = DeclareLaunchArgument('config_static_tf',default_value=static_tf_config, description='Default static tf config file')

declare_args = joy_launch, vesc_launch, sensors_launch, lock_launch , mux_launch, static_launch

# Node declarations
joy_node = Node(package='joy', executable='joy_node', name='joy',parameters=[LaunchConfiguration('joy_config')])
joy_teleop_node = Node(package='joy_teleop', executable='joy_teleop', name='joy_teleop', parameters=[LaunchConfiguration('joy_config')])

joy = joy_node, joy_teleop_node

ackermann_mux_node = Node(package='ackermann_mux',executable='ackermann_mux',name='ackermann_mux',
                        parameters=[LaunchConfiguration('mux_config'),],    # LaunchConfiguration('config_locks')
                        remappings=[('ackermann_drive_out', 'ackermann_drive')])

vesc_driver_node = Node(package='vesc_driver', executable='vesc_driver_node', name='vesc_driver_node',
                        parameters=[LaunchConfiguration('vesc_config'),],)

ackermann_to_vesc_node = Node(package='vesc_ackermann', executable='ackermann_to_vesc_node',name='ackermann_to_vesc_node',
                        parameters=[LaunchConfiguration('vesc_config')],)

vesc_to_odom_node = Node(package='vesc_ackermann', executable='vesc_to_odom_node', name='vesc_to_odom_node',
                        parameters=[LaunchConfiguration('vesc_config')],)

throttle_interpolator = Node(package='f1tenth_stack', executable='throttle_interpolator.py', name='throttle_interpolator',
                        parameters=[LaunchConfiguration('vesc_config')])

vesc =  ackermann_to_vesc_node, vesc_to_odom_node, throttle_interpolator, vesc_driver_node#, safety_node

rviz_node = Node(package="rviz2",executable="rviz2",name="rviz2",output="screen",
                arguments=["-d", os.path.join(root_pkg_path, "visualize", "rviz_display.rviz")],
                condition = IfCondition(LaunchConfiguration("use_rviz")))


# static_tf_node = Node(package='f1tenth_stack',executable='static_tf_publisher.py',output='screen',
#                 parameters=[LaunchConfiguration('static_tf_config')])

static_tf_node = Node(package='f1tenth_stack',executable='static_tf_publisher.py',
        parameters=[{'static_tf_config': LaunchConfiguration('config_static_tf')}],
        condition = IfCondition(LaunchConfiguration("use_static")))



# ==========================================Include Launch File================================================

blink_led_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("f1tenth_firmware"),"launch", "led.launch.py")))

camera_realsense_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("realsense2_camera"),"launch", "rs_launch.py")), 
    condition = IfCondition(LaunchConfiguration("use_camera")))

urg_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("urg_node2"), "launch", "urg_node2.launch.py")),
    condition=IfCondition(LaunchConfiguration("use_lidar")))

bno055_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("bno055"),"launch","bno055.launch.py")),
    condition = IfCondition(LaunchConfiguration("use_imu")))



def generate_launch_description():
    return LaunchDescription([
        *load_param, *declare_args, 
        *joy,
        ackermann_mux_node,    
        # blink_led_node,    
        *vesc,
        bno055_node,
        camera_realsense_node,
        urg_node,
        static_tf_node,

    ])



