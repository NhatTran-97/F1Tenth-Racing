from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

def generate_launch_description():
    # Lifecycle Manager
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_object",
        output="screen",
        parameters=[
            {"node_names": ['yolov7_inference', 'object_coordinate']},
            {"use_sim_time": False},
            {"autostart": True}
        ]
    )

    # Lifecycle Node 1: yolov7_node
    yolov7_node = LifecycleNode(
        package='f1tenth_perception',
        executable='yolov7_objectDetection.py',
        name='yolov7_inference',
        output="screen"
    )

    # Lifecycle Node 2: object_coordinate_node
    object_coordinate_node = LifecycleNode(
        package='f1tenth_perception',
        executable='object_coordinate.py',
        name='object_coordinate',
        output="screen"
    )

    return LaunchDescription([
        lifecycle_manager,
        yolov7_node,
        object_coordinate_node
    ])
