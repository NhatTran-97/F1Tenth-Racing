from launch import LaunchDescription
from launch_ros.actions import Node


blink_led_node = Node(
        package='f1tenth_firmware',
        executable='blink_led',
        name='blink_led',)

def generate_launch_description():    
    return LaunchDescription([

        blink_led_node
    ])
