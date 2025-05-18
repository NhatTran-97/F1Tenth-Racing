#!/usr/bin/env python3
import sys
from ament_index_python import get_package_prefix 
site_packages_path = f'{get_package_prefix("f1tenth_waypoint")}/lib/python3.8/site-packages'
if site_packages_path not in sys.path: sys.path.append(site_packages_path)


import rclpy
from rclpy.node import Node
from enum import Enum
from visualization_msgs.msg import Marker
from typing import  Union

from nav_msgs.msg import Odometry
from f1tenth_waypoint_generator.waypoint_generation_options.record_clicked_waypoints import ClickedWaypoint
from f1tenth_waypoint_generator.waypoint_generation_options.record_odom_waypoints import OdomWaypoint
from geometry_msgs.msg import PointStamped, Point



class WaypointType(Enum):
    CLICKEDPOINT_WAYPOINT = "clicked_waypoint"
    ODOM_WAYPOINT = "odom_waypoint"

class WaypointGenerator(Node):
    def __init__(self, waypoint_type:WaypointType, odom_waypoint:OdomWaypoint, clicked_waypoint:ClickedWaypoint)->None:
        super().__init__('waypoint_generator_node')

        self.odom_waypoint = odom_waypoint
        self.clicked_waypoint = clicked_waypoint

        message_type, waypoint_topic = self.setup_waypoint_option(waypoint_type)

        if message_type is not None and waypoint_topic is not None:
            self.clicked_point_sub_ = self.create_subscription(message_type, waypoint_topic, self.waypoint_callback,10)
        
        self.waypoint_marker_pub_ = self.create_publisher(Marker, '/f1tenth_waypoint_marker', 10)
    
    def setup_waypoint_option(self, waypoint_type:WaypointType):
        
        if not self.has_parameter('waypoint_options'):
            self.declare_parameter(name='waypoint_options', value='clicked_waypoint')  # clicked_waypoint, odom_waypoint
        
        if not self.has_parameter('clicked_point_topic'):
            self.declare_parameter(name='clicked_point_topic', value='/clicked_point')
        
        if not self.has_parameter('odom_topic'):
            self.declare_parameter(name='odom_topic', value='/odom')
            
        try:
            odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
            self.get_logger().info(f'\t odom_topic:\t "{odom_topic}')

            clicked_point_topic = self.get_parameter('clicked_point_topic').get_parameter_value().string_value
            self.get_logger().info(f'\t clicked_point_topic:\t "{clicked_point_topic}')

            waypoint_option = self.get_parameter('waypoint_options').get_parameter_value().string_value
            self.get_logger().info(f'\t waypoint_options:\t "{waypoint_option}')

        except Exception as e:
            self.get_logger().warn('Could not get parameters ...setting variables to default')
            self.get_logger().warn(f'Error: "{e}"')
            return None, None

        if waypoint_option == waypoint_type.CLICKEDPOINT_WAYPOINT.value:
            self.get_logger().info("Using CLICKEDPOINT_WAYPOINT option")
            return PointStamped, clicked_point_topic

        elif waypoint_option == waypoint_type.ODOM_WAYPOINT.value:
            self.get_logger().info("Using ODOM_WAYPOINT option")
            return Odometry, odom_topic
        else:
            self.get_logger().info("Unknown waypoint option, using default")
            return None,None
    
    def waypoint_callback(self, waypoint_msgs: Union[PointStamped, Odometry])->None:
        """
        Callback function to handle incoming waypoint message. Can handle either PointStamped or Odometry message
        """
        # Check if the message is of type PointStamped
        if isinstance(waypoint_msgs, PointStamped):
            self.get_logger().info(f"Received PointStamped message: x = {waypoint_msgs.point.x}, y = {waypoint_msgs.point.y}, z = {waypoint_msgs.point.z}")
            self.clicked_waypoint.clicked_point_callback(waypoint_msgs, self.waypoint_marker_pub_ )
            

        # Check if the message is of type Odometry
        elif isinstance(waypoint_msgs, Odometry):
            # self.get_logger().info(f"Received Odometry message: x = {waypoint_msgs.pose.pose.position.x}, y = {waypoint_msgs.pose.pose.position.y}, z = {waypoint_msgs.pose.pose.position.z}")
            self.odom_waypoint.odom_callback(waypoint_msgs, self.waypoint_marker_pub_)

        else:
            self.get_logger().warn(f"Unknown message type received: {type(waypoint_msgs)}")


def main(args=None):
    rclpy.init(args=args)

    share_node = rclpy.create_node('waypoint_shared_node')
    clicked_waypoint = ClickedWaypoint(share_node); odom_waypoint = OdomWaypoint(share_node)
    
    waypoint_node = WaypointGenerator(WaypointType, odom_waypoint, clicked_waypoint)
        
    try:
        rclpy.spin(waypoint_node)
    except KeyboardInterrupt:
        pass
    
    waypoint_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
