#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header  # Import Header for Marker messages
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.sub_goal_pose_ = self.create_subscription(PoseStamped, "/goal_pose", self.goalPose_Callback, QoSProfile(depth=10))
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.waypointActive = False
    
    def goalPose_Callback(self, msg):
        marker4 = Marker()
        marker4.header = Header()  # Initialize Header
        marker4.header.frame_id = "map"
        marker4.header.stamp = self.get_clock().now().to_msg()
        marker4.ns = "basic_shapes"
        marker4.id = 4
        marker4.type = Marker.TEXT_VIEW_FACING
        marker4.action = Marker.ADD
        marker4.pose.position.x = msg.pose.position.x
        marker4.pose.position.y = msg.pose.position.y
        marker4.pose.position.z = 0.5
        marker4.pose.orientation.x = 0.0
        marker4.pose.orientation.y = 0.0
        marker4.pose.orientation.z = 0.0
        marker4.pose.orientation.w = 1.0
        marker4.scale.x = 0.5
        marker4.scale.y = 0.5
        marker4.scale.z = 0.5
        marker4.color.a = 1.0
        marker4.color.r = 1.0
        marker4.color.g = 0.0
        marker4.color.b = 0.0
        marker4.text = "GOAL"
        self.publisher_.publish(marker4)

        
        
        

    def timer_callback(self):
        # Create Marker 1
        marker1 = Marker()
        marker1.header = Header()  # Initialize Header
        marker1.header.frame_id = "map"
        marker1.header.stamp = self.get_clock().now().to_msg()
        marker1.ns = "basic_shapes"
        marker1.id = 1
        marker1.type = Marker.TEXT_VIEW_FACING
        marker1.action = Marker.ADD
        marker1.pose.position.x = -2.0
        marker1.pose.position.y = 0.0
        marker1.pose.position.z = 1.5
        marker1.pose.orientation.x = 0.0
        marker1.pose.orientation.y = 0.0
        marker1.pose.orientation.z = 0.0
        marker1.pose.orientation.w = 1.0
        marker1.scale.x = 1.0
        marker1.scale.y = 1.0
        marker1.scale.z = 1.0
        marker1.color.a = 1.0
        marker1.color.r = 0.0
        marker1.color.g = 1.0
        marker1.color.b = 0.0
        marker1.text = "EIU_FABLAB"

        # Create Marker 2
        marker2 = Marker()
        marker2.header = Header()  # Initialize Header
        marker2.header.frame_id = "map"
        marker2.header.stamp = self.get_clock().now().to_msg()
        marker2.ns = "basic_shapes"
        marker2.id = 2
        marker2.type = Marker.TEXT_VIEW_FACING
        marker2.action = Marker.ADD
        marker2.pose.position.x = -1.0
        marker2.pose.position.y = 0.0
        marker2.pose.position.z = 1.0
        marker2.pose.orientation.x = 0.0
        marker2.pose.orientation.y = 0.0
        marker2.pose.orientation.z = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.scale.x = 0.6
        marker2.scale.y = 0.6
        marker2.scale.z = 0.6
        marker2.color.a = 1.0
        marker2.color.r = 0.0
        marker2.color.g = 1.0
        marker2.color.b = 1.0
        marker2.text = "Movement of a Racing Car Along Straight Line (INIT => GOAL)"
        
        
               # Create Marker 2
        marker3 = Marker()
        marker3.header = Header()  # Initialize Header
        marker3.header.frame_id = "map"
        marker3.header.stamp = self.get_clock().now().to_msg()
        marker3.ns = "basic_shapes"
        marker3.id = 3
        marker3.type = Marker.TEXT_VIEW_FACING
        marker3.action = Marker.ADD
        marker3.pose.position.x = 0.0
        marker3.pose.position.y = 0.0
        marker3.pose.position.z = 0.5
        marker3.pose.orientation.x = 0.0
        marker3.pose.orientation.y = 0.0
        marker3.pose.orientation.z = 0.0
        marker3.pose.orientation.w = 1.0
        marker3.scale.x = 0.5
        marker3.scale.y = 0.5
        marker3.scale.z = 0.5
        marker3.color.a = 1.0
        marker3.color.r = 0.0
        marker3.color.g = 1.0
        marker3.color.b = 0.0
        marker3.text = "INIT"

        # Publish markers
       # self.publisher_.publish(marker1)  # Corrected publisher name
        #self.publisher_.publish(marker2)  # Corrected publisher name
        self.publisher_.publish(marker3)

def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()
    rclpy.spin(marker_publisher)

    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 
