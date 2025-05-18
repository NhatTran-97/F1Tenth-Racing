#!/usr/bin/env python3
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped



class Initial_Pose(Node):
    def __init__(self):
        super().__init__('initial_pose_node')
        
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.sub_pose = self.create_subscription(PointStamped, '/clicked_point', self.callback, 1)
        self.sub_pose 
    
    def callback(self, msg):
        self.get_logger().info("Received Data: X: %f \n Y: %f \n Z: %f" %(msg.point.x, msg.point.y, msg.point.z))
        self.publish(msg.point.x, msg.point.y)
    
    def publish(self, x, y):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "/map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        self.get_logger().info('Publishing Initial Position \n X= %f \n Y=%f' %(msg.pose.pose.position.x, msg.pose.pose.position.y))
        self.pub_pose.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    pub_initPose = Initial_Pose()
    rclpy.spin(pub_initPose)
    pub_initPose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

        