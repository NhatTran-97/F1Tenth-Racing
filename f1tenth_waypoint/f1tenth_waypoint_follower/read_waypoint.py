#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import csv
import os
from visualization_msgs.msg import Marker, MarkerArray
from typing import List, Tuple
#csv_file_path = os.path.join(os.getcwd(), 'goal_poses.csv')

csv_file_path = '/home/fablab_01/f1_ws/goal_poses.csv'; print("csv_file_path: ", csv_file_path)


class WaypointFollower(Node):

    def __init__(self)->None:
        super().__init__('pose_subscriber')
        self.goal_poses = self.read_goal_poses_from_csv(csv_file_path)
        self.get_logger().info(f"Loaded {len(self.goal_poses)} waypoints from {csv_file_path}")
        print("print: ", self.goal_poses)
        
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        self.publish_waypoints()
        
        
    def read_goal_poses_from_csv(self, file_path:str) -> List[Tuple[float, float, float]]:
        goal_poses = []
        if os.path.exists(file_path):
            with open(file_path, mode='r') as file:
                reader = csv.reader(file)
                next(reader)  
                for row in reader:
                    if len(row) == 3: 
                        x, y, z = map(float, row)
                        goal_poses.append((x, y, z))
        else:
            self.get_logger().error(f"CSV file not found: {file_path}")
        return goal_poses
    
    def publish_waypoints(self)->None:
        marker_array = MarkerArray()
        
        for i, pose in enumerate(self.goal_poses):
            # Create a marker for each waypoint
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            marker.pose.position.x = pose[0]
            marker.pose.position.y = pose[1]
            marker.pose.position.z = 0.0  # Assuming a 2D plane
            
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 0.0
            
            marker.scale.x = 0.3  # Size of the sphere
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            
            marker.color.a = 1.0  # Opacity
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
        
        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)
        self.get_logger().info("Published waypoints as markers")
        


def main(args=None):
    rclpy.init(args=args)
    
    node = WaypointFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
