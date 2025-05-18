#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import csv
import os
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import math

csv_file_path = os.path.join(os.getcwd(), 'goal_poses.csv')
print("csv_file_path: ", csv_file_path)

class PoseSubscriber(Node):

    def __init__(self) -> None:
        super().__init__('pose_subscriber')
        
        self.goalPoses = []
        self.goal_pose = Point()
        # Subscription to 2D Pose Estimate
        self.pose_estimate_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.pose_estimate_callback,10)
        
        # Subscription to 2D Goal Pose
        self.goal_pose_subscriber = self.create_subscription(PoseStamped,'/goal_pose',self.goal_pose_callback,10)
        
        # Pub Goal Pose Marker
        self.pub_marker_ = self.create_publisher(MarkerArray, '/waypoint_marker', 10)
        
        self.initialize_csv(csv_file_path)
        self.get_logger().info("Goal Pose Subscriber Initialized.")
        
    def initialize_csv(self, csv_path:str):
            # Check if the CSV file exists; if not, create it and write the header
            file_exists = os.path.isfile(csv_path)
            with open(csv_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                if not file_exists:
                    # Write the header row if the file is new
                    writer.writerow(['x', 'y','orientation_z'])

    def pose_estimate_callback(self, msg:PoseWithCovarianceStamped)->None:
        self.get_logger().info(f'Received 2D Pose Estimate:\n'
                               f'Position: ({msg.pose.pose.position.x}, '
                               f'{msg.pose.pose.position.y}, '
                               f'{msg.pose.pose.position.z})\n'
                               f'Orientation: ({msg.pose.pose.orientation.x}, '
                               f'{msg.pose.pose.orientation.y}, '
                               f'{msg.pose.pose.orientation.z}, '
                               f'{msg.pose.pose.orientation.w})')

    def goal_pose_callback(self, msg:PoseStamped)-> None:
        
        self.get_logger().info(f'Received 2D Goal Pose:\n'
                               f'Position: ({msg.pose.position.x}, '
                               f'{msg.pose.position.y}, '
                               f'{msg.pose.position.z})\n'
                               f'Orientation: ({msg.pose.orientation.x}, '
                               f'{msg.pose.orientation.y}, '
                               f'{msg.pose.orientation.z}, '
                               f'{msg.pose.orientation.w})')
        
        self.display_goal(msg)
        
        self.save_pose_to_csv(msg)
        
    def display_goal(self, msg:PoseStamped)-> None:
        marker_array = MarkerArray()
        self.goalPoses.append([msg.pose.position.x, msg.pose.position.y,msg.pose.orientation.z, msg.pose.orientation.w])
        for i, pose in enumerate(self.goalPoses):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "goal_poses"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = pose[0]
            marker.pose.position.y = pose[1]
            marker.pose.position.z = 0.0 
            
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = pose[2]
            marker.pose.orientation.w = pose[3]
            
            marker.scale.x = 0.3
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.color.a = 1.0  
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
            
            
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "goal_poses_text"
            text_marker.id = i + len(self.goalPoses)  # Ensure a unique ID for the text marker
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Set the position for the text (slightly above or next to the marker)
            text_marker.pose.position.x = pose[0]
            text_marker.pose.position.y = pose[1]
            text_marker.pose.position.z = 0.5  # Slightly above the arrow, you can adjust as needed
            
            text_marker.pose.orientation.x = 0.0
            text_marker.pose.orientation.y = 0.0
            text_marker.pose.orientation.z = pose[2]
            text_marker.pose.orientation.w = pose[3]
            
            # Set the scale and color for the text
            text_marker.scale.z = 0.2  # Text size
            
            text_marker.color.a = 1.0  # Opacity
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            
            yaw_rad = 2 * math.atan2(pose[2], pose[3])
            yaw_degrees = math.degrees(yaw_rad)
            
            text_marker.text = f"x:{pose[0]:.2f} \ny:{pose[1]:.2f} \nz:{yaw_degrees:.2f}"
            marker_array.markers.append(text_marker)
            
        self.pub_marker_.publish(marker_array)


    def save_pose_to_csv(self, msg:PoseStamped)->None:
        with open(csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            # Write the position and orientation data to the CSV
            
            quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
            (_,_,yaw) = euler_from_quaternion(quaternion)
            
            self.goal_pose.x = msg.pose.position.x; self.goal_pose.y = msg.pose.position.y; self.goal_pose.z = yaw
            
            writer.writerow([self.goal_pose.x,
                             self.goal_pose.y,
                             self.goal_pose.z])

def main(args=None):
    rclpy.init(args=args)
    
    node = PoseSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
