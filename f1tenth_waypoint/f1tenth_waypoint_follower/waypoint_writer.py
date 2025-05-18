#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, PointStamped
import csv
import os
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
import math

# csv_file_path = os.path.join(os.getcwd(), 'goal_poses.csv')
# print("csv_file_path: ", csv_file_path)

class Waypoint_Write(Node):

    def __init__(self) -> None:
        super().__init__('waypoint_node')
        
        self.goalPoses = []
        self.goal_pose = Point()
        
        # Subscription to 2D Goal Pose
        self.goal_pose_subscriber = self.create_subscription(PointStamped,'/clicked_point',self.clicked_point_callback,10)
        
        # Pub Goal Pose Marker
        self.pub_marker_ = self.create_publisher(Marker, '/waypoints_marker', 10)

        
        self.marker = Marker()
        self.initialize_marker()
        self.csv_file = open('waypoints.csv', mode='w')
        self.delimiter = ", "
        self.previous_point = None
        self.precision = 10 # Number of extra points to add
        self.get_logger().info("Goal Pose Subscriber Initialized.")
    

    def initialize_marker(self):
        self.marker.header.frame_id = "map"
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.05 # width of the line
        self.marker.scale.y = 0.1 # size of the spheres
        self.marker.scale.z = 0.1
        self.marker.color.a = 1.0 # alpha
        self.marker.color.r = 1.0 # alpha
        self.marker.color.g = 0. # Green
        self.marker.color.b = 0. # Blue

    def interpolate_points(self, start, end, num_points):
        """Interpolate extra points between start and end"""
        points = []
        for i in range(1, num_points + 1):
            fraction = i/ (num_points + 1)
            X = start.x + fraction * (end.x - start.x)
            Y = start.y + fraction * (end.y - start.y)
            Z = start.z + fraction * (end.z - start.z)
            points.append(Point(x=X, y=Y, z=Z))
        return points
    
    def write_point_to_file(self, point):
        x, y = point.x, point.y
        w_tr_right_m = 1.1
        w_tr_left_m = 1.1
        line = f"{x}{self.delimiter}{y}{self.delimiter}{w_tr_right_m}{self.delimiter}{w_tr_left_m}\n"
        self.csv_file.write(line)
        self.csv_file.flush()
        self.get_logger().info(f'Writing to Filte: {line.strip()}')
    
    def __del__(self):
        self.csv_file.close()



    def clicked_point_callback(self, msg:PointStamped)-> None:
        new_point = msg.point
        if self.previous_point is not None:
            # Generate interpolate points
            interpolated_points = self.interpolate_points(self.previous_point, new_point, self.precision)
            for point in interpolated_points:
                self.write_point_to_file(point)
                self.marker.points.append(point)
        
        self.write_point_to_file(new_point)
        self.marker.points.append(new_point)
        self.pub_marker_.publish(self.marker)
        self.previous_point = new_point

def main(args=None):
    rclpy.init(args=args)
    node = Waypoint_Write()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
