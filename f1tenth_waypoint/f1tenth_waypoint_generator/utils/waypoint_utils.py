import csv
import os
from typing import List, Tuple, Union

from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class CSVFile:
    def __init__(self, rclpy_node: Node):
        self.rclpy_node = rclpy_node

        # rclpy_node.get_logger().info('Initializing parameters')

        if not self.rclpy_node.has_parameter('path_to_csvFile'):
            rclpy_node.declare_parameter(name='path_to_csvFile', value="/f1_ws/src/f1tenth_system/f1tenth_waypoint/racelines")
        
        if not self.rclpy_node.has_parameter('csvFile_name'):
            rclpy_node.declare_parameter(name='csvFile_name', value="f1tenth_waypoint.csv")

        try:
            csv_path = rclpy_node.get_parameter('path_to_csvFile').get_parameter_value().string_value
            csvFile_name = rclpy_node.get_parameter('csvFile_name').get_parameter_value().string_value

        except Exception as e:
            self.rclpy_node.get_logger().warn('Could not get parameters ...setting variables to default')
            self.rclpy_node.get_logger().warn(f'Error: "{e}"')

        # Construct the full path to the CSV file
        self.path_to_csvFile = os.path.join(os.getcwd() , csv_path, csvFile_name )

        # Initialize the CSV file by writing a header
        self.initialize_csv(self.path_to_csvFile )
    
    def initialize_csv(self, csv_path:str):

        """ Initialize the CSV file and write the header. Overwrites the file. """

        with open(csv_path, mode='w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file, delimiter=',')
            self.rclpy_node.get_logger().info(f"Creating or overwriting CSV file at {csv_path} and writing header.")
            csv_writer.writerow(['x', 'y', 'heading'])
    
    def write_points_to_csv(self, waypoint_msgs:Union[Point, Odometry]) -> None:

        """ Write points from PointStamped or Odometry message to the CSV file. """

        with open(self.path_to_csvFile, mode='a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file, delimiter=',')
    
            if isinstance(waypoint_msgs, Point):
                x = waypoint_msgs.x 
                y = waypoint_msgs.y  
                yaw = waypoint_msgs.z 
                self.rclpy_node.get_logger().info(f'Writing Clicked Point: x={x}, y={y}, yaw={yaw}')
                csv_writer.writerow([x, y, yaw])
                
            elif isinstance(waypoint_msgs, Odometry):
                # Extract x, y from Odometry
                x = waypoint_msgs.pose.pose.position.x
                y = waypoint_msgs.pose.pose.position.y
                quaternion = (waypoint_msgs.pose.pose.orientation.x, waypoint_msgs.pose.pose.orientation.y, 
                              waypoint_msgs.pose.pose.orientation.z, waypoint_msgs.pose.pose.orientation.w)

                _,_,yaw = euler_from_quaternion(quaternion)
                self.rclpy_node.get_logger().info(f'Writing Odometry: x={x}, y={y}, yaw={yaw}')
                csv_writer.writerow([x, y, yaw])

            else:
                self.rclpy_node.get_logger().warn(f"Unknown message type received: {type(waypoint_msgs)}")

class Waypoint_Visualization:
    def __init__(self, rclpy_node: Node) -> None:
        self.rclpy_node = rclpy_node
        self.waypoint_marker = Marker()
        self.initialize_marker()
    
    def initialize_marker(self)->None:
        self.waypoint_marker.header.frame_id = "map"
        self.waypoint_marker.type = Marker.LINE_STRIP
        self.waypoint_marker.action = Marker.ADD
        self.waypoint_marker.scale.x = 0.05 # width of the line 
        self.waypoint_marker.scale.y = 0.1 
        self.waypoint_marker.scale.z = 0.1 

        self.waypoint_marker.color.a = 1.0 
        self.waypoint_marker.color.r = 0.0
        self.waypoint_marker.color.g = 1.0
        self.waypoint_marker.color.b = 0.0

    def update_waypoint_marker_callback(self, waypoint_msgs:Union[Point, Odometry])->None:
        """ Updates the marker with new waypoints."""
        if isinstance(waypoint_msgs, Point):
            self.waypoint_marker.points.append(waypoint_msgs)

        elif isinstance(waypoint_msgs, Odometry):
            point = Point()
            point.x = waypoint_msgs.pose.pose.position.x 
            point.y = waypoint_msgs.pose.pose.position.y
            point.z = 0.0
            self.waypoint_marker.points.append(point)

        else:
            # self.rclpy_node.get_logger().warn(f"Unknown message type received: {type(waypoint_msgs)}")
            raise ValueError(f"Unknown message type received: {type(waypoint_msgs)}")
    
    def get_waypoint_marker(self)->Marker:
        """Returns the current marker object for publishing."""
        return self.waypoint_marker




class UtilsWaypoint():
    def __init__(self, rclpy_node: Node):
        """ Initialize the utility class with the Node and a CSVFile instance """

        self.rclpy_node = rclpy_node

        self.csv_file = CSVFile(self.rclpy_node)

        self.waypoint_marker = Waypoint_Visualization(self.rclpy_node)

    def process_waypoint(self, waypoint_msgs: Union[Point, Odometry]) -> None:
        """ Process the waypoint and write to CSV """
        self.csv_file.write_points_to_csv(waypoint_msgs)
    
    def update_waypoint_marker(self, waypoint_msgs: Union[Point, Odometry]) -> None:
        self.waypoint_marker.update_waypoint_marker_callback(waypoint_msgs)
    
    def get_waypoint_marker(self) -> Marker:
        """Return the current waypoint marker"""
        return self.waypoint_marker.get_waypoint_marker()


