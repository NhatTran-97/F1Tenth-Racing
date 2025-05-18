
import math
from rclpy.node import Node, Publisher
from nav_msgs.msg import Odometry
from  f1tenth_waypoint_generator.utils.waypoint_utils import UtilsWaypoint



class OdomWaypoint(UtilsWaypoint):
    def __init__(self, rclpy_node: Node) -> None:
        super().__init__(rclpy_node)

        self.rclpy_node = rclpy_node


        if not self.rclpy_node.has_parameter('odom_min_distance'):
            rclpy_node.declare_parameter(name='odom_min_distance', value=0.05)

        try:
            self.min_distance = rclpy_node.get_parameter('odom_min_distance').get_parameter_value().double_value

        except Exception as e:
            self.rclpy_node.get_logger().warn('Could not get parameters ...setting variables to default')
            self.rclpy_node.get_logger().warn(f'Error: "{e}"')
            self.min_distance = 0.05
        
        self.x_old = 0; 
        self.y_old = 0
    

    def odom_callback(self, odom_msg:Odometry, waypoint_marker_pub_:Publisher)->None:
        
        # Calculate the difference using proper distance formula
        diff = math.sqrt(
                            pow((odom_msg.pose.pose.position.x - self.x_old), 2) + 
                            pow((odom_msg.pose.pose.position.y - self.y_old), 2))
        
        # check if the car has moved more than the minimum distance 
        if diff > self.min_distance:
            # Up date car position
            x_car = odom_msg.pose.pose.position.x
            y_car = odom_msg.pose.pose.position.y 
            self.rclpy_node.get_logger().info(f'Car moved: x = {x_car}, y = {y_car}, distance = {diff:.2f} meters')
            self.process_waypoint(odom_msg)
            self.update_waypoint_marker(odom_msg)
            self.publish_waypoint_marker(waypoint_marker_pub_)

            # Update old position values
            self.x_old = x_car
            self.y_old = y_car
    
    def publish_waypoint_marker(self,  waypoint_marker_pub_:Publisher):
        """Publish the updated marker to the topic"""
        waypoint_marker = self.get_waypoint_marker()
        waypoint_marker_pub_.publish(waypoint_marker)






