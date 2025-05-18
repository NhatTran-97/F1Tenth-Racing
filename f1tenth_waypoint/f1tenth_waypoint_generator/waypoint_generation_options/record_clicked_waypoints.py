from rclpy.node import Node, Publisher
from geometry_msgs.msg import PointStamped, Point
from typing import List
from  f1tenth_waypoint_generator.utils.waypoint_utils import UtilsWaypoint

class ClickedWaypoint(UtilsWaypoint):
    def __init__(self, rclpy_node: Node) -> None:
        super().__init__(rclpy_node)

        self.rclpy_node = rclpy_node
        
        self.previous_point = None
        self.precision = 10  # Precision could be made configurable if needed

    def clicked_point_callback(self, click_point_msg: PointStamped, waypoint_marker_pub_:Publisher ) -> None:
        """
        Callback to handle a new clicked point message.
        """
        new_point = click_point_msg.point
        if self.previous_point is not None:

            # Interpolating points between the previous and new point
            interpolated_points = self.interpolate_points(self.previous_point, new_point, self.precision)
          
            for point in interpolated_points:
                self.process_waypoint(point)
                self.update_waypoint_marker(point)
                # self.rclpy_node.get_logger().info('saved point')

        self.process_waypoint(new_point)
        self.update_waypoint_marker(new_point)
        self.publish_waypoint_marker(waypoint_marker_pub_)
        self.previous_point = new_point
    
    def publish_waypoint_marker(self, waypoint_marker_pub_: Publisher):
        """ Publish the updated marker to the topic """
        waypoint_marker = self.get_waypoint_marker()
        waypoint_marker_pub_.publish(waypoint_marker)
        

    def interpolate_points(self, start: Point, end: Point, num_points: int) -> List[Point]:
        """
        Interpolate extra points between start and end.
        """
        points = []
        for i in range(1, num_points + 1):
            fraction = i / (num_points + 1)
            x = start.x + fraction * (end.x - start.x)
            y = start.y + fraction * (end.y - start.y)
            z = start.z + fraction * (end.z - start.z)
            points.append(Point(x=x, y=y, z=z))
        
        return points

        
