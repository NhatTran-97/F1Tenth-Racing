#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from vision_msgs.msg import  ObjectCoordinates
import math
class Object_MarkerArray(Node):
    def __init__(self) -> None:
        super().__init__('object_marker_node')
        self.pub_marker_ = self.create_publisher(MarkerArray, '/object_marker', 10)
        self.sub_object_coor = self.create_subscription(ObjectCoordinates, '/object_coordinates', self.object_callback,10)

        

    def object_callback(self, msg:ObjectCoordinates)-> None:
        marker_array = MarkerArray()
        
        for i,obj_coord in enumerate (msg.object_coordinates):
            class_name = obj_coord.class_name
            marker = Marker()
            marker.header.frame_id = "camera_link"  # Set this to your appropriate frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = str(i)
            marker.id = i
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.pose.position.x = obj_coord.object_point.z
            marker.pose.position.y = -(obj_coord.object_point.x)
            marker.pose.position.z = -(obj_coord.object_point.y)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0

            marker.scale.x = 0.001
            marker.scale.y = 0.001
            marker.scale.z = 0.001
            marker.color.a = 1.0  # Don't forget to set the alpha!

            # if class_name == "person":
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.z = math.sin(math.pi / 4)
            marker.pose.orientation.w = math.cos(math.pi / 4)
            marker.mesh_resource = "package://f1tenth_perception/meshes/MCR.stl"
            # else:
            #     marker.color.r = 0.0
            #     marker.color.g = 1.0
            #     marker.color.b = 1.0
            #     marker.pose.orientation.z = 0.0
            #     marker.pose.orientation.w = 1.0
            #     marker.mesh_resource = "package://f1tenth_perception/meshes/mbot.stl"
                
            marker.mesh_use_embedded_materials = False
                
            marker_array.markers.append(marker)

        self.pub_marker_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    object_marker = Object_MarkerArray()
    
    try:
        rclpy.spin(object_marker)
    except KeyboardInterrupt:
        pass

    object_marker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
