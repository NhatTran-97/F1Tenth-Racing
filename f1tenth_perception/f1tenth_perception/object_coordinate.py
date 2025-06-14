#!/usr/bin/env python3
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from vision_msgs.msg import YoloResults, ObjectCoordinate, ObjectCoordinates


import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class Object_Coordinate(Node):
    def __init__(self, depth_image_topic):
        super().__init__('object_coordinate')
        self.bridge = CvBridge()
    
        self.dist_mm = np.zeros((480, 640), dtype=np.uint16)

        
        self.fx = 604.63879; self.fy = 604.62677
        
        
        self.cx = 320 # 318.513 320
        self.cy = 240  # 251.910 240
        
        self.object_coordinates_msg = ObjectCoordinates()

        
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "camera_color_optical_frame"

        self.sub_box_ = self.create_subscription(YoloResults, "/yolov7_Inference/bounding_box", self.object_callback, 10)
        self.sub_depth_img_ = self.create_subscription(Image, depth_image_topic, self.imageDepthCallback, 10)
        
        self.pub_objCoor = self.create_publisher(ObjectCoordinates, 'object_coordinates', 10)
        self.marker_publisher_ = self.create_publisher(Marker, 'object_markers', 10)
    
    def object_callback(self, msg:YoloResults)->None:
        if self.dist_mm is not None:

            object_coordinates = []
            for i, object in enumerate(msg.yolo_inference): 
                x_center =  (object.top + object.bottom) // 2; y_center =  (object.left + object.right) // 2
                
                # if object.class_name =="person":
                if self.is_within_bounds(x_center, y_center):
                    dist_obj_m = self.dist_mm[y_center, x_center] / 1000
            
                    # self.get_logger().info(f"Object {i} center depth: {dist_obj_m:.2f} cm at ({x_center}, {y_center})")
                    X = dist_obj_m * ((x_center - self.cx)/ self.fx)
                    Y = (dist_obj_m * ((y_center - self.cy)/ self.fy))
                    Z = dist_obj_m
                    # self.get_logger().info(f"Object {object.class_name} X: {X:.2f} Y: {Y:.2f} Z: {Z:.2f}")
                    object_coordinates.append((object.class_name,X, Y, Z))
                    
                    object_coordinate_msg = ObjectCoordinate()
                    object_coordinate_msg.object_point = Point(x=X, y=Y, z=Z)

                    object_coordinate_msg.class_name = object.class_name
                    
                    
                    self.object_coordinates_msg.object_coordinates.append(object_coordinate_msg)
                    
            self.pub_objCoor.publish(self.object_coordinates_msg)    
            self.object_coordinates_msg.object_coordinates.clear()
                    
                # else:
                #     dist_obj_m = 0
                
            for i, (class_name,X, Y, Z) in enumerate(object_coordinates):
                
                self.transform_stamped_.transform.translation.x = X
                self.transform_stamped_.transform.translation.y = Y
                self.transform_stamped_.transform.translation.z = Z
                
                self.transform_stamped_.transform.rotation.x = 0.0
                self.transform_stamped_.transform.rotation.y = 0.0
                self.transform_stamped_.transform.rotation.z = 0.0
                self.transform_stamped_.transform.rotation.w = 1.0
                
                self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
                self.transform_stamped_.child_frame_id = f"{class_name}_frame"
                self.br_.sendTransform(self.transform_stamped_)

            object_coordinates.clear()
        else:
            self.get_logger().warn("Depth map is not available.")
            
            
    def is_within_bounds(self, x:int, y:int)->bool:
        """Check if (x, y) coordinates are within the bounds of the depth image."""
        return self.dist_mm is not None and 0 <= x < self.dist_mm.shape[1] and 0 <= y < self.dist_mm.shape[0]
    
    def is_valid_distance(dist_obj_m, min_dist, max_dist):
        return min_dist <= dist_obj_m <= max_dist
            
    
    def imageDepthCallback(self, data:Image)->None:
        try:
            self.dist_mm = self.bridge.imgmsg_to_cv2(data, "16UC1") # shape => (480, 640)
          #  self.get_logger().info(f"self.dist_mm  {self.dist_mm/1000.0} ")
            # self.dist_mm = cv2.flip(depth_img, 2)
           
            if data.encoding != "16UC1":
                self.get_logger().warn(f"Unsupported encoding type: {data.encoding}")
                return
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return




def main(args=None):
    rclpy.init(args=args)
    depth_image_topic = '/camera/camera/depth/image_rect_raw'    
    aligned_depth_to_color_topic = "/camera/camera/aligned_depth_to_color/image_raw"
    depth_info_topic = '/camera/camera/depth/camera_info'
    
    node = Object_Coordinate(aligned_depth_to_color_topic) 
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

