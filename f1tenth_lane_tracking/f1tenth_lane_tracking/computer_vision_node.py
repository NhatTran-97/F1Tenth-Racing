#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from drive import Car


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.img_sub = self.create_subscription(Image, '/camera/color/image_raw', self.camera_callback, 10)
        
        # Initialize VideoWriter
        self.get_logger().info('Subscribing Image Feed')
        self.bridge = CvBridge()
        self.car = Car()
    
    def camera_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.car.drive_car(frame=current_frame)
        angle, speed, img = 0.0, 0.0, current_frame
    
    def destroy_node(self):
        super().destroy_node()

def main():
    rclpy.init()
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
