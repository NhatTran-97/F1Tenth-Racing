#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.img_sub = self.create_subscription(Image, '/camera/color/image_raw', self.camera_callback, 10)
        # Initialize VideoWriter
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')  # Define codec
        self.out = cv2.VideoWriter('lane.mp4', cv2.VideoWriter_fourcc(*'MJPG'), 30, (640, 480)) # Output file, codec, FPS, frame size
        self.get_logger().info('Subscribing Image Feed')
        self.bridge = CvBridge()
    
    def camera_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.out.write(current_frame)  # Write frame to video

        cv2.imshow('Camera_feed', current_frame)
        cv2.waitKey(1)
    
    def destroy_node(self):
        self.out.release()  # Release VideoWriter
        super().destroy_node()

def main():
    rclpy.init()
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
