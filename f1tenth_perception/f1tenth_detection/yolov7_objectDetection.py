#!/usr/bin/env python3
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
library_path = os.path.join(script_dir, 'yolov7/build/libmyplugins.so')
engine_path = os.path.join(script_dir, 'yolov7/build/custom_yolov7.engine')


import numpy as np
import cv2
import rclpy
from yoloDet import YoloTRT

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import YoloResults, InferenceResult

bridge = CvBridge()

class Camera_Subscriber(Node):
    def __init__(self)->None:
        super().__init__('yolov7_inference')
        self.color_img = np.zeros((480,640,3), dtype=np.uint8)
        self.yolo_inference = YoloResults()
        self.inference_result = InferenceResult()
        
        self.model = YoloTRT(library=library_path, engine=engine_path, conf=0.6, yolo_ver="v7")

        self.sub_frame_ = self.create_subscription(Image, '/camera/camera/color/image_raw', self.camera_callback,10)
        self.yolov8_pub = self.create_publisher(YoloResults, "/yolov7_Inference/bounding_box", 1)
        self.img_pub = self.create_publisher(Image, "/yolov7_Inference/image", 1)

    def camera_callback(self, data:Image)-> None:
        try:
            self.color_img = bridge.imgmsg_to_cv2(data, "bgr8")
            self.yolo_inference.header.frame_id = "inference"
            self.yolo_inference.header.stamp = self.get_clock().now().to_msg()
            detections,detected_frame = self.model.Inference(self.color_img, track = True)
            
            for obj in detections:
                self.inference_result.class_name = obj['class']
                self.inference_result.id = obj['id']
                self.inference_result.top = int(obj['x'])
                self.inference_result.left = int(obj['y'])
                self.inference_result.bottom = int(obj['width']) + int(obj['x'])
                self.inference_result.right = int(obj['height']) + int(obj['y'])
                
                self.yolo_inference.yolo_inference.append(self.inference_result)
                center_x = (self.inference_result.top + self.inference_result.bottom) // 2
                center_y = (self.inference_result.left + self.inference_result.right) // 2
                
                cv2.circle(detected_frame, (center_x, center_y), radius=5, color=(0, 255, 0), thickness=-1)
                
            img_msg = bridge.cv2_to_imgmsg(detected_frame)
            self.img_pub.publish(img_msg)
            self.yolov8_pub.publish(self.yolo_inference)
            
            self.yolo_inference.yolo_inference.clear()
            
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return
        
def main(args=None):
    rclpy.init(args=args)

    
    node = Camera_Subscriber() 
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            #     # self.get_logger().info(f"Info: class={obj['class']}, confidence={obj['conf']}, box={obj['box']}")
