#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import LaserScan
import math
from tf2_ros import Buffer, TransformListener, LookupException, TransformException
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray
from typing import List, Dict, Optional

ORIGIN_FRAME = "odom"; DEST_FRAME = "laser_link"

class Coordinates:
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y 
        self.z = z 
        self.roll = roll 
        self.pitch = pitch 
        self.yaw = yaw

class LidarPerception(Node):
    def __init__(self)->None:
        super().__init__('lidar_perception')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.subscription  # prevent unused variable warning
        
        self.pub_marker_ = self.create_publisher(MarkerArray, '/lidar_object_marker', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.angle_idx = self.get_index_from_angle(-1, 1)
        print("self.angle_idx: ", self.angle_idx)
        
        
    def get_index_from_angle(self, start_angle:int, end_angle:int, lidar_param:Dict[str, float] = {'angle_min': -135, 'angle_increment':0.25})->Dict[int, int]:

        start_index = int((start_angle - lidar_param["angle_min"]) / lidar_param["angle_increment"])
        
        end_index = int((end_angle - lidar_param["angle_min"]) /lidar_param["angle_increment"])
        
        angle_dict = {"start_angle_index": start_index, "end_angle_index": end_index}
        
        return angle_dict
    
    
    def get_pose_from_tf(self, origin_frame:str = "odom", dest_frame:str = "laser_link")->Optional[Pose]:
        try:
            trans = self.tf_buffer.lookup_transform(origin_frame, dest_frame, rclpy.time.Time())
            
        except TransformException as ex:
            self.get_logger().error(f"Could not tranform {origin_frame} to {dest_frame}:  {ex}")
            return None
        
        translation_pose = trans.transform.translation; rotation_pose = trans.transform.rotation
        pose = Pose()
        pose.position.x = translation_pose.x
        pose.position.y = translation_pose.y
        pose.position.z = translation_pose.z
        
        pose.orientation.x = rotation_pose.x
        pose.orientation.y = rotation_pose.y
        pose.orientation.z = rotation_pose.z
        pose.orientation.w = rotation_pose.w
        
        return pose
        

    def scan_callback(self, scan:LaserScan)->None:
        pose_dest = self.get_pose_from_tf(origin_frame=ORIGIN_FRAME, dest_frame=DEST_FRAME)
        # self.get_logger().info("POSE DEST=" + str(pose_dest))
        if pose_dest is not None:
            explicit_qua = [pose_dest.orientation.x, pose_dest.orientation.y,
                            pose_dest.orientation.z, pose_dest.orientation.w]
            pose_now_euler = euler_from_quaternion(explicit_qua)


            object_coordinates = []
            for i in range(self.angle_idx['start_angle_index'], self.angle_idx['end_angle_index'] + 1, 1):
                if math.isinf(scan.ranges[i]):
                    continue
                
                angle = scan.angle_min + (i * scan.angle_increment) #+ pose_now_euler[2]
                # degree = math.degrees(angle); degree_ang = round(degree, 2)
         
                px = scan.ranges[i] * math.cos(angle)
                py = scan.ranges[i] * math.sin(angle)
             
                px += pose_dest.position.x; py += pose_dest.position.y; 
                pz = pose_dest.position.z
                coordinate_dict = {'x': px, 'y': py, 'z': pz}
                object_coordinates.append(coordinate_dict)
                
            self.objectCoordinate_marker(object_coordinates, frame_id=ORIGIN_FRAME)
        else:
            self.get_logger().error("Failed to get pose from transform.")
   

    
    def objectCoordinate_marker(self, object_poses:List[Dict[str, float]], frame_id:str=ORIGIN_FRAME)->None:
        marker_array = MarkerArray()
        
        for i,obj_coord in enumerate (object_poses):
            marker = Marker()
            marker.header.frame_id = str(frame_id)  # Set this to your appropriate frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = str(i)
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = obj_coord['x']
            marker.pose.position.y = obj_coord['y']
            marker.pose.position.z = obj_coord['z']  #0.2025
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0

            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            
            marker.color.a = 1.0  # Don't forget to set the alpha!
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.z = math.sin(math.pi / 4)
            marker.pose.orientation.w = math.cos(math.pi / 4)
                
            marker_array.markers.append(marker)
            
        self.pub_marker_.publish(marker_array)
                

def main(args=None):
    rclpy.init(args=args)
    lidar_perception = LidarPerception()
    
    try:
        rclpy.spin(lidar_perception)
    except KeyboardInterrupt:
        pass

    lidar_perception.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
