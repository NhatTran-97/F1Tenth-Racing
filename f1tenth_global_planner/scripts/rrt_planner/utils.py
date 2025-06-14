# #!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
import csv
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Point
from typing import List, Tuple
import os
import copy
import math
import numpy as np


class Utils():
    '''
        Credit to 
        - Gongsta:  https://github.com/CL2-UWaterloo/f1tenth_ws/blob/main/src/stanley_avoidance/stanley_avoidance/stanley_avoidance.py#L845
        - Modern robotics: https://github.com/NxRLab/ModernRobotics/blob/master/packages/Python/modern_robotics/core.py#L146
        Code had been modified.
    '''
    def __init__(self):
        pass
        
    def visualize_maker(self, frame_id, time_stamp, position, publisher, color="r", id= 1):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = time_stamp # Use ROS 2 Clock for the timestamp
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set the scale of the marker (size of the sphere)
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
            
        # Set the position of the marker
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.0
        
        marker.color.a = 1.0  # Opacity
        if color == "r":
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0  
        elif color == "g":
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0   
        elif color == "b":
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0  
        # Set an ID for the marker
        marker.id = id

        # Publish the marker to RViz or other visualizers
        publisher.publish(marker) 
    
    def visualize_maker_arrray(self, frame_id, time_stamp, positions, publisher, color="b"):
        # Create a Marker Array message      
        marker = MarkerArray()
        for i, position in enumerate(positions):
            if position is None: 
                continue
            
            marker.header.frame_id = frame_id
            marker.header.stamp = time_stamp # Use ROS 2 Clock for the timestamp
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Set the scale of the marker (size of the sphere)
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
                
            # Set the position of the marker
            marker.pose.position.x = positions[i].x
            marker.pose.position.y = positions[i].y
            marker.pose.position.z = 0.0
            
            marker.color.a = 1.0  # Opacity
            
            if color == "r":
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0  
            elif color == "g":
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0   
            elif color == "b":
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0 
            # Set an ID for the marker
            
            marker.id = i

            # Publish the marker to RViz or other visualizers
            publisher.publish(marker)
            
    def visualize_lines(self, frame_id, time_stamp, path, publisher, color="g"):
        points = []
        for i in range(len(path) - 1):
            a = path[i]
            b = path[i + 1]
            point = Point()
            point.x = a[0]
            point.y = a[1]
            points.append(copy.deepcopy(point))
            point.x = b[0]
            point.y = b[1]
            points.append(copy.deepcopy(point))

        line_list = Marker()
        line_list.header.frame_id = frame_id
        line_list.header.stamp = time_stamp
        line_list.id = 0
        line_list.type = line_list.LINE_LIST
        line_list.action = line_list.ADD
        line_list.scale.x = 0.1
        line_list.color.a = 1.0
        
        if color == "r":
            line_list.color.r = 1.0
            line_list.color.g = 0.0
            line_list.color.b = 0.0  
        elif color == "g":
            line_list.color.r = 0.0
            line_list.color.g = 1.0
            line_list.color.b = 0.0   
        elif color == "b":
            line_list.color.r = 0.0
            line_list.color.g = 0.0
            line_list.color.b = 1.0 
            
        line_list.points = points
        publisher.publish(line_list)
    
    def deg_to_rad(self, degrees: float) -> float:
        return degrees * math.pi / 180.0

    def rad_to_deg(self, radians : float) -> float:
        return radians * 180.0  / math.pi
    
    def distance_between_2points(self,x1: float, y1: float, x2: float, y2: float) -> float:
        return math.sqrt(math.pow((x2 - x1),2)  + math.pow((y2 - y1), 2))

    def quat_to_rot(self, q0: float, q1: float, q2: float, q3: float) -> np.array:
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                               [r10, r11, r12],
                               [r20, r21, r22]])   
                                
        return rot_matrix

    def quaternion_to_yaw(self, quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        # Yaw (rotation around Z-axis)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

    def trans_to_rp(self, T):
        T = np.array(T)
        return T[0: 3, 0: 3], T[0: 3, 3]
  
    def is_rot(self,R):        
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        
        return n < 1e-6
  
    def rot_to_Euler(self, R):
        assert self.is_rot(R), "Input is not a valid rotation matrix"
        
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0   
        return np.array([x, y, z])
   
    def rp_to_trans(self, R, p):
        """Converts a rotation matrix and a position vector into homogeneous
        transformation matrix

        :param R: A 3x3 rotation matrix
        :param p: A 3-vector
        :return: A homogeneous transformation matrix corresponding to the inputs

        Example Input:
            R = np.array([[1, 0,  0],
                        [0, 0, -1],
                        [0, 1,  0]])
                        
            p = np.array([1, 2, 5])
        Output:
            np.array([[1, 0,  0, 1],
                    [0, 0, -1, 2],
                    [0, 1,  0, 5],
                    [0, 0,  0, 1]])
        """
        return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]

