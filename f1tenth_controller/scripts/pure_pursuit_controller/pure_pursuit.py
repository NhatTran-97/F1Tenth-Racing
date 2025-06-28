#!/usr/bin/env python3
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
import rclpy.time 
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from nav_msgs.msg import Odometry
import csv
from visualization_msgs.msg import Marker, MarkerArray
from typing import List, Tuple
from std_msgs.msg import Bool 
import math
import numpy as np
import os
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.callback_group = ReentrantCallbackGroup()
        
        # Declare parameters
        self.declare_parameter("waypoint_path", "/home/fablab_01/f1_ws/src/f1tenth_projects/f1tenth_waypoint/racelines/f1tenth_waypoint_path.csv") # change when needed
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("vel_topic","cmd_vel" ) #
        self.declare_parameter("car_rearframe", "base_link")
        self.declare_parameter("global_frame","map")
        self.declare_parameter("drive_topic", "/drive")  
        self.declare_parameter("current_waypoint_topic", "/current_waypoint")
        self.declare_parameter("lookahead_waypoint_topic", "/lookahead_waypoint")  
        self.declare_parameter("timer_duration", 10)
        self.declare_parameter("steering_angle_limit", 1.57) #rad
        self.declare_parameter("look_ahead_dist", 1.0) # meter   
        self.declare_parameter("min_lookahead_dist", 0.5)  # meters
        self.declare_parameter("max_lookahead_dist", 0.5)  # meter        
        self.declare_parameter("velocity_percentage", 0.5) # 50% of the max speed of robot
        
        # waypoints handle.
        self.x_car_world     = 0.0
        self.y_car_world     = 0.0
        self.car_x_vel       = 0.0
        self.car_y_vel       = 0.0
        self.car_angular_vel = 0.0
        csv_file_path = self.get_parameter("waypoint_path").get_parameter_value().string_value; print("csv_file_path: ", csv_file_path)
        self.goal_poses = self.load_waypoint(csv_file_path)
        # self.closet_points = self.get_closest_point(self.goal_poses, 0.0, 0.5, 1.0, 2)
        
        self.get_logger().info(f"Finished Loading total {len(self.goal_poses)} waypoints from {csv_file_path}")
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.publish_waypoints()           
        
        # Get parameters   
        self.waypoint_path = self.get_parameter("waypoint_path").get_parameter_value().string_value
        self.odom_topic    = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.vel_topic     = self.get_parameter("vel_topic").get_parameter_value().string_value
        self.car_frame     = self.get_parameter("car_rearframe").get_parameter_value().string_value
        self.map_frame     = self.get_parameter("global_frame").get_parameter_value().string_value
        self.drive_topic   = self.get_parameter("drive_topic").get_parameter_value().string_value
        self.current_waypoint_topic   = self.get_parameter("current_waypoint_topic").get_parameter_value().string_value
        self.lookahead_waypoint_topic = self.get_parameter("lookahead_waypoint_topic").get_parameter_value().string_value
        
        self.timer_duration = self.get_parameter("timer_duration").get_parameter_value().double_value
        # self.min_lookahead_dist = self.get_parameter("min_lookahead_dist").get_parameter_value().double_value
        # self.max_lookahead_dist = self.get_parameter("max_lookahead_dist").get_parameter_value().double_value
        self.velocity_percent   = self.get_parameter("velocity_percentage").get_parameter_value().double_value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) 
        self.brake = False
        
        self.drive_pub_ = self.create_publisher(AckermannDriveStamped,self.drive_topic, 25)
        self.cur_waypoint_pub_ = self.create_publisher(Marker, self.current_waypoint_topic, 10)
        self.lookahead_waypoint_pub_ = self.create_publisher(Marker, self.lookahead_waypoint_topic, 10)
        self.pose_safety_pub_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 5)
        self.subscription = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose', self.amcl_pose_callback,10,callback_group=self.callback_group)
        self.safety_sub_ = self.create_subscription(Bool, '/emergency_breaking', self.safety_callback, 10)
        
        self.get_logger().warn('Pure pursuit node has been launched')
                   
    # 🛑 Hàm xử lý trạng thái phanh
    def handle_brake_state(self):
        drive_msgObj = AckermannDriveStamped()
        if self.brake:
            drive_msgObj.drive.steering_angle = 0.0
            drive_msgObj.drive.speed = 0.0
            self.get_logger().warn("⚠️ Emergency brake activated! Vehicle is stopping.")
        else:
            drive_msgObj.drive.steering_angle = 0.0
            drive_msgObj.drive.speed = 1.0  # Có thể điều chỉnh tốc độ khi phanh tắt
            self.get_logger().info("✅ Vehicle resuming normal operation.")

        self.drive_pub_.publish(drive_msgObj)
    
    def amcl_pose_publish(self,x,y):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = '/map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        self.get_logger().info('Publishing  Initial Position  \n X= %f \n Y= %f '% (msg.pose.pose.position.x, msg.pose.pose.position.y))
        self.pose_safety_pub_.publish(msg)
    
    
    # def safety_callback(self, safety_msgs:Bool): 
    #     self.brake = safety_msgs.data

    def safety_callback(self, safety_msgs: Bool): 
        if self.brake != safety_msgs.data:
            self.brake = safety_msgs.data
            # if self.brake:
            #     self.get_logger().warn("⚠️ Emergency braking activated! Obstacle detected.")
            # else:
            #     self.get_logger().info("✅ Path is clear. Resuming normal operation.")


    def amcl_pose_callback(self, msg:PoseWithCovarianceStamped):
        # Extract the position and orientation
        # if self.brake:
        #     self.get_logger().warn("🚦 Vehicle stopped due to brake activation. Ignoring pose callback.")
        #     self.handle_brake_state()
        #     return

        self.x_car_world = msg.pose.pose.position.x
        self.y_car_world = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        robot_pose = np.array([self.x_car_world, self.y_car_world])
        # Log the position and orientation of the robot
        robot_yaw = self.quaternion_to_yaw(quaternion)
        lookahead_point_, curr_point_ = self.get_closest_point(self.goal_poses, 2.0)
        lookahead_point_in_carFrame = self.transform_waypoint(lookahead_point_, curr_point_)
        steering_angle = self.calculate_steering_angle(lookahead_point_in_carFrame, robot_pose, robot_yaw, 10.0)
        self.publish_drive_command(steering_angle, 10.0, 2.0, 1.5)
    


    # def vel_callback(self, vel_msg):
    #     self.car_x_vel = vel_msg.linear.x
    #     self.car_y_vel = vel_msg.linear.y
    #     self.car_angular_vel = vel_msg.angular.z
        # self.get_logger().info(f'Car vel: x_vel={car_x_vel}, y)_vel={car_y_vel}, angular_vel={car_angular_vel}')
 
    def publish_drive_command(self, steering_angle, steering_limit, K_p, max_speed):
        if steering_angle is None:
            self.get_logger().warn("Steering angle is None, cannot publish drive command.")
            return  # Exit the function early if the steering angle is invalid


        # Create a message object for AckermannDriveStamped
        drive_msgObj = AckermannDriveStamped()

        # if self.brake:
        #     drive_msgObj.drive.steering_angle = 0.0
        #     drive_msgObj.drive.speed = 0.0
        #     self.get_logger().warn("⚠️ Vehicle is stopped due to braking state.")
        # else:

            # Ensure the steering angle is within dynamic limits
        if steering_angle < 0.0:
            drive_msgObj.drive.steering_angle = max(steering_angle, -self.deg_to_rad(steering_limit))  # Steering limit (negative)
        else:
            drive_msgObj.drive.steering_angle = min(steering_angle, self.deg_to_rad(steering_limit))  # Steering limit (positive)
        curr_velocity =  self.get_velocity(drive_msgObj.drive.steering_angle, 1.0, 0.7, 0.3, max_speed)
        drive_msgObj.drive.speed = curr_velocity
            
        self.drive_pub_.publish(drive_msgObj)

            
        # Log the information (simulating the RCLCPP_INFO functionality)
        # self.get_logger().info(
        #     # f"index: {waypoints.index} ... "
        #     # f"distance: {self.distance_between_2points(waypoints.X[waypoints.index], self.x_car_world, waypoints.Y[waypoints.index], self.y_car_world):.2f}m ... "
        #     f"Speed: {drive_msgObj.drive.speed:.2f}m/s ... "
        #     f"Steering Angle: {self.rad_to_deg(drive_msgObj.drive.steering_angle):.2f}° ... "
        #     f"K_p: {K_p:.2f} ... ")

        # Publish the drive message
        


    def load_waypoint(self, file_path:str) -> List[Tuple[float, float, float]]:
        goal_poses = []
        if os.path.exists(file_path):
            with open(file_path, mode='r') as file:
                reader = csv.reader(file)
                next(reader)  
                for row in reader:
                    if len(row) == 3: 
                        try:
                            x, y, z = map(float, row)
                            goal_poses.append((x, y, z))
                        except ValueError:
                            self.get_logger().warn(f"Ignoring invalid row: {row}")
        else:
            self.get_logger().error(f"CSV file not found: {file_path}")
            
        remove_goal_poses  =  self.remove_overlapped_waypoints(goal_poses, threshold= 0.2)
        smoother_goal_poses = self.smoother_waypoint(remove_goal_poses)
        return smoother_goal_poses

    def publish_waypoints(self)->None:
        marker_array = MarkerArray() 
        for i, pose in enumerate(self.goal_poses):
            # Create a marker for each waypoint
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = pose[0]
            marker.pose.position.y = pose[1]
            marker.pose.position.z = 0.0  # Assuming a 2D plane
            
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0 
            
            marker.scale.x = 0.2  # Size of the sphere
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            marker.color.a = 1.0  # Opacity
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
                
            marker_array.markers.append(marker)
        
        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)
        self.get_logger().info("Published waypoints as markers")

    def smoother_waypoint(self, waypoints_list, weight_data = 0.2 , weight_smooth = 0.8, tolerance = 0.0001) -> None: 
        # Formula for smoother waypoints
        # distance at point i= distance at point(i-1) + distance formula i,  point (i-1)
        # weight_smooth: b = 0.75 ~ 0.98
        # weight_data: a = 1 - b
        # tolerance = 0.001
        smoother_waypoints_list = np.array(waypoints_list)
        waypoints_list = np.array(waypoints_list)
        
        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1,  len(smoother_waypoints_list) - 1):
                aux = np.copy(smoother_waypoints_list[i, :2])  # Store the original values of the first two components
                
                # Apply the smoothing formula only to the first two values (x and y)
                smoother_waypoints_list[i, :2] += weight_data * (waypoints_list[i, :2] - smoother_waypoints_list[i, :2]) + \
                                        weight_smooth * (smoother_waypoints_list[i - 1, :2] + 
                                                        smoother_waypoints_list[i + 1, :2] - 
                                                        2.0 * smoother_waypoints_list[i, :2])
                # Accumulate the total change (sum of absolute differences)
                change += np.sum(np.abs(aux - smoother_waypoints_list[i, :2]))
                    
        return smoother_waypoints_list

    def remove_overlapped_waypoints(self, waypoints_list, threshold = 0.1 ):
        cleaned_waypoints_list = [waypoints_list[0]]  # Start with the first waypoint
    
        for i in range(1, len(waypoints_list)):
            x1, y1,_ = cleaned_waypoints_list[-1]
            x2, y2,_ = waypoints_list[i]        
            if self.distance_between_2points(x1,y1, x2, y2) >= threshold:
                cleaned_waypoints_list.append(waypoints_list[i])
        return cleaned_waypoints_list
 
    def get_closest_point(self,waypoints, lookahead_distance):
        num_waypoints = len(waypoints)
        waypoints_X = np.array([w[0] for w in waypoints])
        waypoints_Y = np.array([w[1] for w in waypoints])
        # Find the closest point for velocity control
        shortest_distance = self.distance_between_2points(waypoints_X[0], waypoints_Y[0],  self.x_car_world, self.y_car_world)
        shortest_i = 0
        
        for i in range(num_waypoints):
            dist = self.distance_between_2points(waypoints_X[i], waypoints_Y[i], self.x_car_world,  self.y_car_world)
            if dist <= shortest_distance:
                shortest_distance = dist
                shortest_i = i
                
        for i in range(shortest_i, shortest_i + num_waypoints):  # Loop through waypoints, wrap around if needed
            idx_start = i % num_waypoints
            idx_end = (i + 1) % num_waypoints

            E = np.array(waypoints[idx_start][:2])  # Start point of the segment
            L = np.array(waypoints[idx_end][:2])    # End point of the segment
            C = np.array([self.x_car_world, self.y_car_world])  # Robot's current position
            r = lookahead_distance  # Radius of the circle (lookahead distance)

            d = L - E  # Direction vector of the segment
            f = E - C  # Vector from the center of the circle to the start of the segment

            # Quadratic equation coefficients
            a = np.dot(d, d)
            b = 2 * np.dot(f, d)
            c = np.dot(f, f) - r * r

            # Discriminant of the quadratic equation
            discriminant = b * b - 4 * a * c

            # If discriminant is negative, no intersection
            if discriminant < 0:
                continue

            # Calculate the two possible intersection points
            discriminant_sqrt = math.sqrt(discriminant)
            t1 = (-b - discriminant_sqrt) / (2 * a)
            t2 = (-b + discriminant_sqrt) / (2 * a)

            # Check if t1 or t2 lies on the path segment (0 <= t <= 1)
            if 0 <= t1 <= 1:
                intersection_point = E + t1 * d
                return  idx_start, shortest_i

            if 0 <= t2 <= 1:
                intersection_point = E + t2 * d
                return  idx_start, shortest_i                     

        return  shortest_i, shortest_i
    
    def visualize_lookahead_point(self, point):
        # Create a Marker message      
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg() # Use ROS 2 Clock for the timestamp
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set the scale of the marker (size of the sphere)
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
               
        # Set the position of the marker
        marker.pose.position.x = self.goal_poses[point][0]
        marker.pose.position.y = self.goal_poses[point][1]
        marker.pose.position.z = 0.0
        
        marker.color.a = 1.0  # Opacity
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        # Set an ID for the marker
        marker.id = 1

        # Publish the marker to RViz or other visualizers
        self.lookahead_waypoint_pub_.publish(marker)
        # self.get_logger().info("Published lookahead waypoint as a marker")
             
    def visualize_current_point(self, point):
        # Create a Marker message
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg() # Use ROS 2 Clock for the timestamp
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set the scale of the marker (size of the sphere)
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        
        marker.color.a = 1.0  # Opacity
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        
        # Set the position of the marker
        marker.pose.position.x = self.goal_poses[point][0]
        marker.pose.position.y = self.goal_poses[point][1]
        marker.pose.position.z = 0.0
        
        # Set an ID for the marker
        marker.id = 1

        # Publish the marker to RViz or other visualizers
        self.cur_waypoint_pub_.publish(marker)
        # self.get_logger().info("Published current waypoint as a marker")
      
    def transform_waypoint(self, lookahead_point, curr_point):
        # Visualize current and lookahead points in world frame
        self.visualize_lookahead_point(lookahead_point)
        self.visualize_current_point(curr_point)      
        try:
            # Get the transform from car_refFrame to global_refFrame 
            transformStamped = self.tf_buffer.lookup_transform(
                "base_link",# self.car_frame,     # car reference frame
                "map",# self.map_frame,     # global reference frame
                rclpy.time.Time(),  # latest available time
                timeout = rclpy.duration.Duration(seconds=1.0)  # timeout duration
            )
            # transform_timestamp = transformStamped.header.stamp.sec + transformStamped.header.stamp.nanosec * 1e-9
            # self.get_logger().info(f"Received transform at time: {transform_timestamp:.9f}")
            self.get_logger().info(f'Translation: {transformStamped.transform.translation}')
            self.get_logger().info(f'Rotation: {transformStamped.transform.rotation}')
                
        except TransformException as ex:
            self.get_logger().info(f'Transformation return ERROR: {ex}')
            return
        # transform_timestamp = transformStamped.header.stamp.sec + transformStamped.header.stamp.nanosec * 1e-9
        # self.get_logger().info(f"Received transform at time: {transform_timestamp:.9f}")
        # Extract translation and rotation from the transform

        point_translation = np.array([
            transformStamped.transform.translation.x,
            transformStamped.transform.translation.y,
            transformStamped.transform.translation.z
        ])

        # Convert quaternion to rotation matrix
        point_rotation_in_quad = transformStamped.transform.rotation
        point_rotation_in_maxtrix = self.quat_to_rot(point_rotation_in_quad.w, point_rotation_in_quad.x,
                                                     point_rotation_in_quad.y, point_rotation_in_quad.z)

        # Transform the lookahead point from world to car coordinates
        lookahead_point_in_carframe = (point_rotation_in_maxtrix @ self.goal_poses[lookahead_point]) + point_translation
        # lookahead_point_in_carframe = [x_of_waypoint_in_carFrame, y_of_waypoint_in_carFrame, z_of_waypoint_in_carFrame]
        
        return lookahead_point_in_carframe
    
        
    def get_velocity(self, steering_angles: float, high_speed_factor: float, medium_speed_factor: float, 
                     slow_speed_factor: float, car_speed: float) -> float:
        # When the steering angle small -> the car is allowed to run with high speed
        if(abs(steering_angles) >= self.deg_to_rad(0.0) and abs(steering_angles) < self.deg_to_rad(10.0)):
            velocity = high_speed_factor * car_speed
            
        # When the steering angle medium -> the car is allowed to run with medium speed
        elif(abs(steering_angles) >= self.deg_to_rad(10.0) and abs(steering_angles) < self.deg_to_rad(20.0)):
            velocity = medium_speed_factor * car_speed
            
        # When the steering angle high -> the car is allowed to run slow speed
        else:   
            velocity = slow_speed_factor * car_speed
        
        return velocity
    
    def calculate_curvature(self, lookahead, robot_position, robot_angle):
        # Calculate the angle to the lookahead point relative to the car's orientation
        x_lookahead = self.goal_poses[lookahead][0]
        y_lookahead = self.goal_poses[lookahead][1]
        
        dx = x_lookahead - robot_position[0]
        dy = y_lookahead - robot_position[1]

        # Calculate the angle to the lookahead point
        angle_to_lookahead = math.atan2(dy, dx)

        # Calculate the heading error (difference between robot's heading and angle to lookahead point)
        heading_error = angle_to_lookahead - robot_angle

        # Ensure the heading error is between -pi and pi
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        # Calculate the distance between the car and the lookahead point
        lookahead_distance = math.sqrt(dx**2 + dy**2)

        # Calculate curvature: curvature = 2 * sin(heading_error) / lookahead_distance
        # Ackermann steering curvature is the inverse of the radius of the turn
        if lookahead_distance != 0:
            curvature = 2 * math.sin(heading_error) / lookahead_distance
        else:
            curvature = 0

        # Return the curvature value (used to calculate the steering angle)
        return curvature  
    
    def calculate_steering_angle(self, lookahead, robot_position, robot_angle, K_p):
        if lookahead is None:
            self.get_logger().warn("Lookahead point is None, cannot calculate steering angle.")
            return None  # 
        # Lookahead point in global frame
        x_lookahead = lookahead[0]
        y_lookahead = lookahead[1]
        
        # Calculate dx, dy between robot and lookahead point
        dx = x_lookahead - 0
        dy = y_lookahead - 0

        # Calculate the angle to the lookahead point
        angle_to_lookahead = math.atan2(dy, dx)

        # Calculate the heading error (difference between robot's heading and angle to lookahead point)
        heading_error = angle_to_lookahead - robot_angle

        # Normalize heading error to be between -pi and pi
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        # Calculate the distance between the robot and the lookahead point
        lookahead_distance = math.sqrt(dx**2 + dy**2) # L 

        # Apply the proportional controller logic
        # r is the distance between the robot and the lookahead point (same as lookahead_distance)
        r = lookahead_distance
        y = dy  # y is the lateral distance (error) to the lookahead point

        # Apply the proportional control to calculate the final steering angle
        steering_angle = K_p * 2 * y / (r ** 2)

        return steering_angle


           
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

# def main(args=None):
#     rclpy.init(args=args)
#     print("PurePursuit Initialized")
#     pure_pursuit_node = PurePursuit()
#     executor = MultiThreadedExecutor()
#     executor.add_node(pure_pursuit_node)
    
#     try:
#         # rclpy.spin(pure_pursuit_node)
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#         # pure_pursuit_node.drive_msgObj.drive.speed = 0.0
#     finally:
#         if pure_pursuit_node is not None:
#             pure_pursuit_node.get_logger().info("Destroying node resources...")
#         if executor is not None:
#             pure_pursuit_node.get_logger().info("Shutting down executor...")
#             executor.shutdown()
#     # pure_pursuit_node.destroy_node()
#     rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    executor = MultiThreadedExecutor()
    executor.add_node(pure_pursuit_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pure_pursuit_node.get_logger().info("Shutting down node...")
    finally:
        pure_pursuit_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
