#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PointStamped, TransformStamped
from nav_msgs.msg import Path 
from rclpy.qos import QoSProfile
from ackermann_msgs.msg import AckermannDriveStamped
from rrtStar_Spline import RRTStar
from treeViz import TreeViz
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from planner_utils import Planner_Utils
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry

class LocalPlanner_Server(Node):
    def __init__(self, name):
        super().__init__(name)

        # self.tf_buffer = Buffer()



        self.declare_parameter("update_rate", 10.0)  # Hz
        update_rate = self.get_parameter("update_rate").value


        self.callback_group = ReentrantCallbackGroup()
        self.path_callback_group = MutuallyExclusiveCallbackGroup()

        self.costmap_sub = self.create_subscription(OccupancyGrid, '/local_costmap', self.costmap_callback, 10, callback_group=self.callback_group)
        self.costmap_sub 
        self.get_logger().info("Subscribed to /local_costmap for OccupancyGrid data.")

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose', self.robotPose_callback,10, callback_group=self.callback_group)
        self.pose_sub
        self.get_logger().info("Subscribed to /amcl_pose for robot pose.")

        self.globalPath_sub = self.create_subscription(Path, '/fablab_waypoints', self.globalpath_callback, 10, callback_group = self.path_callback_group)
        self.globalPath_sub; self.get_logger().info("Subscribed to /fablab_waypoints for global path.")


        self.goal_sub = self.create_subscription(PoseStamped,'/goal_pose', self.goal_callback,10); self.goal_sub
        self.goal_sub = self.create_subscription(PointStamped,'/clicked_point', self.clicked_point_callback,10); self.goal_sub
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group = self.path_callback_group); self.odom_sub

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 100)
        self.get_logger().info("Publisher to /drive has been created.")

        self.timer = self.create_timer(1.0 / update_rate, self.control_loop, callback_group=self.callback_group)


        self.global_path = None; self.local_costmap = None
        self.robot_pose = None

        self.start = (0, 0); self.goal = None
        
        self.has_received_path = False  # Cờ đánh dấu đã nhận dữ liệu
        self.xy_array = np.empty((0, 2), dtype=np.float32)  # Mảng 2D cho x, y

 
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.planner_utils = Planner_Utils(self, self.tf_buffer)

        self.rrt_star = RRTStar(node=self)
        self.get_logger().info("Initialized RRTStar")

        self.viz = TreeViz(node=self, resolution=0.1, origin=(-4.0, -2.0), id=1)

        self.map_data = []

        self.path_generated = False
        self.path = []  # Khởi tạo đường đi trống

        self.startPoint_in_baselink(self.start)

                                                                                                                                     

    def control_loop(self):
        # 1️⃣ Kiểm tra dữ liệu đầu vào
        if any([
        not hasattr(self, 'local_costmap') or self.local_costmap is None,
        not hasattr(self, 'robot_pose') or self.robot_pose is None,
        # not hasattr(self, 'goal') or self.goal is None,
        not hasattr(self, 'map_data') or not self.map_data]):
            self.get_logger().warn("Missing required data. Skipping control loop.")
            return
        
        if len(self.xy_array) == 0:
            self.get_logger().warn("No waypoints available. Skipping control loop.")
            return 
        
        # 2️⃣ Hiển thị Start trên RViz
        self.viz.publish_initial_and_goal(initial_point=(0, 0), goal_point = None)

        # 3️⃣ Lấy thông tin từ local_costmap
        width = self.local_costmap.info.width; height = self.local_costmap.info.height
        resolution = self.local_costmap.info.resolution
        origin = (self.local_costmap.info.origin.position.x, self.local_costmap.info.origin.position.y)
        if width <= 0 or height <= 0 or resolution <= 0 or any(v is None for v in origin):
            self.get_logger().error("Invalid map dimensions or origin.")
            return

        # 4️⃣ Tìm Goal Point
        # goal_point = self.planner_utils.find_local_goal(self.robot_pose, self.xy_array, target_distance=2.0)
        # print("type: ", type(goal_point))


        try:
            goal_point = self.planner_utils.find_local_goal(self.robot_pose, self.xy_array, target_distance=2.0)
            # self.get_logger().info(f"Goal point: {goal_point}")
        except Exception as e:
            self.get_logger().error(f"Exception in find_local_goal: {e}")
            return
        
        if goal_point:
            self.viz.publish_initial_and_goal(initial_point=None, goal_point=goal_point)
        else:
            self.get_logger().warn("No valid goal point found. Skipping path planning.")
            return 

        # 5️⃣ Chuyển đổi tọa độ Robot (Start) và Goal thành Grid Cell

        initial_position = self.planner_utils.world_to_grid_cell(self.start[0], self.start[1], width, height, resolution, origin)
        target_position = self.planner_utils.world_to_grid_cell(goal_point[0], goal_point[1], width, height, resolution, origin)

        # target_position = self.planner_utils.world_to_grid_cell(self.goal[0], self.goal[1], width, height, resolution, origin)


        if initial_position is  None or target_position is None:
            self.get_logger().error("Coordinates are out of bounds")
            return
        
        # 6️⃣ Thực thi thuật toán RRT*
        start_time = self.get_clock().now()

        path = self.rrt_star.rrt_start(initial_position, target_position, width, height, self.map_data, resolution, self.viz )

        if not path:
            self.get_logger().warn("No path returned by RRT")
            return 
        
        execution_time = self.get_clock().now() - start_time
        # self.get_logger().info(f"RRT execution time: {execution_time.nanoseconds / 1e9} seconds")
        # self.get_logger().info("RRT: Path sent to navigation system")

        self.path_generated = True
        self.path = path  # Lưu trữ đường đi để sử dụng sau này


    def odom_callback(self, msg: Odometry):
        """
        Lấy tọa độ từ /odom để cải thiện tốc độ cập nhật.
        """
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # self.get_logger().info(f"Robot pose (odom): x={self.robot_pose[0]}, y={self.robot_pose[1]}")

    def clicked_point_callback(self, msg:PointStamped):
        self.viz.publish_initial_and_goal(msg, None)
        transformed_start = self.planner_utils.transform_map_to_base_link(
            msg.header.frame_id,
            msg.point.x,
            msg.point.y)
        
        if not transformed_start:
            self.get_logger().warn("Failed to transform start point to base_link. Skipping processing.")
            return  # Bỏ qua xử lý nếu transform thất bại

        self.start = transformed_start
        self.get_logger().info(f"Start Point (base_link): {self.start}")
        self.path_generated = False  # Reset cờ khi goal thay đổi

        # self.start = msg
        # self.get_logger().info(f"Received initial pose: {msg.point.x}, {msg.point.y}") 

    def startPoint_in_baselink(self, start):

        self.get_logger().info(f"Start Point set to (0.0, 0.0) in base_link frame.")
        self.viz.publish_initial_and_goal(start, None)

    def goal_callback(self, msg:PoseStamped):
        
        self.get_logger().info(f"Received goal pose: {msg.pose.position.x}, {msg.pose.position.y}")
       
        self.viz.publish_initial_and_goal(None, msg)

        transformed_goal = self.planner_utils.transform_map_to_base_link( msg.header.frame_id,
                                                                          msg.pose.position.x,
                                                                          msg.pose.position.y)
        
        if not transformed_goal:
            self.get_logger().warn("Failed to transform goal pose to base_link. Skipping processing.")
            return  # Bỏ qua xử lý nếu transform thất bại
    
        self.goal = transformed_goal
        self.get_logger().info(f"Goal Pose (base_link): {self.goal}")

    def costmap_callback(self, msg:OccupancyGrid):
        self.local_costmap = msg
        if  msg.data:
            self.map_data = [1 if cell >= 50 else 0 for cell in msg.data]
        else:
            self.map_data = []

    def robotPose_callback(self, msg:PoseWithCovarianceStamped):
        pass
        # self.robot_pose = msg

        # if self.robot_pose:
        #     transformed_start = self.planner_utils.transform_map_to_base_link(
        #         self.robot_pose.header.frame_id,
        #         self.robot_pose.pose.pose.position.x,
        #         self.robot_pose.pose.pose.position.y)
        #     self.viz.publish_initial_and_goal(initial_point=transformed_start, goal_point = None)

    def globalpath_callback(self, msg:Path):
        if self.has_received_path:
            return 
        if msg and msg.poses:
            transformed_waypoints = []
            for pose_stamped in msg.poses:
                x = pose_stamped.pose.position.x
                y = pose_stamped.pose.position.y
                z = pose_stamped.pose.position.z
                transformed_point = self.planner_utils.transform_map_to_base_link(frame_id=msg.header.frame_id, x=x, y=y)

                if transformed_point is not None:
                    transformed_waypoints.append(transformed_point)
                else:
                    self.get_logger().warn(f"Failed to transform waypoint ({x}, {y}, {z}) from '{msg.header.frame_id}' to 'base_link'.")
            if transformed_waypoints:
                self.xy_array = np.array(transformed_waypoints, dtype=np.float32)

                self.has_received_path = True
                self.destroy_subscription(self.globalPath_sub)
                self.get_logger().info('Waypoints transformed and unsubscribed from /path topic.')
            else:
                self.get_logger().warn('No valid waypoints after transformation.')
        else:
            self.get_logger().warn('Received an empty Path message.')
 
    def destroy_node(self):
        # Hủy các subscription
        if hasattr(self, 'costmap_sub') and self.costmap_sub is not None:
            self.costmap_sub.destroy()
            self.get_logger().info("Destroyed subscription to /local_costmap")

        if hasattr(self, 'pose_sub') and self.pose_sub is not None:
            self.pose_sub.destroy()
            self.get_logger().info("Destroyed subscription to /amcl_pose")

        if hasattr(self, 'globalPath_sub') and self.globalPath_sub is not None:
            try:
                self.globalPath_sub.destroy()
                self.globalPath_sub = None  # Tránh gọi destroy lần nữa
            except rclpy.handle.InvalidHandle:
                self.get_logger().warn("globalPath_sub đã bị hủy trước đó.")

        # Hủy publisher
        if hasattr(self, 'drive_pub') and self.drive_pub is not None:
            self.drive_pub.destroy()
            self.get_logger().info("Destroyed publisher to /drive")

        # # Hủy timer
        # if hasattr(self, 'timer') and self.timer is not None:
        #     self.timer.cancel()
        #     self.get_logger().info("Destroyed timer for control_loop")

        # Gọi phương thức hủy node từ lớp cha
        super().destroy_node()
        self.get_logger().info("Node has been successfully destroyed.")


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner_Server("localplanner_server")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info("localplanner_server node is running...")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down localplanner_server node...")
    finally:
        if node is not None:
            node.get_logger().info("Destroying node resources...")
            node.destroy_node()
        if executor is not None:
            node.get_logger().info("Shutting down executor...")
            executor.shutdown()

        node.get_logger().info("Shutting down ROS 2...")
        rclpy.shutdown()






import rclpy
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner_Server("localplanner_server")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info("localplanner_server node is running...")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected. Shutting down node...")
    except Exception as e:
        node.get_logger().error(f"Unexpected exception: {e}")
    finally:
        # Hủy node đúng cách
        if node is not None:
            node.get_logger().info("Destroying node resources...")
            node.destroy_node()
        
        # Dọn dẹp executor
        if executor is not None:
            node.get_logger().info("Shutting down executor...")
            executor.shutdown()
        
        # Tắt ROS 2
        node.get_logger().info("Shutting down ROS 2...")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
