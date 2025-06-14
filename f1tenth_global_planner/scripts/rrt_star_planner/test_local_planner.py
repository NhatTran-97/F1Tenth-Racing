#!/usr/bin/env python3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import OccupancyGrid
import numpy as np 
from geometry_msgs.msg import TransformStamped, Pose
from utils import Utils_Localcostmap, WaypointUtils, Utils, VisualizeMarkers
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class MappingWithKnownPoses(Node):
    def __init__(self, name):

        
        super().__init__(name)
        self.declare_parameter("waypoints_path", "/home/fablab_01/f1_ws/src/f1tenth_projects/f1tenth_navigation/scripts/racelines/e7_floor5.csv")
        self.declare_parameter("waypoints_path_2nd", "/home/fablab_01/f1_ws/src/f1tenth_projects/f1tenth_navigation/scripts/racelines/e7_floor5.csv")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("occupancy_grid_topic", "/occupancy_grid")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("grid_width_meters", 6.0)
        self.declare_parameter("cells_per_meter", 10)
        self.declare_parameter("interpolation_distance", 0.05)
        self.declare_parameter("min_lookahead", 1.0)
        self.declare_parameter("max_lookahead", 3.0)
        self.declare_parameter("min_lookahead_speed", 3.0)
        self.declare_parameter("max_lookahead_speed", 6.0)
        self.declare_parameter("lane_number", 0)



        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.occupancy_grid_topic = str(self.get_parameter("occupancy_grid_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.L = float(self.get_parameter("max_lookahead").value)
        self.CELLS_PER_METER = int(self.get_parameter("cells_per_meter").value)
        self.grid_width_meters = float(self.get_parameter("grid_width_meters").value)
        self.interpolation_distance = float(self.get_parameter("interpolation_distance").value)
        self.waypoints_world_path = str(self.get_parameter("waypoints_path").value)
        self.waypoints_world_path_2nd = str(self.get_parameter("waypoints_path_2nd").value)
        self.lane_number = int(self.get_parameter("lane_number").value)  # Dynamically change lanes

        min_lookahead = float(self.get_parameter("min_lookahead").value)
        max_lookahead = float(self.get_parameter("max_lookahead").value)
        min_lookahead_speed = float(self.get_parameter("min_lookahead_speed").value)
        max_lookahead_speed = float(self.get_parameter("max_lookahead_speed").value)

        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 1)
        self.occupancy_grid_pub = self.create_publisher(OccupancyGrid, self.occupancy_grid_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 1)
        


        
        # class variables
        self.current_pose = None; self.goal_pos = None
        self.current_pose_wheelbase_front = None
        self.closest_wheelbase_rear_point = None

        self.grid_height = int(self.L * self.CELLS_PER_METER)
        self.grid_width = int(self.grid_width_meters * self.CELLS_PER_METER)


        self.local_costmap = Utils_Localcostmap(self, self.grid_height,self.grid_width, self.CELLS_PER_METER)

        self.waypoint_utils = WaypointUtils(node = self,L = self.L, interpolation_distance=self.interpolation_distance, filepath=self.waypoints_world_path, min_lookahead=min_lookahead,
                                            max_lookahead=max_lookahead, min_lookahead_speed=min_lookahead_speed, max_lookahead_speed=max_lookahead_speed, filepath_2nd=self.waypoints_world_path_2nd,)
        
        self.get_logger().info(f"Loaded {len(self.waypoint_utils.waypoints_world)} waypoints")


        self.utils = Utils()
        self.marker_viz = VisualizeMarkers(self)


    def scan_callback(self, scan_msg):
        """
        LaserScan callback, update occupancy grid and perform local planning

        Args:
            scan_msg (LaserScan): incoming message from subscribed topic
        """
        # make sure we obtain initial pose and goal point
        # if (self.current_pose is None) or (self.goal_pos is None):
        #     return
        self.local_costmap.populate_occupancy_grid(scan_msg.ranges, scan_msg.angle_increment,scan_msg.angle_min)
        self.local_costmap.convolve_occupancy_grid()
        self.local_costmap.publish_occupancy_grid(scan_msg.header.frame_id, scan_msg.header.stamp, self.occupancy_grid_pub)

        path_local = []

        # current_pos = np.array(self.local_costmap.local_to_grid(0, 0))
        current_pos = np.array([0.0, 0.0]) 
        self.marker_viz.visualize_positions(current_pos, None )

        # goal_pos = np.array(self.local_costmap.local_to_grid(self.goal_pos[0], self.goal_pos[1]))
    

    
    def odom_callback(self, pose_msg: Odometry):
        """
        The pose callback when subscribed to particle filter's inferred pose
        """
        self.current_pose = pose_msg.pose.pose


        # self.get_logger().info(str(self.base_link_to_laser_tf))
        
        current_pose_quaternion = np.array(
            [
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w,
            ]
        )

        self.current_pose_wheelbase_front = Pose()

        current_pose_xyz = R.from_quat(current_pose_quaternion).apply((0.285, 0, 0.099)) + (self.current_pose.position.x, self.current_pose.position.y,0)

        self.current_pose_wheelbase_front.position.x = current_pose_xyz[0]
        self.current_pose_wheelbase_front.position.y = current_pose_xyz[1]
        self.current_pose_wheelbase_front.position.z = current_pose_xyz[2]
        self.current_pose_wheelbase_front.orientation = self.current_pose.orientation
        print(self.current_pose_wheelbase_front)



def main(args=None):
    rclpy.init(args=args)
    node = MappingWithKnownPoses("motion_planning")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down motion_planning node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
