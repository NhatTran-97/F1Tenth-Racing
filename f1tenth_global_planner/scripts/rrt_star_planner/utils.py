
import numpy as np
from scipy import signal
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from typing import Tuple, List, Optional
from rclpy.node import Node

class Utils_Localcostmap():
    def __init__(self,node, grid_height, grid_width, CELLS_PER_METER):
        self.node = node

        self.grid_height = grid_height  # 30 cell
        self.grid_width = grid_width # 30 cell 
        self.ANGLE_OFFSET = np.radians(40) # Góc bù cho lidar, thường điều chỉnh góc nhìn  45
        self.IS_OCCUPIED = 100
        self.IS_FREE = 0
        self.CELLS_PER_METER = CELLS_PER_METER
        self.CELL_Y_OFFSET = (self.grid_width // 2) - 1   # Offset trên trục Y để đặt lưới chiếm dụng tại trung tâm robot 

        self.occupancy_grid = np.full(shape=(self.grid_height, self.grid_width), fill_value=-1, dtype=int)

    def populate_occupancy_grid(self, ranges, angle_increment, angle_min):
        """
        Populate occupancy grid using lidar scans and save
        the data in class member variable self.occupancy_grid.

        Optimization performed to improve the speed at which we generate the occupancy grid.

        Args:
            scan_msg (LaserScan): message from lidar scan topic
        """
        # reset empty occupacny grid (-1 = unknown)

        self.occupancy_grid = np.full(shape=(self.grid_height, self.grid_width), fill_value=self.IS_FREE, dtype=int)

        ranges = np.array(ranges)
       
        indices = np.arange(len(ranges))
      
        
        thetas = (indices * angle_increment) - self.ANGLE_OFFSET 

   
        xs = ranges * np.sin(thetas)
        ys = ranges * np.cos(thetas) * -1

        i, j = self.local_to_grid_parallel(xs, ys)

        occupied_indices = np.where((i >= 0) & (i < self.grid_height) & (j >= 0) & (j < self.grid_width))
        self.occupancy_grid[i[occupied_indices], j[occupied_indices]] = self.IS_OCCUPIED

    def convolve_occupancy_grid(self):
        kernel = np.ones(shape=[2, 2])
        self.occupancy_grid = signal.convolve2d(
            self.occupancy_grid.astype("int"), kernel.astype("int"), boundary="symm", mode="same")
        self.occupancy_grid = np.clip(self.occupancy_grid, -1, 100)
    
    def publish_occupancy_grid(self, frame_id, stamp, occupancy_grid_pub):
        """
        Publish populated occupancy grid to ros2 topic
        Args:
            scan_msg (LaserScan): message from lidar scan topic
        """
        oc = OccupancyGrid()
        oc.header.frame_id = frame_id
        oc.header.stamp = stamp

        oc.info.origin.position.y -= ((self.grid_width / 2) + 1) / self.CELLS_PER_METER

        oc.info.width = self.grid_height
        oc.info.height = self.grid_width
        oc.info.resolution = 1 / self.CELLS_PER_METER
        oc.data = np.fliplr(np.rot90(self.occupancy_grid, k=1)).flatten().tolist()
        occupancy_grid_pub.publish(oc)

    def local_to_grid_parallel(self, x, y):# Chuyển đổi tọa độ Cartesian của 1 điểm trong không gian sang chỉ số lưới chiếm dụng (grid indices) trong 1 occupancy grid
        i = np.round(x * -self.CELLS_PER_METER + (self.grid_height - 1)).astype(int)
        j = np.round(y * -self.CELLS_PER_METER + self.CELL_Y_OFFSET).astype(int)
        return i, j
    
    def grid_to_local(self, point):
        i, j = point[0], point[1]
        x = (i - (self.grid_height - 1)) / -self.CELLS_PER_METER
        y = (j - self.CELL_Y_OFFSET) / -self.CELLS_PER_METER
        return (x, y)
    
    def local_to_grid(self, x, y): 
        i = int(x * -self.CELLS_PER_METER + (self.grid_height - 1))
        j = int(y * -self.CELLS_PER_METER + self.CELL_Y_OFFSET)
        return (i, j)


class WaypointUtils:
    def __init__(self,
        node, L=1.7,
        interpolation_distance=None,
        filepath="/home/fablab_01/f1_ws/src/f1tenth_projects/f1tenth_navigation/scripts/racelines/e7_floor5.csv",
        min_lookahead=0.5,
        max_lookahead=3.0,
        min_lookahead_speed=3.0,
        max_lookahead_speed=6.0,
        filepath_2nd="/home/fablab_01/f1_ws/src/f1tenth_projects/f1tenth_navigation/scripts/racelines/e7_floor5.csv",
        lane_number=0,):
             
        self.node = node
        self.L = L 
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.min_lookahead_speed = min_lookahead_speed
        self.max_lookahead_speed = max_lookahead_speed

        self.waypoints_world, self.velocities = self.load_and_interpolate_waypoints(file_path=filepath, interpolation_distance=interpolation_distance)
        

        # For competition, where I want to customize the lanes that I am using
        self.lane_number = lane_number
        self.waypoints_world_2nd, self.velocities_2nd = self.load_and_interpolate_waypoints(file_path=filepath_2nd, interpolation_distance=interpolation_distance)
        
        self.index = 0
        self.velocity_index = 0

    def transform_waypoints(self, waypoints, car_position, pose):
        # translation
        waypoints = waypoints - car_position

        # rotation
        quaternion = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        waypoints = R.inv(R.from_quat(quaternion)).apply(waypoints)
        return waypoints
        
    def load_and_interpolate_waypoints(self, file_path:str, interpolation_distance:float=0.05)-> Tuple[np.ndarray, np.ndarray]:
        # Read waypoints from csv, first two columns are x and y, third column is velocity
        # Exclude last row, because that closes the loop
        points = np.genfromtxt(file_path, delimiter=",")[:, :2]  # Điểm giá trị (x, y)
        velocities = np.genfromtxt(file_path, delimiter=",")[:, 2] # vận tốc di chuyển 

        # Add first point as last point to complete loop
        # self.node.get_logger().info(str(velocities))

        # interpolate, not generally needed because interpolation can be done with the solver, where you feed in target distance between points
        if interpolation_distance != 0 and interpolation_distance is not None: # Điều kiện để nội suy 
            # Calculate the cumulative distances between points
            distances = np.sqrt(np.sum(np.diff(points, axis=0) ** 2, axis=1))
            cumulative_distances = np.insert(np.cumsum(distances), 0, 0)

            # Calculate the number of segments based on the desired distance threshold
            total_distance = cumulative_distances[-1]
            segments = int(total_distance / interpolation_distance)

            # Linear length along the line
            distance = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0) ** 2, axis=1)))
            # Normalize distance between 0 and 1
            distance = np.insert(distance, 0, 0) / distance[-1]

            # Interpolate
            alpha = np.linspace(0, 1, segments)
            interpolator = interp1d(distance, points, kind="slinear", axis=0)
            interpolated_points = interpolator(alpha)
           

            # Interpolate velocities
            velocity_interpolator = interp1d(distance, velocities, kind="slinear")
            interpolated_velocities = velocity_interpolator(alpha)

            # Add z-coordinate to be 0
            interpolated_points = np.hstack((interpolated_points, np.zeros((interpolated_points.shape[0], 1))))
            assert len(interpolated_points) == len(interpolated_velocities)
            # print("interpolated_points: ", interpolated_points)

            return interpolated_points, interpolated_velocities    # Trả về các điểm nội suy gồm cặp điểm (x, y) và velocities 

        else:
            # Add z-coordinate to be 0
            points = np.hstack((points, np.zeros((points.shape[0], 1))))
            return points, velocities

    

    def get_closest_waypoint_with_velocity(self, pose):
        # get current position of car
        if pose is None:
            return

        position = (pose.position.x, pose.position.y, 0)

        # transform way-points from world to vehicle frame of reference
        if self.lane_number == 0:
            waypoints_car = self.transform_waypoints(self.waypoints_world, position, pose)
        else:
            waypoints_car = self.transform_waypoints(self.waypoints_world_2nd, position, pose)

        # get distance from car to all waypoints
        distances = np.linalg.norm(waypoints_car, axis=1)

        # get indices of waypoints sorted by ascending distance
        self.velocity_index = np.argmin(distances)

        if self.lane_number == 0:
            return self.waypoints_world[self.velocity_index], self.velocities[self.velocity_index]
        else:
            return self.waypoints_world_2nd[self.velocity_index], self.velocities_2nd[self.velocity_index]



    def get_waypoint_stanley(self, pose):
        # get current position of car
        if pose is None:
            return
        position = (pose.position.x, pose.position.y, 0)

        # transform way-points from world to vehicle frame of reference
        if self.lane_number == 0:
            waypoints_car = self.transform_waypoints(self.waypoints_world, position, pose)
        else:
            waypoints_car = self.transform_waypoints(self.waypoints_world_2nd, position, pose)

        # get distance from car to all waypoints
        distances = np.linalg.norm(waypoints_car, axis=1)

        # get indices of waypoints sorted by ascending distance
        index = np.argmin(distances)

        if self.lane_number == 0:
            return waypoints_car[index], self.waypoints_world[index]
        else:
            return waypoints_car[index], self.waypoints_world_2nd[index]

    def get_waypoint(self, pose, target_velocity, fixed_lookahead=None):
        # get current position of car
        if pose is None:
            return
        position = (pose.position.x, pose.position.y, 0)

        # transform way-points from world to vehicle frame of reference
        if self.lane_number == 0:
            waypoints_car = self.transform_waypoints(self.waypoints_world, position, pose)
        else:
            waypoints_car = self.transform_waypoints(self.waypoints_world_2nd, position, pose)

        # get distance from car to all waypoints
        distances = np.linalg.norm(waypoints_car, axis=1)

        # get indices of waypoints that are within L, sorted by descending distance
        # Use dynamic lookahead for this part

        if fixed_lookahead:
            self.L = fixed_lookahead
        else:
            # Lookahead is proportional to velocity
            self.L = min(
                max(
                    self.min_lookahead,
                    self.min_lookahead
                    + (self.max_lookahead - self.min_lookahead)
                    * (target_velocity - self.min_lookahead_speed)
                    / (self.max_lookahead_speed - self.min_lookahead_speed),
                ),
                self.max_lookahead,
            )

        indices_L = np.argsort(np.where(distances < self.L, distances, -1))[::-1]

        # set goal point to be the farthest valid waypoint within distance L
        for i in indices_L:
            # check waypoint is in front of car
            x = waypoints_car[i][0]
            if x > 0:
                self.index = i
                if self.lane_number == 0:
                    return waypoints_car[self.index], self.waypoints_world[self.index]
                else:
                    return waypoints_car[self.index], self.waypoints_world_2nd[self.index]
        return None, None



class Utils:
    def __init__(self):
        pass
    def draw_marker(self, frame_id, stamp, position, publisher, color="red", id=0):
        if position is None:
            return
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.id = id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        if color == "red":
            marker.color.r = 1.0
        elif color == "green":
            marker.color.g = 1.0
        elif color == "blue":
            marker.color.b = 1.0
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.0
        publisher.publish(marker)

    def draw_marker_array(self, frame_id, stamp, positions, publisher):
        marker_array = MarkerArray()
        for i, position in enumerate(positions):
            if position is None:
                continue
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = 0.0
            marker.lifetime = Duration(seconds=0.1).to_msg()
            marker_array.markers.append(marker)
        publisher.publish(marker_array)

    def draw_lines(self, frame_id, stamp, path, publisher):
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
        line_list.header.stamp = stamp
        line_list.id = 0
        line_list.type = line_list.LINE_LIST
        line_list.action = line_list.ADD
        line_list.scale.x = 0.1
        line_list.color.a = 1.0
        line_list.color.r = 0.0
        line_list.color.g = 1.0
        line_list.color.b = 0.0
        line_list.points = points
        publisher.publish(line_list)

    def traverse_grid(self, start, end):
        """
        Bresenham's line algorithm for fast voxel traversal

        CREDIT TO: Rogue Basin
        CODE TAKEN FROM: http://www.roguebasin.com/index.php/Bresenham%27s_Line_Algorithm
        """
        # Setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1

        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)

        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # Swap start and end points if necessary and store swap state
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1

        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1

        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1

        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
        return points







class VisualizeMarkers:
    def __init__(self, node: Node):
        self.node = node
        self.marker_pub = self.node.create_publisher(Marker, 'start_stop_marker', 10)

    def visualize_positions(self, current_pos: np.ndarray, goal_pos: Optional[np.ndarray], frame_id: str = "base_link"):
        """
        Visualize current and goal positions on RViz.

        Args:
            current_pos (np.ndarray): Current position (i, j) in grid.
            goal_pos (Optional[np.ndarray]): Goal position (i, j) in grid. Use None if not available.
            frame_id (str): The reference frame in which to visualize (default: "base_link").
        """
        # Current Position Marker
        if current_pos is not None:
            current_marker = Marker()
            current_marker.header.frame_id = frame_id
            current_marker.header.stamp = self.node.get_clock().now().to_msg()
            current_marker.ns = "current_position"
            current_marker.id = 0
            current_marker.type = Marker.SPHERE
            current_marker.action = Marker.ADD
            current_marker.pose.position.x = float(current_pos[0])
            current_marker.pose.position.y = float(current_pos[1])
            current_marker.pose.position.z = 0.0
            current_marker.scale.x = 0.2
            current_marker.scale.y = 0.2
            current_marker.scale.z = 0.2
            current_marker.color.a = 1.0
            current_marker.color.r = 1.0  # Red for current position
            current_marker.color.g = 0.0
            current_marker.color.b = 0.0
            self.marker_pub.publish(current_marker)

        # Goal Position Marker (only if goal_pos is not None)
        if goal_pos is not None:
            goal_marker = Marker()
            goal_marker.header.frame_id = frame_id
            goal_marker.header.stamp = self.node.get_clock().now().to_msg()
            goal_marker.ns = "goal_position"
            goal_marker.id = 1
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = float(goal_pos[0])
            goal_marker.pose.position.y = float(goal_pos[1])
            goal_marker.pose.position.z = 0.0
            goal_marker.scale.x = 0.1
            goal_marker.scale.y = 0.1
            goal_marker.scale.z = 0.1
            goal_marker.color.a = 1.0
            goal_marker.color.r = 0.0
            goal_marker.color.g = 1.0  # Green for goal position
            goal_marker.color.b = 0.0
            self.marker_pub.publish(goal_marker)
