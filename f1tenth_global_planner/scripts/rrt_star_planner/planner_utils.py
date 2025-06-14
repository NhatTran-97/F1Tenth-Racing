import rclpy
from typing import Optional, Tuple, List
from unittest.mock import Mock
import unittest
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
import rclpy.time
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
import time
import math

class Planner_Utils():
    def __init__(self, node:rclpy, tf_buffer: Buffer):
        self.node = node
        self.tf_buffer = tf_buffer
        
    def quaternion_to_yaw(self, orientation):
        """Chuyển đổi quaternion thành góc yaw (radians)"""

        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def transform_map_to_base_link(self, frame_id: str, x: float, y: float, z: float = 0.0) -> Optional[Tuple[float, float]]:
        """
        Chuyển đổi tọa độ từ frame nguồn sang `base_link`.

        :param frame_id: Tên khung tọa độ nguồn (ví dụ: 'map').
        :param x: Tọa độ x trong khung nguồn.
        :param y: Tọa độ y trong khung nguồn.
        :param z: Tọa độ z trong khung nguồn (mặc định là 0.0).
        :return: Tọa độ (x, y) trong khung `base_link` hoặc None nếu có lỗi.
        """
        if not frame_id:
            self.node.get_logger().warn("Frame ID is invalid. Transformation aborted.")
            return None

        # self.node.get_logger().info(f"Transforming point ({x}, {y}, {z}) from '{frame_id}' to 'base_link'.")

        timeout = 5.0  # Thời gian chờ tối đa (giây)
        start_time = time.time()

        try:
            while not self.tf_buffer.can_transform('base_link', frame_id, rclpy.time.Time()):
                if time.time() - start_time > timeout:
                    self.node.get_logger().warn(
                        f"Timeout while waiting for transform from '{frame_id}' to 'base_link'. Skipping transformation."
                    )
                    return None
                self.node.get_logger().info("Waiting for transform to become available...")
                # time.sleep(0.1)   
                # rclpy.spin_once(self.node)
                # self.node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
                rclpy.time(0.1)

            # Tạo một PointStamped
            point = PointStamped()
            point.header.frame_id = frame_id
            point.header.stamp = self.node.get_clock().now().to_msg()
            point.point.x = float(x)
            point.point.y = float(y)
            point.point.z = float(z)

            # Lấy transform
            """
            # Phép biến đổi này lấy vị trí và hướng của robot (base_link) trong hệ tọa độ map. Hay nói cách khác:
            Dịch ngược: Phép biến đổi mô tả cách một điểm hoặc tọa độ trong map sẽ trông như thế nào khi quan sát từ ego_racecar/base_link.
            Chiều chuyển đổi: Từ global_refFrame (map) → car_refFrame (ego_racecar/base_link
            """
            transform: TransformStamped = self.tf_buffer.lookup_transform( 
   
                target_frame='base_link',
                source_frame=frame_id,
                time=rclpy.time.Time()
            )
            
            # Thực hiện transform
            transformed_point = do_transform_point(point, transform)

            # self.node.get_logger().info(
            #     f"Transformed point to (x: {transformed_point.point.x}, y: {transformed_point.point.y}) in 'base_link'.")

            return (transformed_point.point.x, transformed_point.point.y)

        except TransformException as ex:
            self.node.get_logger().warn(f"Failed to transform point: {ex}")
            return None
        except Exception as ex:
            self.node.get_logger().error(f"Unexpected exception during transformation: {ex}")
            return None
        
    # 📏 Calculate Distance
    # ------------------------------------------
    def calculate_distance(self, point1: Tuple[float, float], point2: Tuple[float, float]) -> float:

        """
        Calculate the distance between two points 
        """
        return ((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)**0.5
    
    def find_local_goal(self, robot_pose:Tuple[float, float], waypoints:List[Tuple[float, float]], target_distance:float=2.0)-> Optional[Tuple[float, float]]:
        """
        Tìm điểm goal gần nhất trên waypoint cách robot khoảng target_distance
        Params: 
        - robot_pose: Tuple (x, y) vị trí hiện tại của robot.
        - Danh sách waypoints dưới dạng np.array([[x1, y1], [x2, y2], ...]).
        - target_distance: Khoảng cách mục tiêu từ robot đến goal point.
        - Tuple (x, y) của goal point hoặc None nếu không tìm thấy.
        """
        closest_point = None
        closest_distance_diff = float('inf')
        for waypoint in waypoints:
            distance_pose_to_goal = self.calculate_distance(robot_pose, waypoint)
            
            if distance_pose_to_goal >= target_distance and distance_pose_to_goal - target_distance < closest_distance_diff:
                closest_point = waypoint
                closest_distance_diff = abs(distance_pose_to_goal - target_distance)
        return tuple(closest_point)



    def world_to_grid_cell(self, wx:float, wy:float, width:int, height:int, resolution:float, origin:Tuple[float, float])-> Optional[Tuple[int, int]]:
        """Convert world coordinates to 2D map index."""
        mx = int((wx - origin[0]) / resolution)
        my = int((wy - origin[1]) / resolution)
        # Kiểm tra xem tọa độ có nằm trong ranh giới của bản đồ không
        if 0 <= mx < width and 0 <= my < height:
            return (mx, my)
        else:
            self.node.get_logger().error(f"World coordinates ({wx}, {wy}) are out of grid bounds.")
            return None
        
    # def local_to_grid(self, x, y): # Chuyển đổi tọa độ Cartesian của 1 điểm trong không gian sang chỉ số lưới chiếm dụng (grid indices) trong 1 occupancy grid
    #     i = int(x * -self.CELLS_PER_METER + (self.grid_height - 1))
    #     j = int(y * -self.CELLS_PER_METER + self.CELL_Y_OFFSET)
    #     return (i, j)


        
    def world_to_map_index(self, wx:float, wy:float, width:int, height:int, resolution:float, origin:Tuple[float, float])-> Optional[int]:
        """Convert world coordinates to 2D map index."""
        mx = int((wx - origin[0]) / resolution)
        my = int((wy - origin[1]) / resolution)

        """Convert 2D map to index map"""
        if 0 <= mx < width and 0 <= my < height:
            return my * width + mx 
        else:
            self.node.get_logger().error(f"World coordinates ({wx}, {wy}) are out of map bounds.")
            return None     
        
    def index_to_grid_cell(self, flat_map_index, map_width):
        """
        Converts a linear index of a flat map to grid cell coordinate values // Chuyển đổi từ index 1D => tọa độ 2D [x, y] để biểu diễn không gian 2D trên bản đồ
        """
        if flat_map_index is None:
            self.node.get_logger().error("flat_map_index is None! Cannot calculate grid cell.")
            return None
        if map_width is None or map_width == 0:
            self.node.get_logger().error("map_width is None or zero! Cannot calculate grid cell.")
            return None

        grid_cell_map_x = flat_map_index % map_width
        grid_cell_map_y = flat_map_index // map_width
        return (grid_cell_map_x, grid_cell_map_y)


class TestPlannerUtils(unittest.TestCase):
    def setUp(self):
        """Hàm khởi tạo cho mỗi test case."""
        mock_node = Mock()
        mock_logger = Mock()
        mock_node.get_logger.return_value = mock_logger
        self.planner_utils = Planner_Utils(node=mock_node)

    ## 1. Test world_to_map_index ##
    def test_world_to_map_index_valid(self):
        """Kiểm tra với tọa độ hợp lệ."""
        index = self.planner_utils.world_to_map_index(
            wx=2.0,
            wy=3.0,
            width=10,
            height=10,
            resolution=0.5,
            origin=(0.0, 0.0)
        )
        self.assertEqual(index, 26)  # (4, 6) trên bản đồ 10x10 → 6 * 10 + 4 = 26

    def test_world_to_map_index_out_of_bounds(self):
        """Kiểm tra với tọa độ ngoài phạm vi."""
        index = self.planner_utils.world_to_map_index(
            wx=11.0,
            wy=11.0,
            width=10,
            height=10,
            resolution=1.0,
            origin=(0.0, 0.0)
        )
        self.assertIsNone(index)
        self.planner_utils.node.get_logger().error.assert_called_with(
            "World coordinates (11.0, 11.0) are out of map bounds."
        )

    # ## 2. Test index_to_grid_cell ##
    # def test_index_to_grid_cell_valid(self):
    #     """Kiểm tra chuyển đổi index hợp lệ sang tọa độ 2D."""
    #     cell = self.planner_utils.index_to_grid_cell(flat_map_index=23, map_width=5)
    #     self.assertEqual(cell, (3, 4))  # (23 % 5, 23 // 5) → (3, 4)

    # def test_index_to_grid_cell_invalid_index(self):
    #     """Kiểm tra với index là None."""
    #     cell = self.planner_utils.index_to_grid_cell(flat_map_index=None, map_width=5)
    #     self.assertIsNone(cell)
    #     self.planner_utils.node.get_logger().error.assert_called_with(
    #         "flat_map_index is None! Cannot calculate grid cell."
    #     )

    # def test_index_to_grid_cell_invalid_width(self):
    #     """Kiểm tra với width không hợp lệ."""
    #     cell = self.planner_utils.index_to_grid_cell(flat_map_index=23, map_width=0)
    #     self.assertIsNone(cell)
    #     self.planner_utils.node.get_logger().error.assert_called_with(
    #         "map_width is None or zero! Cannot calculate grid cell."
    #     )

if __name__ == '__main__':
    unittest.main()