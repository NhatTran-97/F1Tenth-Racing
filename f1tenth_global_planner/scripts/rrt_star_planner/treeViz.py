#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from typing import Optional, Tuple

# Pick Color: https://rgbcolorpicker.com/0-1

class TreeViz:
    def __init__(self, node, resolution=0.1, origin=(-4.0,-2.0), id=0, frame="map"):

        self.candidate_points=[]
        self.saved_points = []
        self.saved_positions = []
        self.tested_lines = []

        self.resolution = resolution
        self.origin = origin

        self.id = id
        self.frame = frame
        self.node = node 
        
        # Create a Publisher 
        self.publisher_path = self.node.create_publisher(Marker, '/path_marker', 10)

        self.sample_point = self.node.create_publisher(Marker, '/sample_points', 10)

        self.target_point_publisher = self.node.create_publisher(Marker, '/target_points', 10)

        self.candidate_point_publisher = self.node.create_publisher(Marker, '/candidate_points', 10)

        self.line_publisher = self.node.create_publisher(Marker, '/bresenham_line', 10)

        self.plot_tree = self.node.create_publisher(Marker, '/rrt_tree', 10)

        self.search_space_pub = self.node.create_publisher(Marker, 'search_radius', 10)


        self.path = []  # Danh sách các điểm trong path


        self.tree_viz_pub = self.node.create_publisher(MarkerArray, 'rrt_star_tree', 10)
        self.marker_array = MarkerArray()
        self.node_counter = 0


        # Initialize Marker for Trees (segments_marker)

        self.segments_marker = Marker(
            header=Header(frame_id="map"),ns="Tree",id=self.id,
            type=Marker.LINE_LIST,
            action=Marker.ADD,
            scale=Vector3(x=0.05, y=0.05, z=-0.01),
            color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),)
        
        self.segments_marker.pose.position.x = 0.0
        self.segments_marker.pose.position.y = 0.0
        self.segments_marker.pose.position.z = 0.0
        self.segments_marker.pose.orientation.x = 0.0
        self.segments_marker.pose.orientation.y = 0.0
        self.segments_marker.pose.orientation.z = 0.0
        self.segments_marker.pose.orientation.w = 1.0

        # Initialize Marker for Nodes
        self.nodes_marker = Marker(
            header=Header(frame_id="map"),
            ns="Nodes",
            id=self.id,
            type=Marker.POINTS,
            action=Marker.ADD,
            scale=Vector3(x=0.2, y=0.2, z=0.5),
            color=ColorRGBA(r=0.0, g=1.0, b=0.212, a=1.0),
        )
        self.nodes_marker.pose.position.x = 0.0
        self.nodes_marker.pose.position.y = 0.0
        self.nodes_marker.pose.position.z = 0.0
        self.nodes_marker.pose.orientation.x = 0.0
        self.nodes_marker.pose.orientation.y = 0.0
        self.nodes_marker.pose.orientation.z = 0.0
        self.nodes_marker.pose.orientation.w = 1.0


    def set_path(self, grid_path):
        """Cập nhật đường path từ tọa độ grid cell."""
        self.path = []
        for xy_grid in grid_path:
            world_coord = self.grid_to_world_coord(xy_grid)
            self.path.append((world_coord[0], world_coord[1], 0.1))  # Z = 0 cho 2D
        
        self.publish_path_marker()

    def publish_path_marker(self):
        """Publish path lên RViz dưới dạng LINE_STRIP."""
        if not self.path:
            return

        marker = Marker()
        marker.header.frame_id = "base_link"  # Frame id, đổi nếu cần
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Thêm các điểm vào Marker
        for coord in self.path:
            point = Point()
            point.x = coord[0]
            point.y = coord[1]
            point.z = 0.0#coord[2]
            marker.points.append(point)

        # Cấu hình hiển thị đường
        marker.scale.x = 0.07  # Độ dày của đường
        marker.color.r = 1.0   # Màu đỏ
        marker.color.g = 0.0   # Màu xanh lá
        marker.color.b = 0.0   # Màu xanh dương
        marker.color.a = 1.0   # Độ trong suốt

        self.publisher_path.publish(marker)
        self.node.get_logger().info("Path marker published to RViz")

    def append(self, node):
        if node.parent is None:
            # if root_node, just indicate node
            node_coords = self.grid_to_world_coord(node.coordinates)
            self.nodes_marker.points.append(Point(x=node_coords[0], y=node_coords[1], z=0.1))
      
        else:
            # Convert from grid to world coordinate
            start_point = self.grid_to_world_coord(node.parent.coordinates)
            end_point = self.grid_to_world_coord(node.coordinates)

            # Add line segment from parent to current node
            self.segments_marker.points.append(Point(x=start_point[0], y=start_point[1], z=0.1))
            self.segments_marker.points.append(Point(x=end_point[0], y=end_point[1], z=0.1))

             # Add current node into list
            self.nodes_marker.points.append(Point(x=end_point[0], y=end_point[1], z=0.1))


            # Update header timestamp before publishing
            self.segments_marker.header.stamp = self.node.get_clock().now().to_msg()
            self.nodes_marker.header.stamp = self.node.get_clock().now().to_msg()

            # Test connection with Publisher and send markers
            while self.plot_tree.get_subscription_count() < 1:
                self.node.get_logger().info("Waiting for a connection to Rviz marker publisher...")

            # Publish marker
            self.plot_tree.publish(self.segments_marker)
            self.plot_tree.publish(self.nodes_marker)

    def grid_to_world_coord(self, xy_grid):
        """
        Chuyển đổi tọa độ lưới (x, y) thành tọa độ thực tế trong frame map.
        """
        world_coordinates = []
        world_coordinates.append(self.resolution * xy_grid[0] + self.origin[0])
        world_coordinates.append(self.resolution * xy_grid[1] + self.origin[1])
        return world_coordinates
    
    def gridcels_to_world(self, width_cell, height_cell):
        """
        Chuyển đổi tọa độ ô lưới trong bản đồ sang tọa độ thế giới (Rviz).
        
        Parameters:
            grid_x (int): Tọa độ X của ô lưới trong bản đồ.
            grid_y (int): Tọa độ Y của ô lưới trong bản đồ.
            resolution (float): Độ phân giải của bản đồ (kích thước mỗi ô lưới).
            origin (list): Gốc bản đồ trong Rviz [origin_x, origin_y].
            
        Returns:
            tuple: (x_rviz, y_rviz) tọa độ trong hệ Rviz.
        """
        if not isinstance(self.origin, (list, tuple)) or len(self.origin) != 2:
            self.node.get_logger().error(f"Invalid origin: {self.origin} (type: {type(self.origin)})")
            raise ValueError("Origin must be a list or tuple with exactly two elements [x, y]")

        x_world = float(width_cell * self.resolution + self.origin[0])
        y_world = float(height_cell * self.resolution + self.origin[1])
        return x_world, y_world
    
    def publish_candidate_point_marker(self, candidate, color=None):

        x_world, y_world = self.gridcels_to_world(candidate[0], candidate[1])
        self.candidate_points.append((x_world, y_world)) 

        marker = Marker()
        marker.header.frame_id = "map"  # Tham chiếu đến frame của map
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "candidate_points"
        marker.id = 0  # Mỗi marker có một ID duy nhất
        # marker.id = len(self.saved_points) 
        marker.type = Marker.POINTS  # Marker hình cầu
        marker.action = Marker.ADD  # Thêm marker

        # Kích thước marker
        marker.scale.x = 0.01  # Đặt kích thước của điểm (đường kính)
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        if color is None:
            color = (1.0, 1.0, 0.0, 1.0)  # Mặc định là màu đỏ

        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])

        for point in self.candidate_points:
            marker.points.append(Point(x=point[0], y=point[1], z=0.1))

        self.candidate_point_publisher.publish(marker)

    def publish_sample_point_marker(self, width_Gridcell, height_Gridcel, color=None):

        """
        Xuất bản một marker để hiển thị điểm sample lên RViz
           
        Xuất bản một marker để hiển thị điểm sample lên RViz với màu sắc tùy chỉnh.

        Parameters:
            width_gridcell (int): Tọa độ X của ô lưới.
            height_gridcell (int): Tọa độ Y của ô lưới.
            color (tuple): Màu sắc RGBA (mặc định là màu đỏ).
        """
        

        x_world, y_world = self.gridcels_to_world(width_Gridcell, height_Gridcel)

        # Lưu điểm mới vào danh sách
        self.saved_points.append((x_world, y_world))
        
        marker = Marker()
        marker.header.frame_id = "map"  # Tham chiếu đến frame của map
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "sample_points"
        marker.id = 0  # Mỗi marker có một ID duy nhất
        # marker.id = len(self.saved_points) 
        marker.type = Marker.POINTS  # Marker hình cầu
        marker.action = Marker.ADD  # Thêm marker

        # Kích thước marker
        marker.scale.x = 0.01  # Đặt kích thước của điểm (đường kính)
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        if color is None:
            color = (1.0, 0.0, 0.0, 1.0)  # Mặc định là màu đỏ

        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])

        for point in self.saved_points:
            marker.points.append(Point(x=point[0], y=point[1], z=0.1))

        # Xuất bản marker
        self.sample_point.publish(marker)
                            
    def publish_initial_and_goal(self, initial_point:Tuple[float, float]=None, goal_point=None):
        """
        Publish both initial and goal points to RViz.

        Parameters:
            initial_point (tuple): Tọa độ (x, y) của điểm bắt đầu trong hệ tọa độ grid.
            goal_point (tuple): Tọa độ (x, y) của điểm mục tiêu trong hệ tọa độ grid.
        """

        if initial_point is not None:
            # Marker cho initial point
            initial_marker = Marker()
            initial_marker.header.frame_id = "base_link"
            initial_marker.header.stamp = self.node.get_clock().now().to_msg()
            initial_marker.ns = "initial_point"
            initial_marker.id = 0
            initial_marker.type = Marker.SPHERE
            initial_marker.action = Marker.ADD

            initial_marker.pose.position.x = float(initial_point[0])
            initial_marker.pose.position.y = float(initial_point[1])
            initial_marker.pose.position.z = 0.1

            initial_marker.scale.x = 0.1
            initial_marker.scale.y = 0.1
            initial_marker.scale.z = 0.1
            initial_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Màu xanh lá cây
            self.target_point_publisher.publish(initial_marker)

        if goal_point is not None:
            # Marker cho goal point
            goal_marker = Marker()
            goal_marker.header.frame_id = "base_link"
            goal_marker.header.stamp = self.node.get_clock().now().to_msg()
            goal_marker.ns = "goal_point"
            goal_marker.id = 1
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = float(goal_point[0])
            goal_marker.pose.position.y =float(goal_point[1])
            goal_marker.pose.position.z = 0.1
            goal_marker.scale.x = 0.2
            goal_marker.scale.y = 0.2
            goal_marker.scale.z = 0.2
            goal_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Màu xanh dương

            # Xuất bản cả hai điểm
            self.target_point_publisher.publish(goal_marker)

    def visualize_line(self, covered_cells, color=(0.0, 1.0, 0.0, 1.0)):
        """
        Hiển thị tất cả các đường thẳng từ `covered_cells` lên RViz.
        """

        # Khởi tạo danh sách nếu chưa có
        if not hasattr(self, "tested_lines"):
            self.tested_lines = []

        # Lưu các cell hiện tại vào danh sách `tested_lines`
        for i in range(len(covered_cells) - 1):
            x1, y1 = self.gridcels_to_world(covered_cells[i][0], covered_cells[i][1])
            x2, y2 = self.gridcels_to_world(covered_cells[i + 1][0], covered_cells[i + 1][1])
            self.tested_lines.append(((x1, y1), (x2, y2)))

        # Tạo Marker
        marker = Marker()
        marker.header.frame_id = "map"  # Frame tham chiếu là "map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "bresenham_lines"
        marker.id = 0
        marker.type = Marker.LINE_LIST  # Sử dụng LINE_LIST để hiển thị nhiều đoạn thẳng
        marker.action = Marker.ADD

        # Kích thước đường thẳng
        marker.scale.x = 0.1  # Độ dày đường thẳng

        # Màu sắc
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])

        # Thêm tất cả các đoạn thẳng vào marker
        for line in self.tested_lines:
            start, end = line
            marker.points.append(Point(x=start[0], y=start[1], z=0.1))
            marker.points.append(Point(x=end[0], y=end[1], z=0.1))

        # Xuất bản marker
        self.line_publisher.publish(marker)
        # self.node.get_logger().info(f"Visualized {len(self.tested_lines)} lines with {len(marker.points)} points.")

    def visualize_search_radius(self, node, radius: float):
        """
        Visualize the search radius in RViz2 as a marker.
        """

        x_world, y_world = self.gridcels_to_world(node.coordinates[0], node.coordinates[1])
        marker = Marker()
        marker.header.frame_id = "map"  # Replace with your frame (e.g., "odom" or "world")
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "search_radius"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set the position to the node's coordinates
        marker.pose.position.x = float(x_world)
        marker.pose.position.y = float(y_world)
        marker.pose.position.z = 0.0

        # Set the size of the sphere
        marker.scale.x = float(radius*0.05 * 2)  # Diameter in x
        marker.scale.y = float(radius*0.05 * 2)  # Diameter in y
        marker.scale.z = 0.1  # Flat circle

        # Set color (RGBA)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5  # Transparency

        # Publish the marker
        self.search_space_pub.publish(marker)

