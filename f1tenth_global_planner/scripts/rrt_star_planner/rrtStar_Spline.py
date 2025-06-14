#!/usr/bin/env python3

from random import randrange as rand
import math
from bresenham import bresenham
from typing import List, Tuple, Dict, Optional
from rclpy.node import Node
from visualization_msgs.msg import Marker
from cubic_spline import CUBIC_SPLINE
import random
import numpy as np

# ================================
# 📚 Create_Node Class
# ================================
class Create_Node:
    def __init__(self, coordinates=None, parent=None, cost=None):
        self.coordinates = coordinates # (x, y)
        self.parent = parent 
        self.cost = cost

    def __str__(self):
            parent_coords = self.parent.coordinates if self.parent else None
            return f"TreeNode(coordinates={self.coordinates}, parent={parent_coords})"
    def __repr__(self):
        return self.__str__()

# ================================
# 📚 RRTStar Class
# ================================

class RRTStar():
    def __init__(self, node:Node)->None:
        self.node = node
        
    # 📏 Calculate Distance
    # ------------------------------------------
    def calculate_distance(self, point1: Tuple[int, int], point2: Tuple[int, int]) -> float:

        """
        Calculate the distance between two points 
        """
        return ((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)**0.5
    # 📐 Calculate Angle
    # ------------------------------------------
    def calculate_angle(self, p1: Tuple[int, int], p2: Tuple[int, int]) -> float:

        return math.atan2((p2[1] - p1[1]), (p2[0] - p1[0]))
    # 🌳 Find Closest Node
    # ------------------------------------------

    def find_closest_node(self, random_pt: Tuple[int, int], node_list:List[Create_Node]) -> Create_Node:
        """
            Finds the closest node in the tree

            random_pt: a (X, Y) point
            node_list: list that keeps all nodes in the tree
            returns: Node instance that is the closest node in the tree
        """
        return min(node_list, key=lambda node: self.calculate_distance(node.coordinates, random_pt))

    # 🟢 Check if Cell is Free
    # ------------------------------------------

    def is_free(self, cell_x: int, cell_y: int, map_data: List[int], width: int, height: int, resolution: float) -> bool:
        """
        Kiểm tra ô có bị chiếm dụng hay không.
        """
        try:
            # Ép kiểu tọa độ thành số nguyên
            cell_x = int(cell_x)
            cell_y = int(cell_y)
            
            # Kiểm tra phạm vi hợp lệ
            if cell_x < 0 or cell_x >= width or cell_y < 0 or cell_y >= height:
                return False
            
            index = cell_y * width + cell_x
            
            # Kiểm tra giá trị tại index
            if map_data[index] == 1:
                return False

            # Kiểm tra các ô lân cận
            cells_extra = math.ceil(2.0 / 2.0 / resolution)
            for i in range(1, cells_extra + 1):
                if (cell_x - i >= 0 and map_data[cell_y * width + (cell_x - i)] != 0) or \
                (cell_x + i < width and map_data[cell_y * width + (cell_x + i)] != 0) or \
                (cell_y - i >= 0 and map_data[(cell_y - i) * width + cell_x] != 0) or \
                (cell_y + i < height and map_data[(cell_y + i) * width + cell_x] != 0):
                    return False
            
            return True

        except IndexError as e:
            self.node.get_logger().error(f"IndexError in is_free: {e}. cell_x: {cell_x}, cell_y: {cell_y}, index: {index}")
            return False
        except Exception as e:
            self.node.get_logger().error(f"Unexpected error in is_free: {e}")
            return False
    
    # 💥 Collision Detection
    # ------------------------------------------
    def collision_detected(self, p1: Tuple[int, int], p2: Tuple[int, int], map_data: List[int], map_width: int, tree_viz: Marker = None) -> bool:
        """
        Check for collisions along a line segment between two points on the map.
        
        Parameters:
        - p1 (Tuple[int, int]): Start point (x, y).
        - p2 (Tuple[int, int]): End point (x, y).
        - map_data (List[int]): 1D representation of the map.
        - map_width (int): Width of the map grid.
        - tree_viz (Marker, optional): Visualization tool for debugging.

        Returns:
        - bool: True if a collision is detected, False otherwise.
        """
        # Ensure integer coordinates for Bresenham
        p1 = (int(p1[0]), int(p1[1]))
        p2 = (int(p2[0]), int(p2[1]))
        
        # Compute cells covered by the line p1-p2 using Bresenham's algorithm
        covered_cells = list(bresenham(p1[0], p1[1], p2[0], p2[1]))
        if tree_viz is not None: 
            tree_viz.visualize_line(covered_cells)

        for cell in covered_cells:
            # Validate cell bounds
            if cell[0] < 0 or cell[1] < 0 or cell[0] >= map_width or cell[1] * map_width >= len(map_data):
                continue  # Skip invalid cells (out of bounds)
            
            # Check if the cell is occupied
            if map_data[cell[0] + map_width * cell[1]]:
                return True  # Collision detected

        return False  # No collision detected

    # 🌱 Create New Branch Point
    # ------------------------------------------
  
    def create_new_branch_point(self, closest_node: Create_Node, p2: Tuple[int, int], max_distance: float) -> Optional[Create_Node]:
        """
        Creates a new branch point from the closest node towards a target point, limited by a maximum distance.

        Parameters:
        - closest_node (Create_Node): The starting node, containing coordinates, parent, and cost.
        - p2 (Tuple[int, int]): The target point represented as (x, y).
        - max_distance (float): The maximum allowable distance for creating a new branch.
                                If the distance to the target is smaller, it will be limited to that distance.

        Returns:
        - Create_Node: A new node created towards the target point with updated coordinates, parent, and cost.
        - None: If an exception occurs.

        Exceptions:
        - ValueError: If max_distance is non-positive.
        - AttributeError: If there is an issue with accessing object attributes.
        - Exception: For any unexpected errors.
        """
        try:
            # Kiểm tra max_distance hợp lệ
            if max_distance <= 0:
                raise ValueError("max_distance must be a positive float greater than 0.")

            # Calculate the Euclidean distance between the closest node and the target point (p2)
            d = self.calculate_distance(closest_node.coordinates, p2)  
            
            # Calculate the angle (in radians) from the closest node towards the target point (p2)
            theta = self.calculate_angle(closest_node.coordinates, p2)  

            # GConstrain the distance
            effective_distance = min(d, max_distance)

            # Create new coordinate
            new_x = closest_node.coordinates[0] + round(effective_distance * math.cos(theta), 2)
            new_y = closest_node.coordinates[1] + round(effective_distance * math.sin(theta), 2)

            # Tạo nút mới
            candidate_node = Create_Node(
                coordinates=(new_x, new_y),
                parent=closest_node,
                cost=closest_node.cost + effective_distance
            )

            return candidate_node

        except ValueError as ve:
            self.node.get_logger().error(f"ValueError in create_new_branch_point: {str(ve)}")
        except AttributeError as ae:
            self.node.get_logger().error(f"AttributeError in create_new_branch_point: {str(ae)}")
        except TypeError as te:
            self.node.get_logger().error(f"TypeError in create_new_branch_point: {str(te)}")
        except Exception as e:
            self.node.get_logger().error(f"Unexpected error in create_new_branch_point: {str(e)}")
        
        return None

    # ------------------------------------------
    # 🔄 Find Neighbors
    # ------------------------------------------
    def find_neighbors(self, tree:List[Create_Node], node:Create_Node, search_radius = 15, tree_viz:Marker=None)->List[Create_Node]:
        """This method should return the cost of the neighborhood of nodes around the given node
        Args:
            tree ([]): Current tree as a list of Nodes
            node: Current node we're finding neighbors
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes

        """
        if tree_viz is not None: 
            tree_viz.visualize_search_radius(node, search_radius)

        return [n for n in tree if self.calculate_distance(n.coordinates, node.coordinates) <= search_radius]

    # 📊 Adaptive Search Radius
    # ------------------------------------------
    def adaptive_search_radius(self, num_nodes: int, dimension:int, gamma: float=1.0)-> float:
        """
        Calculate an adaptive search radius based on the number of nodes and the search space dimension.

        Parameters:
        - num_nodes (int): The current number of nodes in the tree (must be >= 1).
        - dimension (int): The dimension of the search space (must be > 0).
        - gamma (float): A scaling factor to adjust the radius (default is 1.0, must be > 0).

        Returns:
        - float: The calculated search radius.
        - None:
        """
        if num_nodes <= 0:
            raise ValueError("num_nodes must be greater than 0.")
        if dimension <= 0:
            raise ValueError("dimension must be greater than 0.")
        if gamma <= 0:
            raise ValueError("gamma must be a positive number.")
        
         # Calculate adaptive search radius
        try:
            
            return gamma * (math.log(num_nodes) / num_nodes) ** (1 / dimension)
        except (ValueError, ZeroDivisionError) as e:
            self.node.get_logger().error(f"Error calculating adaptive_search_radius: {e}")
            return None
    
    # 📏 Line Cost
    # ------------------------------------------
    def line_cost(self, n1: Create_Node, n2: Create_Node) -> float:
        """
        This method should return the cost of straight line between n1 and n2
        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straight line

        Returns:
            cost (float): The cost value of the line
        """
        return self.calculate_distance(n1.coordinates, n2.coordinates)
    # 🧭 Check Goal
    # ------------------------------------------

    def check_goal(self, p1: Create_Node, p_goal: Tuple[int, int], tolerance: int) -> bool:
        """
        Test if the goal has been reached considering a tolerance distance.

        Parameters:
        - p1: A TreeNode containing the current point coordinates.
        - p_goal: A tuple (x, y) representing the goal point.
        - tolerance: An integer representing the tolerance distance.

        Returns:
        - True: Goal is within tolerance, False if goal is not within tolerance
        """
        return self.calculate_distance(p1.coordinates, p_goal) <= tolerance

    # ------------------------------------------
    # 🚶 Reconstruct Path
    # ------------------------------------------
    def reconstruct_path(self, candidate_node: Create_Node, target_position:Tuple[int, int]) -> List[Tuple[int, int]]:
        path = []
        path.append(target_position)
        node = candidate_node
        while node.parent:
            path.append(node.coordinates)
            node = node.parent
        path.append(node.coordinates)
        return path[::-1]


    def adaptive_goal_bias(self, current_position: Tuple[int, int], 
                            target_position: Tuple[int, int], 
                            map_data: List[int], 
                            width: int, 
                            height: int) -> float:
        """
        Adaptive Goal Bias điều chỉnh dựa vào khoảng cách đến mục tiêu và mật độ vật cản.
        """
        # Giá trị mặc định
        goal_bias = 0.5  

        # 1️⃣ Tính khoảng cách đến mục tiêu
        distance = self.calculate_distance(current_position, target_position)
        
        # 2️⃣ Tính mật độ vật cản xung quanh
        x, y = map(int, current_position)  # Ép kiểu về int
        obstacle_count = 0
        radius = 3  # Phạm vi kiểm tra phù hợp cho local costmap (3 ô lưới ~ 0.3m)

        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                nx, ny = int(x + i), int(y + j)
                if 0 <= nx < width and 0 <= ny < height:
                    index = int(ny * width + nx)
                    if map_data[index] > 50:  # Giả sử giá trị > 50 là vật cản
                        obstacle_count += 1

        density = obstacle_count / ((2 * radius + 1) ** 2)

        # 3️⃣ Kết hợp mật độ vật cản và khoảng cách để điều chỉnh Goal Bias
        if density > 0.5:  # Mật độ cao
            goal_bias = 0.3
        elif density > 0.2:  # Mật độ trung bình
            goal_bias = 0.5
        else:  # Mật độ thấp
            goal_bias = 0.8

        # 4️⃣ Điều chỉnh theo khoảng cách đến mục tiêu
        if distance < 10:  # Rất gần mục tiêu (10 ô lưới ~ 1m)
            goal_bias = max(goal_bias, 0.9)
        elif distance < 20:  # Trung bình gần (2m)
            goal_bias = max(goal_bias, 0.8)
        elif distance > 40:  # Rất xa mục tiêu (4m)
            goal_bias = min(goal_bias, 0.5)

        return goal_bias


    # def adaptive_goal_bias(self, current_position: Tuple[int, int], 
    #                     target_position: Tuple[int, int], 
    #                     map_data: List[int], 
    #                     width: int, 
    #                     height: int) -> float:
    #     """
    #     Adaptive Goal Bias điều chỉnh dựa vào khoảng cách đến mục tiêu và mật độ vật cản.
    #     """
    #     # 1. Tính khoảng cách đến mục tiêu
    #     distance = self.calculate_distance(current_position, target_position)
        
    #     # 2. Tính mật độ vật cản xung quanh
    #     x, y = map(int, current_position)  # Ép kiểu về int
    #     obstacle_count = 0
    #     radius = 5  # Phạm vi kiểm tra chướng ngại vật

    #     for i in range(-radius, radius + 1):
    #         for j in range(-radius, radius + 1):
    #             nx, ny = int(x + i), int(y + j)  # Ép kiểu về int trước khi tính index
    #             if 0 <= nx < width and 0 <= ny < height:
    #                 index = int(ny * width + nx)  # Đảm bảo index là int
    #                 if map_data[index] == 1:  # Phát hiện vật cản
    #                     obstacle_count += 1

    #     density = obstacle_count / ((2 * radius + 1) ** 2)

    #     # 3. Kết hợp khoảng cách và mật độ
    #     if density > 0.5:  # Mật độ cao
    #         goal_bias = 0.3
    #     elif density > 0.2:  # Mật độ trung bình
    #         goal_bias = 0.5
    #     else:  # Mật độ thấp
    #         goal_bias = 0.7

    #     if distance < 50:  # Gần mục tiêu
    #         goal_bias = max(goal_bias, 0.8)
    #     elif distance > 200:  # Xa mục tiêu
    #         goal_bias = min(goal_bias, 0.4)

    #     return goal_bias

    def sample_point_towards_goal(self, current_position: Tuple[float, float], 
                                target_position: Tuple[float, float], 
                                width: int, 
                                height: int, 
                                goal_bias: float = 0.3) -> Tuple[int, int]:
        """
        Tạo điểm sample tối ưu dựa vào vector hướng đến đích.

        Args:
            current_position (Tuple[float, float]): Vị trí hiện tại của robot.
            target_position (Tuple[float, float]): Vị trí đích.
            width (int): Chiều rộng bản đồ.
            height (int): Chiều cao bản đồ.
            goal_bias (float): Xác suất hướng đến mục tiêu.

        Returns:
            Tuple[int, int]: Điểm sample mới dưới dạng số nguyên.
        """
        if random.random() < goal_bias:
            # Sampling hướng đến mục tiêu
            dx = target_position[0] - current_position[0]
            dy = target_position[1] - current_position[1]
            distance = np.hypot(dx, dy)  # Tính khoảng cách Euclidean

            if distance == 0:
                return int(target_position[0]), int(target_position[1])  # Ép kiểu về int

            # Vector hướng đến mục tiêu (chuẩn hóa)
            direction = (dx / distance, dy / distance)

            # Sampling gần theo vector hướng
            sample_x = current_position[0] + direction[0] * random.uniform(0, min(20, distance))
            sample_y = current_position[1] + direction[1] * random.uniform(0, min(20, distance))
        else:
            # Sampling ngẫu nhiên trên toàn bản đồ
            sample_x = random.uniform(0, width)
            sample_y = random.uniform(0, height)

        # Ép kiểu tọa độ thành số nguyên
        return int(sample_x), int(sample_y)

    def rrt_start(self, initial_position:Tuple[int, int], target_position:Tuple[int, int], width:int, height:int, map_data:List[int],  map_resolution:float,  tree_viz:Marker)->Tuple[Tuple[int, int], ...]:
        """
        Performs Rapidly-Exploring Random Tree Star (RRT*)
        """
        cubic_spline = CUBIC_SPLINE()
        root_node = Create_Node(coordinates = initial_position, cost = 0)

        # self.node.get_logger().info(f"root_node: {root_node}, initial_position: {initial_position}")
        tree = [root_node]
        iterations = 0 
        max_iterations = 500
        max_branch_length = 9
        goal_tolerance = 10   # 5*0.1 = 0.05M
        search_radius = 10 # cells
        closest_node = root_node
        while iterations < max_iterations:
            
            if iterations >= max_iterations:
                self.node.get_logger().warn("RRT: Max iterations exceeded")
                return [] 
            iterations += 1
            goal_bias = self.adaptive_goal_bias(current_position=closest_node.coordinates, target_position=target_position, map_data=map_data, width=width, height=height)
            random_point = self.sample_point_towards_goal(current_position=closest_node.coordinates, target_position=target_position, width=width, height=height, goal_bias=goal_bias)
            # random_point = self.sample_point_towards_goal(root_node.coordinates, target_position, width, height)



            if not self.is_free(random_point[0], random_point[1], map_data, width, height, map_resolution):
                continue

            # tree_viz.publish_sample_point_marker(random_point[0],random_point[1])
            
            closest_node = self.find_closest_node(random_point, tree)
            candidate_node = self.create_new_branch_point(closest_node, random_point, max_branch_length)
            if not self.collision_detected(closest_node.coordinates, candidate_node.coordinates, map_data, width):
                neighbor_node_list = self.find_neighbors(tree = tree, node = candidate_node,search_radius = search_radius) #,tree_viz = tree_viz

                if not neighbor_node_list:
                    continue 
                
                best_parent_node = closest_node 
                initial_line_cost = self.line_cost(closest_node, candidate_node)

                if not isinstance(initial_line_cost, (int, float)) or initial_line_cost < 0:
                    continue
                candidate_node.cost = closest_node.cost + initial_line_cost
               # Rewiring Step: Lựa chọn node cha tốt nhất 
                for neighbor_node in neighbor_node_list:
                    cond1 = not self.collision_detected(neighbor_node.coordinates, candidate_node.coordinates,  map_data, width) 
                    line_cost = self.line_cost(candidate_node, neighbor_node)
                    if not isinstance(line_cost, (int, float)) or line_cost < 0:
                        continue
                    trial_cost = neighbor_node.cost + line_cost 
                    cond2 = trial_cost < candidate_node.cost 
                    if (cond1 and cond2):  
                        best_parent_node = neighbor_node 
                        candidate_node.cost = trial_cost 

                if best_parent_node is not None:
                    candidate_node.parent = best_parent_node # Cập nhật cha cho Candidate Node

                # Rewire
                for neighbor_node in neighbor_node_list:
                    cond1 = not self.collision_detected(neighbor_node.coordinates, candidate_node.coordinates,  map_data, width)
                    line_cost = self.line_cost(candidate_node, neighbor_node)
                    trial_cost = candidate_node.cost + line_cost
                    cond2 = trial_cost < neighbor_node.cost 
                    if (cond1 and cond2): 
                        neighbor_node.cost = trial_cost
                        neighbor_node.parent = candidate_node

                tree.append(candidate_node)
                # tree_viz.append(candidate_node)
                                    
                if self.check_goal(candidate_node, target_position, goal_tolerance):
                    # self.node.get_logger().warn("RRT: Goal reached")
                    break
        
        # self.node.get_logger().warn("RRT: Path search ended")
        reconstruct_path = self.reconstruct_path(candidate_node=candidate_node, target_position=target_position)
        reconstruct_path = cubic_spline.cubic_spline_function(100,reconstruct_path)
        tree_viz.set_path(reconstruct_path)
        # self.node.get_logger().warn("RRT: Done reconstructing path")


        return  tuple(reconstruct_path)