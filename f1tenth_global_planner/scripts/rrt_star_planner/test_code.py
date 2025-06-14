#!/usr/bin/env python3

import math
import random
from bresenham import bresenham
from typing import List, Tuple, Optional
from visualization_msgs.msg import Marker
import numpy as np

# ================================
# 📚 Create_Node Class
# ================================
class Create_Node:
    def __init__(self, coordinates=None, parent=None, cost=None):
        self.coordinates = coordinates  # (x, y)
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
class RRTStar:
    def __init__(self, node):
        self.node = node

    # 📐 Calculate Distance
    def calculate_distance(self, point1: Tuple[int, int], point2: Tuple[int, int]) -> float:
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    # 📐 Calculate Angle
    def calculate_angle(self, p1: Tuple[int, int], p2: Tuple[int, int]) -> float:
        return math.atan2((p2[1] - p1[1]), (p2[0] - p1[0]))

    # 🌳 Find Closest Node
    def find_closest_node(self, random_pt: Tuple[int, int], node_list: List[Create_Node]) -> Create_Node:
        return min(node_list, key=lambda node: self.calculate_distance(node.coordinates, random_pt))

    # 🟢 Check if Cell is Free
    def is_free(self, cell_x: int, cell_y: int, map_data: List[int], width: int, height: int) -> bool:
        if cell_x < 0 or cell_x >= width or cell_y < 0 or cell_y >= height:
            return False
        index = cell_y * width + cell_x
        return map_data[index] == 0  # 0 đại diện cho ô trống

    # 💥 Collision Detection
    def collision_detected(self, p1: Tuple[int, int], p2: Tuple[int, int], map_data: List[int], map_width: int) -> bool:
        for cell in bresenham(p1[0], p1[1], p2[0], p2[1]):
            if cell[0] < 0 or cell[1] < 0 or cell[0] >= map_width or cell[1] * map_width >= len(map_data):
                continue
            if map_data[cell[1] * map_width + cell[0]] != 0:
                return True
        return False

    # 🌱 Create New Branch Point
    def create_new_branch_point(self, closest_node: Create_Node, p2: Tuple[int, int], max_distance: float) -> Optional[Create_Node]:
        try:
            d = self.calculate_distance(closest_node.coordinates, p2)
            theta = self.calculate_angle(closest_node.coordinates, p2)
            effective_distance = min(d, max_distance)
            new_x = closest_node.coordinates[0] + round(effective_distance * math.cos(theta), 2)
            new_y = closest_node.coordinates[1] + round(effective_distance * math.sin(theta), 2)
            return Create_Node(coordinates=(new_x, new_y), parent=closest_node, cost=closest_node.cost + effective_distance)
        except Exception as e:
            self.node.get_logger().error(f"Error in create_new_branch_point: {e}")
            return None

    # 🔄 Find Neighbors
    def find_neighbors(self, tree: List[Create_Node], node: Create_Node, search_radius: float) -> List[Create_Node]:
        return [n for n in tree if self.calculate_distance(n.coordinates, node.coordinates) <= search_radius]

    # 🚶 Reconstruct Path
    def reconstruct_path(self, candidate_node: Create_Node, target_position: Tuple[int, int]) -> List[Tuple[int, int]]:
        path = [target_position]
        node = candidate_node
        while node.parent:
            path.append(node.coordinates)
            node = node.parent
        path.append(node.coordinates)
        return path[::-1]

    # 📊 Adaptive Goal Bias
    def adaptive_goal_bias(self, current_position: Tuple[int, int], target_position: Tuple[int, int]) -> float:
        distance = self.calculate_distance(current_position, target_position)
        if distance < 10:
            return 0.8
        elif distance > 50:
            return 0.4
        return 0.5

    # 📍 Sample Point Towards Goal
    def sample_point_towards_goal(self, current_position: Tuple[int, int], target_position: Tuple[int, int], width: int, height: int, goal_bias: float) -> Tuple[int, int]:
        if random.random() < goal_bias:
            return target_position
        return (random.randint(0, width - 1), random.randint(0, height - 1))

    # 🌳 RRT* Main Function
    def rrt_start(self, initial_position: Tuple[int, int], target_position: Tuple[int, int], width: int, height: int, map_data: List[int], map_resolution: float, origin: Tuple[float, float]) -> List[Tuple[float, float]]:
        initial_grid = (
            int((initial_position[0] - origin[0]) / map_resolution),
            int((initial_position[1] - origin[1]) / map_resolution)
        )
        target_grid = (
            int((target_position[0] - origin[0]) / map_resolution),
            int((target_position[1] - origin[1]) / map_resolution)
        )
        root_node = Create_Node(coordinates=initial_grid, cost=0)
        tree = [root_node]
        iterations = 0
        max_iterations = 1000

        while iterations < max_iterations:
            iterations += 1
            random_point = self.sample_point_towards_goal(root_node.coordinates, target_grid, width, height, 0.5)
            
            if not self.is_free(random_point[0], random_point[1], map_data, width, height):
                continue

            closest_node = self.find_closest_node(random_point, tree)
            candidate_node = self.create_new_branch_point(closest_node, random_point, 3.0)

            if not self.collision_detected(closest_node.coordinates, candidate_node.coordinates, map_data, width):
                tree.append(candidate_node)
                if self.calculate_distance(candidate_node.coordinates, target_grid) < 1.0:
                    path = self.reconstruct_path(candidate_node, target_grid)
                    return [(x * map_resolution + origin[0], y * map_resolution + origin[1]) for x, y in path]

        return []
