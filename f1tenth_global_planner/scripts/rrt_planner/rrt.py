#!/usr/bin/env python3
import numpy as np
from numpy import linalg as LA
import math
from typing import List, Tuple
import random

class treeNode():
    def __init__(self, x, y, is_root = False):
        self.x = x
        self.y = y
        self.is_root  = is_root
        self.children = []      # list of children 
        self.parent   = None
        self.cost     = 0.0     # Cost from the root (start_node) to the node itself.
              
class RRTStarAlgorithm():
    def __init__(self, start, goal, interations, collision_margin, steer_length, goal_tolerance, grid):
        self.start_node = treeNode(start[0], start[1], True)       # The RRT (root position) (has 0 cost) # Set the start node as the root
        self.goal_node  = treeNode(goal[0], goal[1])               # Goal position (initialize to a high cost).                                 
        self.tree       = [self.start_node]
        self.iterations = min(interations, 1000)                    # Number of iterations to run.
        self.grid = grid                                           # x -> grid width | y -> grid height.
        self.margin = collision_margin                             # Width of car / 2 ==> for check collision free path => turn into the grid size
        self.Waypoints = []                                        # The waypoints -> new waypoints list for the car to follow.
        self.neighbouringNodes = []                                # Neighbouring nodes  
        self.steer_length = steer_length                           # Steering length limit -> limit the length of the nearest node and the sample node.
                                                                   # This value can be 1.0 meter.
        self.goal_tolerance = goal_tolerance                       # Limit to radius of the circle around the goal point.
        self.nearestDist = 10000                                   # distance to nearest node (initialize with large)
        self.numWaypoints = 0                                      # Number of waypoints
        self.path_distance = 0                                     # Total path distance               
        self.goalCosts = [10000]                                   # The costs to the goal (ignore first value)
        self.rewireCount = 0

    def sample(self):
        """
        This method should randomly sample the grid (map), and returns a viable point

        Args:
        Returns:
            point = np.array([x, y]): an array representing the sampled point

        """
        free_x, free_y = np.where(self.grid == 0)   # This might be changed --> because of the grid[0][1]        
        # If there are no free cells, return None or raise an exception -->
        # !NOTE: Might need to check the collision free in in the future.
        if len(free_x) == 0:
            print("No free space available in the grid!")
            return None
        # Randomly select an index from the list of free cells
        random_index = np.random.choice(len(free_x))
        x = free_x[random_index]
        y = free_y[random_index]
        point = np.array([x, y])
        
        return point #(x, y) # point

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point
        Args:
            tree ([]): the current RRT tree
            sampled_point (array of (int, int)): point sampled in the grid (map)
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node_idx = 0
        nearest_dist = 10000000 # create a huge number for in order to compare
        for idx in range(len(tree)):
            dist = self.Eulidean_dist(tree[idx].x, tree[idx].y, sampled_point[0], sampled_point[1])
            # print(f"Distance {idx}: {dist}")
            if dist < nearest_dist:
                nearest_node_idx = idx
                nearest_dist = dist
        # print(f"nearest_node_idx {nearest_node_idx}: {nearest_dist}")        
        return nearest_node_idx

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        new_node = treeNode(0,0)
        dist = self.Eulidean_dist(nearest_node.x, nearest_node.y, sampled_point[0], sampled_point[1])
        # print(f"Steer Length: {self.steer_length}")
        if dist <= self.steer_length: # The distance from the nearest node to the sampled point < the limit ==> can be used
            new_node.x = sampled_point[0]
            new_node.y = sampled_point[1]
            
        else: # If not ==> has to scale value to a certain distance.
            new_node.x = int(nearest_node.x + (self.steer_length)/dist * (sampled_point[0] - nearest_node.x))
            new_node.y = int(nearest_node.y + (self.steer_length)/dist * (sampled_point[1] - nearest_node.y))
            
        return new_node

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                            with the occupancy grid
                              
        """
        x_end, y_end = new_node.x, new_node.y
        x_start, y_start = nearest_node.x, nearest_node.y
        dy =   abs(y_end - y_start)
        dx =   abs(x_end - x_start)
        sx =   1 if x_start < x_end else -1
        sy =   1 if y_start < y_end else -1
        error = dx - dy 
        # Bounds checking
        def is_within_bounds(x, y):
            return 0 <= x < self.grid.shape[0] and 0 <= y < self.grid.shape[1]
        
        # Enlarge the surrounding of the cell by a margin.
        # Check collision with margin around the path
        def is_in_collision(x, y):
            # Define the margin range to check surrounding cells
            for i in range(-self.margin, self.margin + 1):
                for j in range(-self.margin, self.margin + 1):
                    xi, yj = x + i, y + j
                    if (is_within_bounds(xi, yj) and self.grid[xi, yj] == 100) or (is_within_bounds(xi, yj) and self.grid[xi, yj] == -1):
                        return True
            return False
        
        while (x_start != x_end) or (y_start != y_end):
            if not is_within_bounds(x_start, y_start):
                return True  # Consider out-of-bounds areas as collision 
            if self.grid[x_start, y_start] == 100:         
                return True
            if is_in_collision(x_start, y_start):
                return True  # Collision detected within the margin
            
            e2 = 2 * error
            if e2 > -dy:
                error -= dy
                x_start += sx
            if e2 < dx:
                error += dx
                y_start += sy
             # Final check at the endpoint
        if is_in_collision(x_end, y_end):
            return True
                    
        return False
    
    def check_collision_with_margin(self, nearest_node, new_node):
        pass

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
            
        """
        if (self.Eulidean_dist(latest_added_node.x, latest_added_node.y, goal_x, goal_y) <= self.goal_tolerance):
            return True
        return False

    def find_path(self, latest_added_node):
        path = []
        next_node = latest_added_node.parent
        
        # if latest_added_node is None:
        #     print("Error: latest_added_node is None")
        #     return None
        while not next_node.is_root:   
            path.append(next_node)
            if next_node.is_root:
                break
            next_node = next_node.parent  # Get the parent node from the tree
            print(f"Backtracking: current node ({next_node.x}, {next_node.y})")  # Debugging line
            print(f"Goal node: {self.goal_node.x, self.goal_node.y}")
            print(f"Root node: {self.start_node.x, self.start_node.y}")
            print(f"Parent of node: {next_node.parent.x},{next_node.parent.y}")
            print("END!")
        
        path.append(next_node)  # Add the root node
        path.reverse()  # Reverse to have the path from root to target node 
        
        # print(f"Path found with {len(path)} nodes")  # Debugging line
        return path
    
    def find_path_2(self, latest_added_node):
        """
        Backtrack from the latest_added_node to the start node using parent pointers.

        Args:
            latest_added_node (treeNode): The node that reached the goal.

        Returns:
            list: The path from start to goal (list of nodes).
        """
        path = []
        current_node = latest_added_node

        # Safety check to ensure current_node is valid
        if current_node is None:
            print("Error: latest_added_node is None")
            return None

        while current_node is not None:
            if current_node == current_node.parent:
                break
            path.append(current_node)
            # if current_node.parent is not None:
            #     print(f"Current node: ({current_node.x}, {current_node.y})")
            #     print(f"Parent node: ({current_node.parent.x}, {current_node.parent.y})")
            # else:
            #     print(f"Current node: ({current_node.x}, {current_node.y}) has no parent (reached root).")
            print("Stuck Here")
            # Check if we've reached the root node
            if current_node.is_root:
                break

            current_node = current_node.parent

        # Reverse the path to get it from start to goal
        path.reverse()
        print(f"Path found with {len(path)} nodes.")
        return path

    # The following methods are needed for RRT* and not RRT
    def cost(self, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """  
        if node.is_root:  # Root node
            return 0.0  # Root has zero cost
        
        return node.cost # + self.line_cost(tree[node.parent], node)

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2
        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return math.sqrt(pow((n1.x - n2.x),2) + pow((n1.y - n2.y),2))

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node
        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        
        for node_ in tree:
            if node_ != node:
                dist =  int(self.Eulidean_dist(node_.x, node_.y, node.x, node.y))
                # print(dist)
                # print(f"steer length {self.steer_length}")
                if dist <= self.steer_length:
                    neighborhood.append(node_)
                
        return neighborhood

    def Eulidean_dist(self, x1, y1, x2, y2):
        return math.sqrt(pow((x1 - x2),2) + pow((y1-y2),2))
    

