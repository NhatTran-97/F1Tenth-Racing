#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import csv
import os
import numpy as np
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import math
from nav_msgs.msg import Odometry

home = expanduser('~/home/fablab_01')


class WaypointLogger(Node):

    def __init__(self) -> None:
        super().__init__('waypoint_logger')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
