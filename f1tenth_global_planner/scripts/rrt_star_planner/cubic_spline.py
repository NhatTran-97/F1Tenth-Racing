#!/usr/bin/env python3

from scipy.interpolate import CubicSpline
import numpy as np
from rclpy.node import Node
from typing import List, Tuple


class CUBIC_SPLINE:
    def __init__(self):
        """
        Khởi tạo đối tượng Cubic Spline.
        """
        self.yaw = []  # Danh sách lưu giá trị góc hướng (Yaw)

    def cubic_spline_function(self, num_of_interpolate_point: int, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Tính toán Cubic Spline và nội suy đường đi.

        Args:
            num_of_interpolate_point (int): Số lượng điểm nội suy.
            path (List[Tuple[float, float]]): Danh sách các điểm điều khiển (x, y).

        Returns:
            List[Tuple[float, float]]: Danh sách các điểm (x, y) đã được nội suy.
        """
        if len(path) < 2:
            raise ValueError("Path must contain at least two points for interpolation.")

        # print("tétttttttttttt")

        # Tách x và y từ path
        x = np.array([point[0] for point in path])
        y = np.array([point[1] for point in path])

        # Tạo chỉ số thời gian t cho mỗi điểm điều khiển
        t = np.arange(len(x))

        # Tạo các spline cho x và y
        spline_x = CubicSpline(t, x)
        spline_y = CubicSpline(t, y)

        # Nội suy các điểm
        t_dense = np.linspace(t[0], t[-1], num_of_interpolate_point)
        x_dense = spline_x(t_dense)
        y_dense = spline_y(t_dense)

        # Tạo danh sách kết quả với định dạng List[Tuple[float, float]]
        interpolated_path = [(x_dense[i], y_dense[i]) for i in range(len(x_dense))]

        # Tính toán góc hướng (Yaw)
        yaw = []
        for i in range(len(x_dense) - 1):
            dy = y_dense[i + 1] - y_dense[i]
            dx = x_dense[i + 1] - x_dense[i]
            yaw.append(np.arctan2(dy, dx))

        # Thêm góc cuối cùng
        yaw.append(yaw[-1])
        self.yaw = yaw

        return interpolated_path
